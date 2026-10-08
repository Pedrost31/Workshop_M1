#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
marche_obstacle.py - A LANCER SUR LE ROBOT (dans /home/pi)

Fait marcher le Yanshee en continu. Devant un obstacle, il s'arrete, tourne sur
place jusqu'a trouver un passage libre, puis repart.
Si capteurs_serveur.py signale une alerte (gaz ou temperature), il s'arrete, la
dit a voix haute (repetee toutes les ALERTE_REPETITION_S secondes) et ne repart
que quand l'alerte a cesse.
Lit les capteurs via capteurs_serveur.py (qui doit tourner) :
    GET http://localhost:8080/capteurs -> {"c": 28, "g": -1, ..., "alertes": [], "perime": false}

Compatible Python 3.5 (pas de f-strings).

    scp marche_obstacle.py pi@10.124.7.2:/home/pi/
    ssh pi@10.124.7.2
    python3 marche_obstacle.py              (vitesse normale, Ctrl + C pour arreter)
    python3 marche_obstacle.py fast         (vitesse : "very slow", "slow", "normal", "fast", "very fast")
    python3 marche_obstacle.py --arret      (s'arrete devant l'obstacle sans tourner, ancien comportement)
    python3 marche_obstacle.py --test       (affiche les decisions, ne marche pas)
    python3 marche_obstacle.py --carte      (reste dans la zone filmee par la webcam, voir cartographie.py)

MODE DISTANT (sur le PC, robot pilote par le Wi-Fi, rien a changer sur le robot) :
    ROBOT_IP=10.124.7.2 python3 marche_obstacle.py slow
    (PowerShell : $env:ROBOT_IP="10.124.7.2"; python marche_obstacle.py slow)
    Il faut YanAPI.py a cote de ce fichier (copie depuis le robot, voir README),
    et capteurs_serveur.py lance en mode distant sur le PC.
"""

import json
import math
import os
import random
import sys
import tempfile
import time

try:
    from urllib.request import urlopen
except ImportError:                      # Python 2, au cas ou
    from urllib2 import urlopen

URL_CAPTEURS = os.environ.get("URL_CAPTEURS", "http://127.0.0.1:8080/capteurs")
ROBOT_IP = os.environ.get("ROBOT_IP", "127.0.0.1")     # adresse de l'API du robot (YanAPI)
SEUIL_CM = 35            # obstacle si distance centre < 35 cm (marge car le robot marche en continu)
SEUIL_LIBRE_CM = 60      # apres une rotation, on ne repart que si c'est libre sur 60 cm
PAS_MAX = 20             # nombre de pas demandes a chaque depart en marche
TOURS_MAX = 6            # rotations maximum pour trouver un passage, sinon arret
DUREE_MAX_S = int(os.environ.get("DUREE_MAX_S", "120"))   # securite : duree maximum du programme
PERIODE_S = 0.1          # lecture des capteurs 10 fois par seconde
ECHECS_MAX = 3           # lectures ratees d'affilee toleres avant l'arret
ALERTE_REPETITION_S = 10 # pendant une alerte, le robot la repete toutes les 10 s
VITESSES = ("very slow", "slow", "normal", "fast", "very fast")
MOUVEMENT_TOURNER = "turn around"          # mouvement YanAPI, direction "left" ou "right"
NOMS_SENS = {"left": "gauche", "right": "droite"}

# Mode --carte : la webcam (cartographie.py, relayee par capteurs_serveur.py) donne
# la position du robot dans la zone et les obstacles poses au sol.
URL_CARTE = os.environ.get("URL_CARTE", "http://127.0.0.1:8080/carte")
MARGE_BORD_CM = 25       # le robot reste a 25 cm au moins des bords de la zone
ANTICIPATION_CM = 30     # on regarde la carte sur les 30 cm devant le robot
RAYON_ROBOT_CM = 12      # les obstacles de la carte sont grossis de la demi-largeur du robot
CARTE_AGE_MAX_S = 1.5    # position plus vieille : la camera a perdu le robot de vue
TOURS_CARTE_MAX = 12     # assez de rotations pour faire demi-tour face a un bord

FICHIER_ETAT = os.path.join(tempfile.gettempdir(), "marche_etat.json")  # /tmp sur le robot ; lu par capteurs_serveur.py -> interface web

MODE_TEST = "--test" in sys.argv
MODE_ARRET = "--arret" in sys.argv
MODE_CARTE = "--carte" in sys.argv
VITESSE = "normal"
for argument in sys.argv[1:]:
    if argument in VITESSES:
        VITESSE = argument

def importer_yanapi():
    """import YanAPI, aussi sur le PC : YanAPI.py importe lib_ukit, un module qui
    n'existe que sur le robot et ne sert qu'aux accessoires uKit (inutilises ici)."""
    try:
        import lib_ukit  # noqa: F401
    except ImportError:
        import types
        faux = types.ModuleType("lib_ukit")
        faux.lib_send = None
        sys.modules["lib_ukit"] = faux
    import YanAPI
    return YanAPI


journal = []             # derniers evenements, affiches dans l'interface
dernier_etat = [None]


def publier(etat, message, c=None):
    """Ecrit l'etat courant pour l'interface web. etat : demarrage, marche,
    obstacle, bord, perdu, tourne, libre, alerte, arret, refus, erreur, termine."""
    dernier_etat[0] = etat
    if message:
        journal.append({"t": time.time(), "etat": etat, "message": message})
        del journal[:-15]
    donnees = {"etat": etat, "message": message, "c": c, "vitesse": VITESSE,
               "test": MODE_TEST, "t": time.time(), "journal": journal}
    try:
        temporaire = FICHIER_ETAT + ".tmp"
        with open(temporaire, "w") as f:
            json.dump(donnees, f)
        os.replace(temporaire, FICHIER_ETAT)
    except (OSError, IOError):
        pass


def lire_capteurs():
    """Renvoie le dict des capteurs, ou None si serveur absent ou mesure perimee."""
    try:
        reponse = urlopen(URL_CAPTEURS, timeout=1)
        donnees = json.loads(reponse.read().decode("utf-8"))
    except Exception as erreur:
        print("[capteurs] serveur injoignable : %s" % erreur)
        return None
    if donnees.get("perime", True):
        print("[capteurs] mesure perimee (age=%s) : l'Arduino n'envoie rien ?" % donnees.get("age"))
        return None
    return donnees


def obstacle(donnees):
    """True si quelque chose est a moins de SEUIL_CM devant.
    -1 = pas d'echo = rien a moins de 4 m = LIBRE.
    0 est considere comme un obstacle (mesure douteuse : prudence)."""
    c = donnees.get("c")
    if c is None:
        return True                      # pas de capteur centre : on ne marche pas
    if c < 0:
        return False
    return c < SEUIL_CM


def lire_capteurs_tolerant(echecs):
    """Comme lire_capteurs, mais une mesure ratee de temps en temps n'arrete pas
    le robot. Renvoie (donnees, echecs) ; donnees None = arreter."""
    while True:
        donnees = lire_capteurs()
        if donnees is not None:
            return donnees, 0
        echecs += 1
        if echecs >= ECHECS_MAX:
            return None, echecs
        time.sleep(PERIODE_S)


def texte_alertes(donnees):
    """Message des alertes en cours ("" si aucune)."""
    return " ; ".join(a.get("message", a.get("type", "?")) for a in donnees.get("alertes") or [])


def cle_alertes(donnees):
    """"gaz:alerte,temperature:danger" : identifie les alertes en cours sans
    leur valeur, pour ne reagir qu'aux changements ("" si aucune)."""
    return ",".join("%s:%s" % (a.get("type"), a.get("niveau"))
                    for a in donnees.get("alertes") or [])


def phrase_alerte(donnees):
    """Ce que le robot dit a voix haute."""
    types = [a.get("type") for a in donnees.get("alertes") or []]
    danger = any(a.get("niveau") == "danger" for a in donnees.get("alertes") or [])
    morceaux = []
    if "gaz" in types:
        morceaux.append("gaz detecte")
    if "temperature" in types:
        morceaux.append("temperature trop elevee")
    return ("Danger, " if danger else "Attention, ") + " et ".join(morceaux or ["alerte"])


def libre(donnees):
    """True si la voie est assez degagee pour repartir (plus exigeant qu'obstacle())."""
    c = donnees.get("c")
    return c is not None and (c < 0 or c >= SEUIL_LIBRE_CM)


def lire_carte():
    """Carte de la webcam, ou None si cartographie.py est injoignable, la zone pas
    calibree ou le robot pas vu depuis CARTE_AGE_MAX_S."""
    try:
        reponse = urlopen(URL_CARTE, timeout=1)
        carte = json.loads(reponse.read().decode("utf-8"))
    except Exception as erreur:
        print("[carte] injoignable : %s" % erreur)
        return None
    if not carte.get("calibre"):
        print("[carte] zone non calibree : %s" % carte.get("erreur"))
        return None
    robot = carte.get("robot")
    if not robot or robot.get("age") is None or robot["age"] > CARTE_AGE_MAX_S:
        print("[carte] robot hors de vue de la camera")
        return None
    return carte


def hors_zone(x, y, zone):
    """De combien (cm) le point (x, y) depasse de la zone autorisee (la zone
    moins MARGE_BORD_CM sur chaque bord) ; 0 s'il est dedans."""
    return max(0, MARGE_BORD_CM - x, x - (zone["largeur"] - MARGE_BORD_CM),
               MARGE_BORD_CM - y, y - (zone["hauteur"] - MARGE_BORD_CM))


def dans_obstacle(x, y, o):
    r = RAYON_ROBOT_CM
    return o["x"] - r <= x <= o["x"] + o["l"] + r and o["y"] - r <= y <= o["y"] + o["h"] + r


def sens_vers(robot, x, y):
    """"right" ou "left" : le cote ou tourner pour faire face au point (x, y).
    Sur la carte, y va vers le bas (marqueurs dans le sens des aiguilles d'une
    montre vus d'en haut) : un angle positif est un virage a droite."""
    ecart = math.degrees(math.atan2(y - robot["y"], x - robot["x"])) - robot["cap"]
    return "right" if (ecart + 180) % 360 - 180 > 0 else "left"


def danger_carte(carte):
    """Ce qui attend le robot sur les ANTICIPATION_CM devant lui : None si rien,
    sinon (raison, sens) avec raison "bord" ou "obstacle" et sens le cote ou
    tourner pour s'en eloigner."""
    robot, zone = carte["robot"], carte["zone"]
    cap = math.radians(robot["cap"])
    devant = [(robot["x"] + d * math.cos(cap), robot["y"] + d * math.sin(cap))
              for d in range(5, ANTICIPATION_CM + 1, 5)]
    # deja dans la marge mais en train d'en sortir vers l'interieur : on le laisse faire
    x, y = devant[-1]
    if hors_zone(x, y, zone) > 0 and hors_zone(x, y, zone) >= hors_zone(robot["x"], robot["y"], zone):
        return "bord", sens_vers(robot, zone["largeur"] / 2.0, zone["hauteur"] / 2.0)
    for o in carte.get("obstacles") or []:
        if dans_obstacle(robot["x"], robot["y"], o):
            continue                     # deja colle a l'obstacle : l'ultrason s'en occupe
        if any(dans_obstacle(x, y, o) for x, y in devant):
            cote = sens_vers(robot, o["x"] + o["l"] / 2.0, o["y"] + o["h"] / 2.0)
            return "obstacle", "left" if cote == "right" else "right"
    return None


def reussi(resultat):
    """Les fonctions YanAPI renvoient soit un booleen, soit {"code": 0, ...}."""
    if isinstance(resultat, dict):
        return resultat.get("code") == 0
    return bool(resultat)


def parler(texte):
    print("[robot] " + texte)
    if not MODE_TEST:
        try:
            YanAPI.sync_do_tts(texte)
        except Exception as erreur:
            print("[robot] tts impossible : %s" % erreur)


def afficher_batterie():
    if MODE_TEST:
        return
    try:
        print("[robot] batterie : %s" % YanAPI.get_robot_battery_info())
    except Exception as erreur:
        print("[robot] batterie illisible : %s" % erreur)


def demarrer_marche():
    """Lance la marche en continu (sans attendre la fin). True si le robot a accepte."""
    if MODE_TEST:
        print("[robot] (test) marche en avant, vitesse %s" % VITESSE)
        return True
    resultat = YanAPI.start_play_motion(name="walk", direction="forward",
                                        speed=VITESSE, repeat=PAS_MAX)
    if not reussi(resultat):
        print("[robot] le robot refuse de marcher : %s" % resultat)
        if "energy saving" in str(resultat):
            print("        -> mode economie d'energie : batterie faible ? Voir le message ci-dessus.")
            publier("refus", "Mode economie d'energie : le robot refuse de marcher")
        else:
            publier("refus", "Le robot refuse de marcher : %s" % resultat)
        return False
    return True


def arreter_marche():
    if MODE_TEST:
        print("[robot] (test) arret")
        return
    try:
        YanAPI.stop_play_motion()
    except Exception as erreur:
        print("[robot] stop impossible : %s" % erreur)
    repos()


def repos():
    if not MODE_TEST:
        try:
            YanAPI.sync_play_motion(name="reset")
        except Exception as erreur:
            print("[robot] reset impossible : %s" % erreur)


def marche_terminee():
    """True si le robot a fini ses PAS_MAX pas et s'est arrete de lui-meme.
    En cas de doute (fonction absente, reponse inconnue), on considere qu'il marche."""
    if MODE_TEST:
        return False
    try:
        etat = YanAPI.get_current_motion_play_state()
    except Exception:
        return False
    donnees = etat.get("data") if isinstance(etat, dict) else None
    return isinstance(donnees, dict) and donnees.get("status") == "idle"


def tourner(sens):
    """Une rotation sur place (attend la fin du mouvement). True si le robot a accepte."""
    if MODE_TEST:
        print("[robot] (test) tourne a %s" % NOMS_SENS[sens])
        time.sleep(1)
        return True
    resultat = YanAPI.sync_play_motion(name=MOUVEMENT_TOURNER, direction=sens,
                                       speed=VITESSE, repeat=1)
    if not reussi(resultat):
        print("[robot] le robot refuse de tourner : %s" % resultat)
        publier("refus", "Le robot refuse de tourner : %s" % resultat)
        return False
    return True


def chercher_passage(sens=None, tours_max=TOURS_MAX):
    """Tourne sur place, toujours du meme cote (au hasard si sens est None),
    jusqu'a voir la voie libre (ultrason, et carte en mode --carte).
    True si un passage est trouve, False si bloque ou erreur."""
    sens = sens or random.choice(("left", "right"))
    for tour in range(1, tours_max + 1):
        publier("tourne", "Tourne a %s (%d/%d)" % (NOMS_SENS[sens], tour, tours_max))
        if not tourner(sens):
            return False
        time.sleep(0.5)                  # laisser l'ultrason et la camera voir la nouvelle direction
        donnees, _ = lire_capteurs_tolerant(0)
        carte = lire_carte() if MODE_CARTE else None
        if donnees is None or (MODE_CARTE and carte is None):
            publier("erreur", "Pas de mesure fiable (capteurs ou camera) : arret")
            return False
        print("[capteurs] apres rotation : c=%s cm" % donnees.get("c"))
        if cle_alertes(donnees):
            return True                  # la boucle principale s'occupe de l'alerte
        if libre(donnees) and not (MODE_CARTE and danger_carte(carte)):
            publier("libre", "Passage libre (%s cm), on repart" % donnees.get("c"), donnees.get("c"))
            return True
    publier("arret", "Aucun passage apres %d rotations : arret" % tours_max)
    parler("Je suis bloque")
    return False


def main():
    publier("demarrage", "Lancement, vitesse %s%s" % (VITESSE, ", carte webcam" if MODE_CARTE else ""))
    afficher_batterie()
    if MODE_CARTE and lire_carte() is None:
        print("Arret : pas de carte. Lancer cartographie.py sur le PC de la webcam, "
              "marqueurs 0 a 3 et robot (marqueur 10) visibles. URL : %s" % URL_CARTE)
        publier("erreur", "Pas de carte : cartographie.py lance ? robot visible ?")
        return
    en_marche = False
    depart_marche = 0.0
    echecs = 0
    alerte_en_cours = ""
    derniere_annonce = 0.0
    debut = time.time()
    while time.time() - debut < DUREE_MAX_S:
        donnees, echecs = lire_capteurs_tolerant(echecs)
        if donnees is None:
            print("Arret : pas de mesure fiable (%d echecs d'affilee)." % echecs)
            publier("erreur", "Pas de mesure fiable : arret")
            break
        print("[capteurs] c=%s cm  gaz=%s  temp=%s"
              % (donnees.get("c"), donnees.get("gaz"), donnees.get("temp")))

        # 1. Alerte gaz / temperature : prioritaire sur tout le reste
        alerte = cle_alertes(donnees)
        if alerte:
            if en_marche:
                arreter_marche()
                en_marche = False
            nouvelle = alerte != alerte_en_cours
            # message seulement si l'alerte change (la valeur, elle, change a chaque mesure)
            publier("alerte", texte_alertes(donnees) if nouvelle else "", donnees.get("c"))
            if nouvelle or time.time() - derniere_annonce > ALERTE_REPETITION_S:
                parler(phrase_alerte(donnees))
                derniere_annonce = time.time()
            alerte_en_cours = alerte
            time.sleep(PERIODE_S)
            continue
        if alerte_en_cours:
            alerte_en_cours = ""
            publier("libre", "Fin de l'alerte, on repart", donnees.get("c"))
            parler("Alerte terminee")

        # 2. Mode --carte : robot vu par la camera, et loin des bords et des obstacles
        if MODE_CARTE:
            carte = lire_carte()
            if carte is None:
                if en_marche:
                    arreter_marche()
                    en_marche = False
                if dernier_etat[0] != "perdu":
                    publier("perdu", "Robot hors de vue de la camera : arret", donnees.get("c"))
                    parler("Je ne suis plus sur la carte")
                time.sleep(PERIODE_S)
                continue
            probleme = danger_carte(carte)
            if probleme:
                if en_marche:
                    arreter_marche()
                    en_marche = False
                robot = carte["robot"]
                if probleme[0] == "bord":
                    publier("bord", "Bord de la zone (x=%d, y=%d cm, cap %d)"
                            % (robot["x"], robot["y"], robot["cap"]), donnees.get("c"))
                    parler("Bord de la zone")
                else:
                    publier("obstacle", "Obstacle sur la carte (x=%d, y=%d cm)"
                            % (robot["x"], robot["y"]), donnees.get("c"))
                    parler("Obstacle")
                if MODE_ARRET or not chercher_passage(probleme[1], TOURS_CARTE_MAX):
                    return
                continue

        # 3. Obstacle devant (ultrason)
        if obstacle(donnees):
            if en_marche:
                arreter_marche()
                en_marche = False
            publier("obstacle", "Obstacle a %s cm" % donnees.get("c"), donnees.get("c"))
            parler("Obstacle")
            if MODE_ARRET or not chercher_passage():
                return
            continue

        if not en_marche:
            if not demarrer_marche():
                return
            en_marche = True
            depart_marche = time.time()
            publier("marche", "Marche en avant", donnees.get("c"))
        elif time.time() - depart_marche > 2 and marche_terminee():
            en_marche = False            # PAS_MAX pas faits : on relancera au tour suivant
        else:
            publier("marche", "", donnees.get("c"))
        time.sleep(PERIODE_S)
    else:
        print("Duree maximum atteinte (%d s)." % DUREE_MAX_S)
        publier("arret", "Duree maximum atteinte (%d s)" % DUREE_MAX_S)
    arreter_marche()


if __name__ == "__main__":
    if not MODE_TEST:
        try:
            YanAPI = importer_yanapi()
        except ImportError as erreur:
            sys.exit("YanAPI inutilisable (%s). Sur le PC : copier YanAPI.py depuis le robot "
                     "et pip install nest_asyncio requests (voir README)." % erreur)
        YanAPI.yan_api_init(ROBOT_IP)
    print("Vitesse : %s  robot : %s  capteurs : %s%s" % (VITESSE, ROBOT_IP, URL_CAPTEURS,
                                                       "  carte : " + URL_CARTE if MODE_CARTE else ""))
    try:
        main()
    except KeyboardInterrupt:
        print("\nArret demande (Ctrl + C).")
        publier("arret", "Arret demande (Ctrl + C)")
        arreter_marche()
    except Exception as erreur:
        publier("erreur", "Erreur : %s" % erreur)
        raise
    # on garde affichee la raison de l'arret (obstacle en mode --arret, refus, erreur...)
    if dernier_etat[0] not in ("obstacle", "bord", "perdu", "arret", "refus", "erreur", "alerte"):
        publier("termine", "Programme termine")
