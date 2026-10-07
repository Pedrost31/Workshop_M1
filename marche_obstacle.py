#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
marche_obstacle.py - A LANCER SUR LE ROBOT (dans /home/pi)

Fait marcher le Yanshee en continu. Devant un obstacle, il s'arrete, tourne sur
place jusqu'a trouver un passage libre, puis repart.
Lit les capteurs via capteurs_serveur.py (qui doit tourner) :
    GET http://localhost:8080/capteurs -> {"c": 28, "g": -1, ..., "perime": false}

Compatible Python 3.5 (pas de f-strings).

    scp marche_obstacle.py pi@10.124.7.2:/home/pi/
    ssh pi@10.124.7.2
    python3 marche_obstacle.py              (vitesse normale, Ctrl + C pour arreter)
    python3 marche_obstacle.py fast         (vitesse : "very slow", "slow", "normal", "fast", "very fast")
    python3 marche_obstacle.py --arret      (s'arrete devant l'obstacle sans tourner, ancien comportement)
    python3 marche_obstacle.py --test       (affiche les decisions, ne marche pas)
"""

import json
import os
import random
import sys
import time

try:
    from urllib.request import urlopen
except ImportError:                      # Python 2, au cas ou
    from urllib2 import urlopen

URL_CAPTEURS = "http://localhost:8080/capteurs"
SEUIL_CM = 35            # obstacle si distance centre < 35 cm (marge car le robot marche en continu)
SEUIL_LIBRE_CM = 60      # apres une rotation, on ne repart que si c'est libre sur 60 cm
PAS_MAX = 20             # nombre de pas demandes a chaque depart en marche
TOURS_MAX = 6            # rotations maximum pour trouver un passage, sinon arret
DUREE_MAX_S = 120        # securite : duree maximum du programme
PERIODE_S = 0.1          # lecture des capteurs 10 fois par seconde
VITESSES = ("very slow", "slow", "normal", "fast", "very fast")
MOUVEMENT_TOURNER = "turn around"          # mouvement YanAPI, direction "left" ou "right"
NOMS_SENS = {"left": "gauche", "right": "droite"}

FICHIER_ETAT = "/tmp/marche_etat.json"     # lu par capteurs_serveur.py -> interface web

MODE_TEST = "--test" in sys.argv
MODE_ARRET = "--arret" in sys.argv
VITESSE = "normal"
for argument in sys.argv[1:]:
    if argument in VITESSES:
        VITESSE = argument

journal = []             # derniers evenements, affiches dans l'interface


def publier(etat, message, c=None):
    """Ecrit l'etat courant pour l'interface web. etat : demarrage, marche,
    obstacle, arret, refus, erreur, termine."""
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


def libre(donnees):
    """True si la voie est assez degagee pour repartir (plus exigeant qu'obstacle())."""
    c = donnees.get("c")
    return c is not None and (c < 0 or c >= SEUIL_LIBRE_CM)


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


def chercher_passage():
    """Tourne sur place, toujours du meme cote, jusqu'a voir la voie libre.
    True si un passage est trouve, False si bloque ou erreur."""
    sens = random.choice(("left", "right"))
    for tour in range(1, TOURS_MAX + 1):
        publier("tourne", "Tourne a %s (%d/%d)" % (NOMS_SENS[sens], tour, TOURS_MAX))
        if not tourner(sens):
            return False
        time.sleep(0.5)                  # laisser l'ultrason mesurer dans la nouvelle direction
        donnees = lire_capteurs()
        if donnees is None:
            publier("erreur", "Pas de mesure fiable : arret")
            return False
        print("[capteurs] apres rotation : c=%s cm" % donnees.get("c"))
        if libre(donnees):
            publier("libre", "Passage libre (%s cm), on repart" % donnees.get("c"), donnees.get("c"))
            return True
    publier("arret", "Aucun passage apres %d rotations : arret" % TOURS_MAX)
    parler("Je suis bloque")
    return False


def main():
    publier("demarrage", "Lancement, vitesse %s" % VITESSE)
    afficher_batterie()
    en_marche = False
    depart_marche = 0.0
    debut = time.time()
    while time.time() - debut < DUREE_MAX_S:
        donnees = lire_capteurs()
        if donnees is None:
            print("Arret : pas de mesure fiable.")
            publier("erreur", "Pas de mesure fiable : arret")
            break
        print("[capteurs] c=%s cm" % donnees.get("c"))

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
        import YanAPI
        YanAPI.yan_api_init("127.0.0.1")
    print("Vitesse : %s" % VITESSE)
    try:
        main()
    except KeyboardInterrupt:
        print("\nArret demande (Ctrl + C).")
        publier("arret", "Arret demande (Ctrl + C)")
        arreter_marche()
    except Exception as erreur:
        publier("erreur", "Erreur : %s" % erreur)
        raise
    publier("termine", "Programme termine")
