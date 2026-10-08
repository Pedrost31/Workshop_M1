# -*- coding: utf-8 -*-
"""
robot_serveur.py — CÔTÉ ROBOT (Raspberry Pi du Yanshee, dans /home/pi)
=====================================================================

Rôle : attendre les ordres du PC de la caméra (Wi-Fi, port TCP 5005) et les
traduire en mouvements du Yanshee : "tourner de X degrés" puis "avancer de Y cm".

Même façon de piloter le robot que marche_obstacle.py de l'équipe :
  - import de YanAPI (avec le faux module lib_ukit, pour pouvoir tourner aussi sur le PC) ;
  - marche lancée avec start_play_motion("walk"), SURVEILLÉE pendant qu'elle se fait,
    arrêt par stop_play_motion() + "reset" ;
  - rotation avec sync_play_motion("turn around", "left"/"right") ;
  - capteurs lus chez capteurs_serveur.py (http://127.0.0.1:8080/capteurs) :
    ultrason "c" en cm (-1 = rien à moins de 4 m) et alertes gaz / température ;
  - état écrit dans /tmp/marche_etat.json => visible dans l'interface web de l'équipe.

Principe de la BOUCLE FERMÉE (un humanoïde dérive beaucoup) :
  le PC envoie la position du robot (vue par la caméra) + les waypoints ; le robot
  fait UN petit tronçon (30 cm maximum), répond, le PC le revoit et renvoie la suite.
Sécurité embarquée : obstacle à moins de SEUIL_CM devant, alerte gaz/température,
capteurs muets ou "stop" du PC => arrêt immédiat, et le PC est prévenu.

Compatible Python 3.5 (pas de f-strings).

Lancement SUR LE ROBOT (capteurs_serveur.py doit déjà tourner, NE PAS lancer
marche_obstacle.py en même temps : les deux commanderaient les jambes) :
    scp robot_serveur.py pi@10.124.7.2:/home/pi/
    ssh pi@10.124.7.2
    python3 robot_serveur.py                (vitesse "slow" par défaut)
    python3 robot_serveur.py normal         (vitesse : "very slow", "slow", "normal", "fast", "very fast")

MODE DISTANT (tout sur le PC, robot piloté par le Wi-Fi, rien à copier sur le robot,
comme le mode distant de marche_obstacle.py ; YanAPI.py à côté de ce fichier) :
    $env:ROBOT_IP="10.124.7.2"; $env:URL_CAPTEURS="http://10.124.7.2:8080/capteurs"; python robot_serveur.py
    (et dans liaison_robot.py : ROBOT_IP = "127.0.0.1")

Test sur le PC sans robot (ordres seulement affichés) :
    python robot_serveur.py --simulation

Format des messages (une ligne JSON par message, réponse = une ligne JSON) :
  PC -> robot : {"type": "waypoints", "pose": {"x": 30, "y": 40, "cap": 90},
                 "waypoints": [[30, 40], [30, 100], [100, 130]]}
                {"type": "stop"}   {"type": "ping"}
  robot -> PC : {"ok": true, "arrive": false, "ordre": {...}, "alerte": null}

Repère (le même que la carte du PC et que marche_obstacle.py) : x vers le marqueur 1,
y vers le marqueur 3, en cm ; cap 0° = vers le marqueur 1, cap 90° = vers le marqueur 3.
Un angle POSITIF = tourner à DROITE.
"""
import json
import math
import os
import socketserver
import sys
import tempfile
import threading
import time

try:
    from urllib.request import urlopen
except ImportError:                      # Python 2, au cas où
    from urllib2 import urlopen

# =============================== PARAMÈTRES ===============================
PORT = 5005                 # port d'écoute (le même que ROBOT_PORT dans liaison_robot.py)
ROBOT_IP = os.environ.get("ROBOT_IP", "127.0.0.1")       # API YanAPI (127.0.0.1 = ce robot)
URL_CAPTEURS = os.environ.get("URL_CAPTEURS", "http://127.0.0.1:8080/capteurs")  # capteurs_serveur.py

TRONCON_MAX_CM = 30         # distance maxi parcourue par ordre avant que la caméra revérifie
ARRIVEE_CM = 10             # waypoint considéré atteint à moins de 10 cm
ANGLE_TOLERE_DEG = 10       # écart de cap ignoré (évite de tourner pour rien)
ANGLE_SEUL_DEG = 45         # au-delà, le robot tourne SEULEMENT, puis la caméra revérifie son cap

# --- Sécurité (mêmes règles que marche_obstacle.py) ---
SEUIL_CM = 35               # arrêt si l'ultrason voit quelque chose à moins de 35 cm devant
CAPTEUR_OBLIGATOIRE = True  # capteurs_serveur.py muet ou capteur absent => le robot ne marche pas
PERIODE_S = 0.1             # surveillance des capteurs 10 fois par seconde pendant la marche
ECHECS_MAX = 3              # lectures ratées d'affilée tolérées avant l'arrêt
PAROLE = True               # le robot annonce à voix haute obstacle / alerte

# =================== À ADAPTER au robot réel (après test) ===================
VITESSE = "slow"            # "slow" : pas plus réguliers, donc distances plus précises
CM_PAR_PAS = 4.0            # distance d'un pas de "walk" à cette vitesse (mesurer 10 pas, diviser)
DEG_PAR_ROTATION = 30.0     # angle d'un mouvement "turn around" (à mesurer)
SECONDES_PAR_PAS = 1.0      # durée d'un pas (sécurité : temps maxi accordé à une marche)
SENS_ROTATION = 1           # mettre -1 si le robot tourne du mauvais côté
MOUVEMENT_TOURNER = "turn around"
# ===========================================================================

VITESSES = ("very slow", "slow", "normal", "fast", "very fast")
for _argument in sys.argv[1:]:
    if _argument in VITESSES:
        VITESSE = _argument

FICHIER_ETAT = os.path.join(tempfile.gettempdir(), "marche_etat.json")  # lu par capteurs_serveur.py


def normaliser_angle(a):
    """Ramène un angle entre -180 et +180 degrés."""
    return (a + 180.0) % 360.0 - 180.0


def calculer_ordre(pose, waypoints):
    """Traduit (position actuelle, waypoints) en UN ordre : tourner de X°, puis avancer de Y cm.
    Utilisée par le robot, et par le PC en mode simulation (liaison_robot.py)."""
    x, y, cap = float(pose["x"]), float(pose["y"]), float(pose["cap"])
    if not waypoints:
        return {"tourner_deg": 0.0, "avancer_cm": 0.0, "arrive": True, "cible": None}
    fin = waypoints[-1]
    if math.hypot(fin[0] - x, fin[1] - y) < ARRIVEE_CM:
        return {"tourner_deg": 0.0, "avancer_cm": 0.0, "arrive": True, "cible": fin}
    # Prochain waypoint pas encore atteint (le 1er est en général la position de départ)
    cible = fin
    for w in waypoints:
        if math.hypot(w[0] - x, w[1] - y) >= ARRIVEE_CM:
            cible = w
            break
    dx, dy = cible[0] - x, cible[1] - y
    distance = math.hypot(dx, dy)
    tourner = normaliser_angle(math.degrees(math.atan2(dy, dx)) - cap)
    if abs(tourner) < ANGLE_TOLERE_DEG:
        tourner = 0.0
    avancer = 0.0 if abs(tourner) > ANGLE_SEUL_DEG else min(distance, TRONCON_MAX_CM)
    return {"tourner_deg": round(tourner, 1), "avancer_cm": round(avancer, 1),
            "arrive": False, "cible": [round(cible[0], 1), round(cible[1], 1)]}


# ------------------------------------------------------------------ outils repris de marche_obstacle.py
def importer_yanapi():
    """import YanAPI, aussi sur le PC : YanAPI.py importe lib_ukit, un module qui
    n'existe que sur le robot et ne sert qu'aux accessoires uKit (inutilisés ici)."""
    try:
        import lib_ukit  # noqa: F401
    except ImportError:
        import types
        faux = types.ModuleType("lib_ukit")
        faux.lib_send = None
        sys.modules["lib_ukit"] = faux
    import YanAPI
    return YanAPI


def reussi(resultat):
    """Les fonctions YanAPI renvoient soit un booléen, soit {"code": 0, ...}."""
    if isinstance(resultat, dict):
        return resultat.get("code") == 0
    return bool(resultat)


journal = []

def publier(etat, message):
    """Écrit l'état pour l'interface web de l'équipe (même fichier et format que marche_obstacle.py)."""
    if message:
        print("[etat] {} : {}".format(etat, message))
        journal.append({"t": time.time(), "etat": etat, "message": message})
        del journal[:-15]
    donnees = {"etat": etat, "message": message, "c": None, "vitesse": VITESSE,
               "test": commande.simulation if commande else True, "t": time.time(),
               "journal": journal, "source": "robot_serveur (guidage camera)"}
    try:
        temporaire = FICHIER_ETAT + ".tmp"
        with open(temporaire, "w") as f:
            json.dump(donnees, f)
        os.replace(temporaire, FICHIER_ETAT)
    except (OSError, IOError):
        pass


class CommandeYanshee(object):
    """
    =====================  Partie qui parle au robot  =====================
    Reprend les appels YanAPI de marche_obstacle.py. En simulation, les ordres
    sont seulement affichés et les capteurs considérés comme "rien devant".
    """

    def __init__(self, simulation=False):
        self.simulation = simulation
        self.api = None
        if not simulation:
            try:
                self.api = importer_yanapi()
                self.api.yan_api_init(ROBOT_IP)
            except Exception as e:
                print("YanAPI inutilisable ({}) -> MODE SIMULATION. Sur le PC : copier YanAPI.py "
                      "depuis le robot et pip install nest_asyncio requests.".format(e))
                self.simulation = True
        print("Commande robot : {}  vitesse : {}  capteurs : {}".format(
            "SIMULATION (ordres affichés)" if self.simulation else "YanAPI " + ROBOT_IP, VITESSE,
            "ignorés (simulation)" if self.simulation else URL_CAPTEURS))

    # --- capteurs (capteurs_serveur.py) ---
    def lire_capteurs(self):
        """Dict des capteurs, ou None si serveur injoignable / mesure périmée."""
        if self.simulation:
            return {"c": -1, "alertes": []}
        try:
            donnees = json.loads(urlopen(URL_CAPTEURS, timeout=1).read().decode("utf-8"))
        except Exception as e:
            print("[capteurs] serveur injoignable : {}".format(e))
            return None
        if donnees.get("perime", True):
            print("[capteurs] mesure perimee (age={}) : l'Arduino n'envoie rien ?".format(donnees.get("age")))
            return None
        return donnees

    def danger(self):
        """None si on peut marcher, sinon la raison de l'arrêt (texte)."""
        donnees = None
        for _ in range(ECHECS_MAX):
            donnees = self.lire_capteurs()
            if donnees is not None:
                break
            time.sleep(PERIODE_S)
        if donnees is None:
            return "capteurs muets (capteurs_serveur.py lance ?)" if CAPTEUR_OBLIGATOIRE else None
        alertes = donnees.get("alertes") or []
        if alertes:
            return "alerte " + " ; ".join(a.get("message", a.get("type", "?")) for a in alertes)
        c = donnees.get("c")
        if c is None:
            return "pas de capteur ultrason centre" if CAPTEUR_OBLIGATOIRE else None
        if 0 <= c < SEUIL_CM:          # -1 = pas d'écho = libre ; 0 = mesure douteuse = prudence
            return "obstacle a {} cm devant".format(c)
        return None

    # --- mouvements ---
    def tourner(self, angle_deg, arret):
        """Tourne de angle_deg (positif = droite), une rotation "turn around" après l'autre."""
        angle_deg *= SENS_ROTATION
        n = int(round(abs(angle_deg) / DEG_PAR_ROTATION))
        sens = "right" if angle_deg > 0 else "left"
        print("  -> tourner {:+.0f} deg ({} x {} {})".format(angle_deg, n, MOUVEMENT_TOURNER, sens))
        for _ in range(n):
            if arret.is_set():
                return "stop demande par le PC"
            if self.simulation:
                time.sleep(0.3)
                continue
            resultat = self.api.sync_play_motion(name=MOUVEMENT_TOURNER, direction=sens, speed=VITESSE, repeat=1)
            if not reussi(resultat):
                return "le robot refuse de tourner : {}".format(resultat)
        return None

    def avancer(self, distance_cm, arret):
        """Marche de distance_cm en surveillant les capteurs 10 fois par seconde (comme
        marche_obstacle.py). Renvoie None si tout va bien, sinon la raison de l'arrêt."""
        n = max(1, int(round(distance_cm / CM_PAR_PAS)))
        print("  -> avancer {:.0f} cm ({} pas, vitesse {})".format(distance_cm, n, VITESSE))
        if self.simulation:
            time.sleep(0.2 * n)
            return "stop demande par le PC" if arret.is_set() else None
        resultat = self.api.start_play_motion(name="walk", direction="forward", speed=VITESSE, repeat=n)
        if not reussi(resultat):
            if "energy saving" in str(resultat):
                return "mode economie d'energie : batterie faible ?"
            return "le robot refuse de marcher : {}".format(resultat)
        debut = time.time()
        while time.time() - debut < n * SECONDES_PAR_PAS + 5:
            time.sleep(PERIODE_S)
            if arret.is_set():
                self.arreter()
                return "stop demande par le PC"
            probleme = self.danger()
            if probleme:
                self.arreter()
                return probleme
            if time.time() - debut > 2 and self.marche_terminee():
                return None
        self.arreter()
        return None

    def marche_terminee(self):
        """True si le robot a fini ses pas. En cas de doute, on considère qu'il marche."""
        try:
            etat = self.api.get_current_motion_play_state()
        except Exception:
            return False
        donnees = etat.get("data") if isinstance(etat, dict) else None
        return isinstance(donnees, dict) and donnees.get("status") == "idle"

    def arreter(self):
        print("  -> ARRET")
        if self.simulation:
            return
        try:
            self.api.stop_play_motion()
        except Exception as e:
            print("[robot] stop impossible : {}".format(e))
        try:
            self.api.sync_play_motion(name="reset")
        except Exception as e:
            print("[robot] reset impossible : {}".format(e))

    def parler(self, texte):
        print("[robot] " + texte)
        if PAROLE and not self.simulation:
            try:
                self.api.sync_do_tts(texte)
            except Exception as e:
                print("[robot] tts impossible : {}".format(e))


# ------------------------------------------------------------------ serveur réseau
commande = None
verrou_mouvement = threading.Lock()   # un seul mouvement à la fois
arret_demande = threading.Event()     # posé par un message "stop"


def executer(ordre):
    """Exécute un ordre (tourner puis avancer). Renvoie None ou la raison de l'arrêt."""
    arret_demande.clear()
    probleme = commande.danger()
    if probleme:
        return probleme
    publier("tourne" if ordre["tourner_deg"] else "marche",
            "Guidage camera : tourner {:+.0f} deg, avancer {:.0f} cm".format(ordre["tourner_deg"], ordre["avancer_cm"]))
    probleme = commande.tourner(ordre["tourner_deg"], arret_demande) if ordre["tourner_deg"] else None
    if probleme is None and ordre["avancer_cm"] > 0:
        time.sleep(0.3)                  # laisser l'ultrason voir la nouvelle direction
        probleme = commande.avancer(ordre["avancer_cm"], arret_demande)
    return probleme


def traiter(msg):
    t = msg.get("type")
    if t == "ping":
        return {"ok": True, "simulation": commande.simulation, "vitesse": VITESSE}
    if t == "stop":
        arret_demande.set()
        commande.arreter()
        publier("arret", "Stop demande par le PC")
        return {"ok": True}
    if t == "waypoints":
        ordre = calculer_ordre(msg["pose"], msg["waypoints"])
        print("Ordre : {}".format(ordre))
        if ordre["arrive"]:
            publier("termine", "Arrive a destination")
            commande.parler("Je suis arrive")
            return {"ok": True, "arrive": True, "ordre": ordre, "alerte": None}
        if not verrou_mouvement.acquire(False):
            return {"ok": False, "erreur": "robot deja en mouvement"}
        try:
            alerte = executer(ordre)
        finally:
            verrou_mouvement.release()
        if alerte:
            etat = "alerte" if alerte.startswith("alerte") else ("arret" if "stop" in alerte else "obstacle")
            publier(etat, alerte)
            if etat != "arret":
                commande.parler("Alerte" if etat == "alerte" else "Obstacle")
        return {"ok": True, "arrive": False, "ordre": ordre, "alerte": alerte}
    return {"ok": False, "erreur": "type de message inconnu : {}".format(t)}


class Gestionnaire(socketserver.StreamRequestHandler):
    def handle(self):
        ligne = self.rfile.readline()
        try:
            rep = traiter(json.loads(ligne.decode("utf-8")))
        except Exception as e:
            rep = {"ok": False, "erreur": "message invalide : {}".format(e)}
        self.wfile.write((json.dumps(rep) + "\n").encode("utf-8"))


class Serveur(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    commande = CommandeYanshee(simulation="--simulation" in sys.argv)
    serveur = Serveur(("0.0.0.0", PORT), Gestionnaire)
    publier("demarrage", "Guidage camera : en attente des ordres du PC (port {})".format(PORT))
    print("Serveur robot en attente sur le port {} (Ctrl+C pour arreter)".format(PORT))
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        commande.arreter()
        publier("arret", "Arret demande (Ctrl + C)")
        print("Serveur arrete.")
