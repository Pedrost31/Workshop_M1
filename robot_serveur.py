# -*- coding: utf-8 -*-
"""
robot_serveur.py — CÔTÉ ROBOT (à copier sur le Raspberry Pi du Yanshee)
======================================================================

Rôle : attendre les messages du PC (Wi-Fi, port TCP 5005), et les traduire en
ordres Yanshee : "tourner de X degrés" puis "avancer de Y cm".

Principe de la BOUCLE FERMÉE (un humanoïde dérive beaucoup en marchant) :
  - le PC envoie la position actuelle du robot (vue par la caméra) + les waypoints ;
  - le robot ne fait qu'UN PETIT TRONÇON (30 cm maximum) vers le prochain waypoint ;
  - il répond au PC, qui le revoit à la caméra, recalcule et renvoie la suite.
Sécurité : avant et pendant la marche, le capteur à ultrasons du robot est
consulté ; si un obstacle est trop proche, le robot s'arrête et prévient le PC.

Lancement :
  - sur le robot :              python3 robot_serveur.py
  - test sur le PC, sans robot : python robot_serveur.py --simulation
    (puis, dans liaison_robot.py : ROBOT_IP = "127.0.0.1" et SIMULATION = False)

Format des messages (une ligne JSON par message, réponse = une ligne JSON) :
  PC -> robot : {"type": "waypoints", "pose": {"x": 30, "y": 40, "cap": 90},
                 "waypoints": [[30, 40], [30, 100], [100, 130]]}
                {"type": "stop"}   {"type": "ping"}
  robot -> PC : {"ok": true, "arrive": false, "ordre": {...}, "alerte": null}

Repère (le même que la carte du PC) : x vers le marqueur 1, y vers le marqueur 3,
en cm ; cap 0° = vers le marqueur 1, cap 90° = vers le marqueur 3.
Un angle de rotation POSITIF = tourner à DROITE (vu du dessus).

NB : écrit sans "f-strings" pour rester compatible avec un vieux Python 3 du robot.
"""
import json
import math
import socketserver
import sys
import threading
import time

# =============================== PARAMÈTRES ===============================
PORT = 5005                 # port d'écoute (le même que ROBOT_PORT dans liaison_robot.py)
TRONCON_MAX_CM = 30         # distance maxi parcourue par ordre avant que la caméra revérifie
ARRIVEE_CM = 10             # waypoint considéré atteint à moins de 10 cm
ANGLE_TOLERE_DEG = 10       # écart de cap ignoré (évite de tourner pour rien)
ANGLE_SEUL_DEG = 45         # au-delà, le robot tourne SEULEMENT, puis la caméra revérifie son cap
SENS_ROTATION = 1           # mettre -1 si le robot tourne du mauvais côté

# =================== À ADAPTER au SDK Yanshee (après test) ===================
CM_PAR_PAS = 4.0            # distance parcourue par un pas de "walk" (à mesurer : 10 pas, diviser)
DEG_PAR_ROTATION = 30.0     # angle tourné par un mouvement "turn around" (à mesurer)
PAS_PAR_BLOC = 2            # nb de pas entre deux vérifications du capteur d'obstacle
DISTANCE_ARRET_MM = 150     # arrêt si le capteur ultrason voit un obstacle à moins de 15 cm
CAPTEUR_OBLIGATOIRE = False # True = refuser de marcher si le capteur ne répond pas
# ===========================================================================


def normaliser_angle(a):
    """Ramène un angle entre -180 et +180 degrés."""
    return (a + 180.0) % 360.0 - 180.0


def calculer_ordre(pose, waypoints):
    """Traduit (position actuelle, waypoints) en UN ordre : tourner de X°, puis avancer de Y cm.
    Utilisée par le robot, et par le PC en mode simulation."""
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


class CommandeYanshee(object):
    """
    =====================  À ADAPTER au SDK Yanshee  =====================
    SEULE partie qui parle au robot. Les appels YanAPI ci-dessous sont une
    première proposition à vérifier avec la documentation / un test réel.
    En simulation, les ordres sont seulement affichés.
    """

    def __init__(self, simulation=False):
        self.simulation = simulation
        self.api = None
        if not simulation:
            try:
                import YanAPI                      # bibliothèque fournie sur le Yanshee
                YanAPI.yan_api_init("127.0.0.1")   # le script tourne SUR le robot
                self.api = YanAPI
            except Exception as e:
                print("YanAPI indisponible ({}) -> MODE SIMULATION".format(e))
                self.simulation = True
        print("Commande robot : {}".format("SIMULATION (ordres affichés)" if self.simulation else "YanAPI"))

    def tourner(self, angle_deg, arret):
        """Tourne de angle_deg (positif = droite)."""
        angle_deg *= SENS_ROTATION
        n = int(round(abs(angle_deg) / DEG_PAR_ROTATION))
        sens = "right" if angle_deg > 0 else "left"
        print("  -> tourner {:+.0f} deg ({} x turn around {})".format(angle_deg, n, sens))
        if n == 0 or arret.is_set():
            return
        if self.simulation:
            time.sleep(0.3 * n)
            return
        # À ADAPTER : nom du mouvement / paramètres selon le SDK
        self.api.sync_play_motion(name="turn around", direction=sens, speed="slow", repeat=n)

    def avancer(self, distance_cm, arret):
        """Avance de distance_cm, par petits blocs, en vérifiant le capteur entre chaque bloc.
        Renvoie None si tout va bien, sinon le motif de l'arrêt."""
        n = int(round(distance_cm / CM_PAR_PAS))
        print("  -> avancer {:.0f} cm ({} pas)".format(distance_cm, n))
        while n > 0:
            if arret.is_set():
                return "stop demande par le PC"
            if self.obstacle_proche():
                self.arreter()
                return "obstacle detecte par le capteur du robot"
            bloc = min(n, PAS_PAR_BLOC)
            if self.simulation:
                time.sleep(0.2 * bloc)
            else:
                # À ADAPTER : nom du mouvement / paramètres selon le SDK
                self.api.sync_play_motion(name="walk", direction="forward", speed="slow", repeat=bloc)
            n -= bloc
        return None

    def arreter(self):
        print("  -> ARRET")
        if not self.simulation:
            try:
                self.api.stop_play_motion()        # À ADAPTER
            except Exception as e:
                print("Erreur arret : {}".format(e))

    def obstacle_proche(self):
        """Sécurité embarquée : True si le capteur ultrason voit quelque chose de trop proche."""
        if self.simulation:
            return False
        try:
            # À ADAPTER : format de réponse à vérifier (valeur en mm attendue)
            rep = self.api.get_sensors_ultrasound()
            valeurs = [c.get("value") for c in rep.get("data", {}).get("ultrasound", [])]
            valeurs = [v for v in valeurs if isinstance(v, (int, float)) and v > 0]
            return bool(valeurs) and min(valeurs) < DISTANCE_ARRET_MM
        except Exception as e:
            print("Capteur ultrason illisible ({})".format(e))
            return CAPTEUR_OBLIGATOIRE


# ------------------------------------------------------------------ serveur réseau
commande = None
verrou_mouvement = threading.Lock()   # un seul mouvement à la fois
arret_demande = threading.Event()     # posé par un message "stop"


def traiter(msg):
    t = msg.get("type")
    if t == "ping":
        return {"ok": True, "simulation": commande.simulation}
    if t == "stop":
        arret_demande.set()
        commande.arreter()
        return {"ok": True}
    if t == "waypoints":
        ordre = calculer_ordre(msg["pose"], msg["waypoints"])
        print("Ordre : {}".format(ordre))
        if ordre["arrive"]:
            return {"ok": True, "arrive": True, "ordre": ordre, "alerte": None}
        if not verrou_mouvement.acquire(False):
            return {"ok": False, "erreur": "robot deja en mouvement"}
        try:
            arret_demande.clear()
            if commande.obstacle_proche():
                return {"ok": True, "arrive": False, "ordre": ordre, "alerte": "obstacle detecte par le capteur du robot"}
            commande.tourner(ordre["tourner_deg"], arret_demande)
            alerte = commande.avancer(ordre["avancer_cm"], arret_demande) if ordre["avancer_cm"] > 0 else None
        finally:
            verrou_mouvement.release()
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
    print("Serveur robot en attente sur le port {} (Ctrl+C pour arreter)".format(PORT))
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        commande.arreter()
        print("Serveur arrete.")
