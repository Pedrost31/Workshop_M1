"""
liaison_robot.py — CÔTÉ PC : envoi des ordres au robot par le Wi-Fi
==================================================================

Utilisé par cartographie.py (fonction send_to_robot). Envoie des messages JSON
au script robot_serveur.py qui tourne sur le Raspberry Pi du Yanshee.

Trois façons de l'utiliser :
  1. SIMULATION = True (par défaut) : rien n'est envoyé, les ordres que le robot
     exécuterait sont affichés dans le terminal. Aucun robot nécessaire.
  2. Test réseau sur le PC seul : lancer dans un 2e terminal
         python robot_serveur.py --simulation
     puis mettre ROBOT_IP = "127.0.0.1" et SIMULATION = False ci-dessous.
  3. Robot réel : ROBOT_IP = adresse IP du Yanshee (affichée dans l'appli Yanshee
     ou par la commande "hostname -I" sur le robot), SIMULATION = False.

Test rapide de la liaison (sans caméra) :
    python liaison_robot.py
"""
import json
import socket
import threading
import time

# =============================== PARAMÈTRES ===============================
ROBOT_IP = "10.124.7.2"        # IP du Yanshee de l'équipe (même réseau Wi-Fi que le PC)
ROBOT_PORT = 5005              # doit être le même que PORT dans robot_serveur.py
SIMULATION = True              # True = pas de robot, les ordres sont seulement affichés
DELAI_CONNEXION_S = 3          # temps maxi pour joindre le robot
DELAI_REPONSE_S = 60           # temps maxi d'un mouvement (le robot répond quand il a fini)
# ==========================================================================


class LiaisonRobot:
    def __init__(self, ip=ROBOT_IP, port=ROBOT_PORT, simulation=SIMULATION):
        self.ip, self.port, self.simulation = ip, port, simulation
        self.occupe = False          # True pendant qu'un ordre est en cours d'exécution
        self._reponse = None
        print(f"Liaison robot : {'SIMULATION (aucun envoi)' if simulation else f'{ip}:{port}'}")

    def envoyer(self, message):
        """Envoie un message et attend la réponse du robot (bloquant). Ne lève jamais d'erreur :
        en cas de problème, renvoie {"ok": False, "erreur": ...}."""
        if self.simulation:
            return self._simuler(message)
        try:
            with socket.create_connection((self.ip, self.port), timeout=DELAI_CONNEXION_S) as s:
                s.settimeout(DELAI_REPONSE_S)
                s.sendall((json.dumps(message) + "\n").encode("utf-8"))
                data = b""
                while not data.endswith(b"\n"):
                    bloc = s.recv(4096)
                    if not bloc: break
                    data += bloc
            return json.loads(data.decode("utf-8")) if data.strip() else {"ok": False, "erreur": "reponse vide"}
        except (OSError, ValueError) as e:
            return {"ok": False, "erreur": f"robot injoignable ({self.ip}:{self.port}) : {e}"}

    def envoyer_async(self, message):
        """Envoie en arrière-plan (l'affichage de la carte continue pendant que le robot marche).
        La réponse se récupère ensuite avec recuperer_reponse()."""
        if self.occupe: return False
        self.occupe = True
        def tache():
            self._reponse = self.envoyer(message)
            self.occupe = False
        threading.Thread(target=tache, daemon=True).start()
        return True

    def recuperer_reponse(self):
        """Renvoie la dernière réponse du robot (une seule fois), ou None."""
        rep, self._reponse = self._reponse, None
        return rep

    def stop(self):
        """Arrêt immédiat, envoyé même si un mouvement est en cours."""
        threading.Thread(target=self.envoyer, args=({"type": "stop"},), daemon=True).start()

    def ping(self):
        return self.envoyer({"type": "ping"})

    def _simuler(self, message):
        """Ce que ferait le robot : on utilise le même calcul que robot_serveur.py."""
        from robot_serveur import calculer_ordre
        if message.get("type") != "waypoints":
            print(f"[SIMULATION] message {message.get('type')}")
            return {"ok": True, "simulation": True}
        ordre = calculer_ordre(message["pose"], message["waypoints"])
        print(f"[SIMULATION] ordre robot : tourner {ordre['tourner_deg']:+.0f} deg, "
              f"avancer {ordre['avancer_cm']:.0f} cm (cible {ordre['cible']}){'  -> ARRIVE' if ordre['arrive'] else ''}")
        time.sleep(0.5)   # temps "fictif" de mouvement
        return {"ok": True, "simulation": True, "arrive": ordre["arrive"], "ordre": ordre, "alerte": None}


if __name__ == "__main__":
    # Test sans caméra : robot en (20, 20) cm tourné vers le marqueur 1, trajet en L.
    lien = LiaisonRobot()
    print("ping ->", lien.ping())
    exemple = {"type": "waypoints", "pose": {"x": 20, "y": 20, "cap": 0},
               "waypoints": [[20, 20], [20, 100], [100, 130]]}
    print("waypoints ->", lien.envoyer(exemple))
