#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ultrason_serveur.py - A LANCER SUR LE ROBOT (Raspberry Pi du Yanshee)

Lit les distances envoyees par l'Arduino sur le port serie USB et les publie
en HTTP :   GET http://<ip_robot>:8080/distance
        ->  {"centre": 0.87, "gauche": null, "droite": null, "age": 0.04}
            (metres ; 4.0 = pas d'echo = rien a moins de 4 m ; null = capteur absent)

Aucune dependance : utilise pyserial s'il est present, sinon lit le port
comme un fichier apres l'avoir configure avec stty.

ATTENTION : sur le Yanshee, /dev/ttyACM0 est le CONTROLEUR DE SERVOS (STM32,
fabricant USB 0483). Ce script le reconnait et l'exclut ; l'Arduino est choisi
par son identifiant USB. Pour forcer un port : PORT_ARDUINO=/dev/ttyACM1 python3 ...

INSTALLATION (depuis le Mac, robot sur le meme reseau)
    scp capteurs_serveur.py pi@IP_ROBOT:~
    ssh pi@IP_ROBOT
    pkill -f ultrason_serveur.py ; pkill -f capteurs_serveur.py
    nohup python3 capteurs_serveur.py > capteurs.log 2>&1 &
    curl http://localhost:8080/capteurs

Pour le lancer a chaque demarrage du robot :
    (crontab -l 2>/dev/null; echo "@reboot sleep 20 && cd /home/pi && python3 capteurs_serveur.py > capteurs.log 2>&1") | crontab -
"""

import glob
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

PORT_HTTP = 8080
BAUDS = 115200
AGE_MAX_S = 0.5          # au-dela, la mesure est consideree perimee

DOSSIER = os.path.dirname(os.path.abspath(__file__))
FICHIER_INTERFACE = os.path.join(DOSSIER, "interface.html")
FICHIER_MARCHE = "/tmp/marche_etat.json"     # ecrit par marche_obstacle.py
BATTERIE_CACHE_S = 10

etat = {"brut": {}, "t": 0.0, "lignes": 0}      # brut : {"c": 87, "gaz": 312, ...}
verrou = threading.Lock()


# ---------------------------------------------------------------------------
# Port serie
# ---------------------------------------------------------------------------

FABRICANTS_OK = {"2341": "Arduino", "2a03": "Arduino", "1a86": "CH340 (clone)",
                 "0403": "FTDI", "10c4": "CP210x"}
FABRICANTS_INTERDITS = {"0483": "STMicroelectronics = controleur de servos du Yanshee"}


def fabricant(tty):
    """Identifiant USB (idVendor) du peripherique derriere /dev/ttyXXX, ou None."""
    base = "/sys/class/tty/%s/device" % os.path.basename(tty)
    for niveau in ("..", "../..", "../../.."):
        chemin = os.path.join(base, niveau, "idVendor")
        try:
            with open(chemin) as f:
                return f.read().strip().lower()
        except OSError:
            continue
    return None


def trouver_port():
    """Le port de l'Arduino, choisi par identifiant USB et JAMAIS par ordre
    alphabetique : sur le Yanshee, /dev/ttyACM0 est le controleur de servos
    (STM32F407) et il ne faut surtout pas l'ouvrir."""
    force = os.environ.get("PORT_ARDUINO")
    if force:
        return force
    candidats = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    for tty in candidats:
        vid = fabricant(tty)
        if vid in FABRICANTS_INTERDITS:
            continue
        if vid in FABRICANTS_OK:
            return tty
    for tty in candidats:                       # inconnu mais pas interdit
        if fabricant(tty) not in FABRICANTS_INTERDITS:
            print("[serie] %s : fabricant %s inconnu, on tente" % (tty, fabricant(tty)))
            return tty
    return None


def ouvrir_port(chemin):
    """Renvoie un objet avec readline(). pyserial si dispo, sinon fichier brut."""
    try:
        import serial
        return serial.Serial(chemin, BAUDS, timeout=1)
    except ImportError:
        pass
    # sans pyserial : on configure la ligne avec stty puis on lit le device
    subprocess.call(["stty", "-F", chemin, str(BAUDS), "raw", "-echo", "-hupcl"])
    return open(chemin, "rb", buffering=0)


PORTEE_MAX_M = 4.0


def nombre(texte):
    try:
        return int(texte)
    except ValueError:
        try:
            return float(texte)
        except ValueError:
            return None


def analyser(ligne):
    """'c=87 g=-1 gaz=312' -> {"c": 87, "g": -1, "gaz": 312}.
    Accepte aussi l'ancien CSV '87,-1,-1' -> {"c": 87, "g": -1, "d": -1}."""
    ligne = ligne.strip()
    if "=" in ligne:
        brut = {}
        for morceau in ligne.split():
            cle, egal, valeur = morceau.partition("=")
            v = nombre(valeur) if egal else None
            if cle and v is not None:
                brut[cle] = v
        return brut or None
    champs = ligne.split(",")
    if len(champs) == 3:
        valeurs = [nombre(x) for x in champs]
        if None not in valeurs:
            return {"c": valeurs[0], "g": valeurs[1], "d": valeurs[2]}
    return None


def metres(cm):
    """Ultrason : cm brut -> metres. -1 (pas d'echo) = rien a moins de 4 m =
    LIBRE, donc PORTEE_MAX_M ; capteur absent (None) -> None."""
    if cm is None:
        return None
    return PORTEE_MAX_M if cm < 0 else cm / 100.0


def lire_serie():
    while True:
        chemin = trouver_port()
        if not chemin:
            print("[serie] aucun Arduino (ttyACM*/ttyUSB*) : nouvelle tentative dans 2 s")
            time.sleep(2)
            continue
        print("[serie] lecture de %s (fabricant USB %s) a %d bauds"
              % (chemin, FABRICANTS_OK.get(fabricant(chemin), fabricant(chemin)), BAUDS))
        try:
            port = ouvrir_port(chemin)
            tampon = b""
            while True:
                octet = port.read(1)
                if not octet:
                    continue
                if octet != b"\n":
                    tampon += octet
                    if len(tampon) > 240:
                        tampon = b""
                    continue
                brut = analyser(tampon.decode("ascii", "ignore"))
                tampon = b""
                if not brut:
                    continue
                with verrou:
                    etat["brut"] = brut
                    etat["t"] = time.time()
                    etat["lignes"] += 1
                    if etat["lignes"] % 200 == 1:
                        print("[serie] %s  %s" % (etat["lignes"], brut))
        except (OSError, IOError) as erreur:
            print("[serie] erreur %s, reconnexion dans 2 s" % erreur)
            time.sleep(2)


# ---------------------------------------------------------------------------
# Batterie (YanAPI, lue au plus toutes les 10 s) et etat de marche_obstacle.py
# ---------------------------------------------------------------------------

batterie = {"valeur": None, "t": 0.0, "api": None}
verrou_batterie = threading.Lock()


def lire_batterie():
    with verrou_batterie:
        if batterie["valeur"] is not None and time.time() - batterie["t"] < BATTERIE_CACHE_S:
            return batterie["valeur"]
        try:
            if batterie["api"] is None:
                import YanAPI
                YanAPI.yan_api_init("127.0.0.1")
                batterie["api"] = YanAPI
            valeur = batterie["api"].get_robot_battery_info()
        except Exception as erreur:
            valeur = {"erreur": "batterie illisible : %s" % erreur}
        batterie["valeur"] = valeur
        batterie["t"] = time.time()
        return valeur


def lire_marche():
    """Dernier etat publie par marche_obstacle.py, avec son age en secondes."""
    try:
        with open(FICHIER_MARCHE) as f:
            donnees = json.load(f)
    except (OSError, IOError, ValueError):
        return {"etat": "jamais lance", "age": None}
    donnees["age"] = time.time() - donnees.get("t", 0)
    return donnees


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        with verrou:
            brut = dict(etat["brut"])
            age = time.time() - etat["t"] if etat["t"] else None
        perime = age is None or age > AGE_MAX_S
        if self.path.startswith("/distance"):
            self._repondre(200, {"centre": metres(brut.get("c")), "gauche": metres(brut.get("g")),
                                 "droite": metres(brut.get("d")), "age": age, "perime": perime})
        elif self.path.startswith("/capteurs"):
            brut.update({"age": age, "perime": perime})
            self._repondre(200, brut)
        elif self.path.startswith("/batterie"):
            self._repondre(200, lire_batterie())
        elif self.path.startswith("/marche"):
            self._repondre(200, lire_marche())
        elif self.path == "/" or self.path.startswith("/interface"):
            self._envoyer_interface()
        else:
            self._repondre(404, {"erreur": "GET / , /capteurs , /distance , /batterie ou /marche"})

    def _envoyer_interface(self):
        try:
            with open(FICHIER_INTERFACE, "rb") as f:
                donnees = f.read()
        except (OSError, IOError):
            self._repondre(404, {"erreur": "interface.html absent de %s" % DOSSIER})
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(donnees)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(donnees)

    def _repondre(self, code, corps):
        donnees = json.dumps(corps).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(donnees)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(donnees)

    def log_message(self, *args):
        pass                                 # silence : 10 requetes/s


class ServeurMultiTache(ThreadingMixIn, HTTPServer):
    """Une requete lente (batterie, navigateur) ne bloque pas marche_obstacle.py."""
    daemon_threads = True


def main():
    threading.Thread(target=lire_serie, daemon=True).start()
    serveur = ServeurMultiTache(("0.0.0.0", PORT_HTTP), Handler)
    print("[http] interface : http://0.0.0.0:%d/" % PORT_HTTP)
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
