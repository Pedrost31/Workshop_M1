#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ultrason_serveur.py - A LANCER SUR LE ROBOT (Raspberry Pi du Yanshee)

Lit les distances envoyees par l'Arduino sur le port serie USB et les publie
en HTTP :   GET http://<ip_robot>:8080/distance
        ->  {"centre": 0.87, "gauche": null, "droite": null, "age": 0.04}
            (metres ; null = capteur absent ou pas d'echo)

Aucune dependance : utilise pyserial s'il est present, sinon lit le port
comme un fichier apres l'avoir configure avec stty.

ATTENTION : sur le Yanshee, /dev/ttyACM0 est le CONTROLEUR DE SERVOS (STM32,
fabricant USB 0483). Ce script le reconnait et l'exclut ; l'Arduino est choisi
par son identifiant USB. Pour forcer un port : PORT_ARDUINO=/dev/ttyACM1 python3 ...

INSTALLATION (depuis le Mac, robot sur le meme reseau)
    scp ultrason_serveur.py pi@IP_ROBOT:~
    ssh pi@IP_ROBOT
    nohup python3 ultrason_serveur.py > ultrason.log 2>&1 &
    curl http://localhost:8080/distance

Pour le lancer a chaque demarrage du robot :
    (crontab -l 2>/dev/null; echo "@reboot sleep 20 && cd /home/pi && python3 ultrason_serveur.py > ultrason.log 2>&1") | crontab -
"""

import glob
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

PORT_HTTP = 8080
BAUDS = 115200
AGE_MAX_S = 0.5          # au-dela, la mesure est consideree perimee

etat = {"centre": None, "gauche": None, "droite": None, "t": 0.0, "lignes": 0}
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


def convertir(champ):
    try:
        cm = int(champ)
    except ValueError:
        return None
    return None if cm < 0 else cm / 100.0


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
                    if len(tampon) > 64:
                        tampon = b""
                    continue
                champs = tampon.decode("ascii", "ignore").strip().split(",")
                tampon = b""
                if len(champs) != 3:
                    continue
                with verrou:
                    etat["centre"] = convertir(champs[0])
                    etat["gauche"] = convertir(champs[1])
                    etat["droite"] = convertir(champs[2])
                    etat["t"] = time.time()
                    etat["lignes"] += 1
                    if etat["lignes"] % 200 == 1:
                        print("[serie] %s  centre=%s gauche=%s droite=%s"
                              % (etat["lignes"], etat["centre"], etat["gauche"], etat["droite"]))
        except (OSError, IOError) as erreur:
            print("[serie] erreur %s, reconnexion dans 2 s" % erreur)
            time.sleep(2)


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path.startswith("/distance"):
            with verrou:
                age = time.time() - etat["t"] if etat["t"] else None
                corps = {"centre": etat["centre"], "gauche": etat["gauche"],
                         "droite": etat["droite"], "age": age,
                         "perime": age is None or age > AGE_MAX_S}
            self._repondre(200, corps)
        else:
            self._repondre(404, {"erreur": "GET /distance"})

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


def main():
    threading.Thread(target=lire_serie, daemon=True).start()
    serveur = HTTPServer(("0.0.0.0", PORT_HTTP), Handler)
    print("[http] http://0.0.0.0:%d/distance" % PORT_HTTP)
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
