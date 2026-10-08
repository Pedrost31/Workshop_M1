#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capteurs_serveur.py - A LANCER SUR LE ROBOT (Raspberry Pi du Yanshee)

Lit les capteurs envoyes par l'Arduino sur le port serie USB, detecte les
alertes gaz / temperature et publie le tout en HTTP (port 8080) :
    GET /           interface web (interface.html)
    GET /capteurs   {"c": 87, "gaz": 312, "temp": 24, ..., "alertes": [...],
                     "gaz_ref": 70, "seuils": {...}, "age": 0.04, "perime": false}
    GET /distance   {"centre": 0.87, "gauche": null, "droite": null, "age": 0.04}
                    (metres ; 4.0 = pas d'echo = rien a moins de 4 m ; null = capteur absent)
    GET /batterie   batterie du robot (YanAPI)
    GET /marche     etat publie par marche_obstacle.py
    GET /carte      carte de la webcam, relayee depuis cartographie.py (CAMERA_HTTP)
    GET /carte.jpg  vue de dessus de la webcam ; POST /reference : photo du sol vide

Alertes (liste vide si tout va bien) :
    {"type": "gaz", "niveau": "alerte" | "danger", "message": "...", "valeur": 412}
    Gaz : le MQ-2 chauffe GAZ_CHAUFFE_S secondes, puis le niveau d'air normal
    (gaz_ref) est mesure pendant GAZ_CALIBRATION_S secondes. Alerte si la valeur
    depasse gaz_ref + GAZ_ECART_ALERTE, danger au-dela de gaz_ref + GAZ_ECART_DANGER.
    Temperature (DHT11) : alerte a TEMP_ALERTE_C, danger a TEMP_DANGER_C.

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

MODE DISTANT (sur le PC, sans rien changer sur le robot) : au lieu du port
serie, on lit le /capteurs du serveur deja lance sur le robot, et la batterie
via l'API du robot sur le Wi-Fi :
    SOURCE_HTTP=http://10.124.7.2:8080 ROBOT_IP=10.124.7.2 python3 capteurs_serveur.py
    (PowerShell : $env:SOURCE_HTTP="http://10.124.7.2:8080"; $env:ROBOT_IP="10.124.7.2"; python capteurs_serveur.py)

CARTE : cartographie.py tourne sur le PC de la webcam (port 8081). Si ce serveur
tourne sur le robot, lui donner l'adresse du PC :
    CAMERA_HTTP=http://IP_DU_PC:8081 python3 capteurs_serveur.py
"""

import glob
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

PORT_HTTP = int(os.environ.get("PORT_HTTP", "8080"))
BAUDS = 115200
SOURCE_HTTP = os.environ.get("SOURCE_HTTP", "").rstrip("/")   # mode distant, voir plus haut
ROBOT_IP = os.environ.get("ROBOT_IP", "127.0.0.1")            # pour YanAPI (batterie)
CAMERA_HTTP = os.environ.get("CAMERA_HTTP", "http://127.0.0.1:8081").rstrip("/")  # cartographie.py
PERIODE_DISTANTE_S = 0.05
# au-dela, la mesure est consideree perimee. Par le Wi-Fi (mode distant), le
# robot repond en ~0.2 s mais parfois en 1.1 s : on tolere donc plus.
AGE_MAX_S = float(os.environ.get("AGE_MAX_S", "1.5" if SOURCE_HTTP else "0.5"))

DOSSIER = os.path.dirname(os.path.abspath(__file__))
FICHIER_INTERFACE = os.path.join(DOSSIER, "interface.html")
FICHIER_MARCHE = os.path.join(tempfile.gettempdir(), "marche_etat.json")  # /tmp sur le robot ; ecrit par marche_obstacle.py
BATTERIE_CACHE_S = 10

# Ultrasons reellement cables. L'Arduino envoie toujours g=-1 d=-1 : sans cette
# liste, /distance les annoncerait "libres a 4 m" alors qu'ils n'existent pas.
ULTRASONS_BRANCHES = set(os.environ.get("ULTRASONS", "c").split(","))

GAZ_CHAUFFE_S = 60       # le MQ-2 donne des valeurs fausses tant qu'il chauffe
GAZ_CALIBRATION_S = 20   # puis on mesure l'air normal pendant 20 s -> gaz_ref
GAZ_ECART_ALERTE = 100   # gaz_ref + 100 -> alerte
GAZ_ECART_DANGER = 250   # gaz_ref + 250 -> danger
TEMP_ALERTE_C = 40
TEMP_DANGER_C = 48       # le DHT11 ne mesure que jusqu'a 50 C
HYSTERESIS_GAZ = 20      # une alerte ne cesse que nettement sous le seuil,
HYSTERESIS_TEMP = 1      # sinon elle clignote a chaque mesure

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


def distance(brut, cle):
    """Distance en metres du capteur cle ("c", "g" ou "d"), None s'il n'est pas cable."""
    return metres(brut.get(cle)) if cle in ULTRASONS_BRANCHES else None


# ---------------------------------------------------------------------------
# Alertes gaz / temperature
# ---------------------------------------------------------------------------

class DetecteurAlertes(object):
    """Suit les mesures successives (chauffe et calibration du MQ-2) et donne
    la liste des alertes en cours. Appele a chaque ligne recue de l'Arduino."""

    def __init__(self):
        self.debut = None            # premiere mesure de gaz recue
        self.calibration = []        # valeurs pendant la calibration
        self.gaz_ref = None          # niveau de l'air normal
        self.niveaux = {}            # {"gaz": "alerte", ...} : alertes en cours

    def gaz_etat(self, t):
        """"absent", "chauffe", "calibration" ou "pret"."""
        if self.debut is None:
            return "absent"
        if self.gaz_ref is not None:
            return "pret"
        return "chauffe" if t - self.debut < GAZ_CHAUFFE_S else "calibration"

    def chauffe_restante(self, t):
        if self.debut is None or self.gaz_ref is not None:
            return 0
        return max(0, int(round(self.debut + GAZ_CHAUFFE_S + GAZ_CALIBRATION_S - t)))

    def seuils(self):
        s = {"temp_alerte": TEMP_ALERTE_C, "temp_danger": TEMP_DANGER_C}
        if self.gaz_ref is not None:
            s["gaz_alerte"] = self.gaz_ref + GAZ_ECART_ALERTE
            s["gaz_danger"] = self.gaz_ref + GAZ_ECART_DANGER
        return s

    def _suivre_gaz(self, gaz, t):
        if self.debut is None:
            self.debut = t
        if self.gaz_etat(t) != "calibration":
            return
        self.calibration.append(gaz)
        if t - self.debut >= GAZ_CHAUFFE_S + GAZ_CALIBRATION_S:
            self.gaz_ref = int(round(sum(self.calibration) / float(len(self.calibration))))
            self.calibration = []
            print("[gaz] calibre : air normal = %d, alerte a %d, danger a %d"
                  % (self.gaz_ref, self.gaz_ref + GAZ_ECART_ALERTE,
                     self.gaz_ref + GAZ_ECART_DANGER))

    def _niveau(self, cle, valeur, alerte, danger, hysteresis):
        """Niveau avec hysteresis : on ne redescend qu'une fois nettement sous le seuil."""
        precedent = self.niveaux.get(cle)
        if valeur >= danger or (precedent == "danger" and valeur > danger - hysteresis):
            niveau = "danger"
        elif valeur >= alerte or (precedent and valeur > alerte - hysteresis):
            niveau = "alerte"
        else:
            niveau = None
        self.niveaux[cle] = niveau
        return niveau

    def mesurer(self, brut, t):
        """Renvoie la liste des alertes pour cette mesure."""
        alertes = []
        gaz = brut.get("gaz")
        if gaz is not None:
            self._suivre_gaz(gaz, t)
            if self.gaz_ref is not None:
                niveau = self._niveau("gaz", gaz, self.gaz_ref + GAZ_ECART_ALERTE,
                                      self.gaz_ref + GAZ_ECART_DANGER, HYSTERESIS_GAZ)
                if niveau:
                    alertes.append({"type": "gaz", "niveau": niveau, "valeur": gaz,
                                    "message": "Gaz %s : %d (air normal %d)"
                                               % ("DANGER" if niveau == "danger" else "detecte",
                                                  gaz, self.gaz_ref)})
        temp = brut.get("temp")
        if temp is not None:
            niveau = self._niveau("temp", temp, TEMP_ALERTE_C, TEMP_DANGER_C, HYSTERESIS_TEMP)
            if niveau:
                alertes.append({"type": "temperature", "niveau": niveau, "valeur": temp,
                                "message": "Temperature %s : %s C"
                                           % ("DANGER" if niveau == "danger" else "elevee", temp)})
        return alertes


detecteur = DetecteurAlertes()


def resume(alertes):
    """[("gaz", "alerte"), ...] : pour ne journaliser que les changements."""
    return [(a["type"], a["niveau"]) for a in alertes]


# ---------------------------------------------------------------------------
# Simulation : fausse fuite de gaz / surchauffe, pour tester la reaction du
# robot sans gaz ni chaleur. Les valeurs simulees remplacent les vraies mesures
# et passent par la meme detection d'alertes : le robot reagit pour de vrai.
#     POST /simuler?type=gaz            (ou type=temperature, type=stop)
#     options : pic=450  montee=5  palier=15  descente=5  (secondes)
# ---------------------------------------------------------------------------

SCENARIOS = {
    # cle mesuree, duree de montee / palier / descente (s), valeur de depart si
    # le capteur n'existe pas (le DHT11 n'est peut-etre pas branche)
    "gaz": {"cle": "gaz", "montee": 6, "palier": 15, "descente": 6, "defaut": None},
    "temperature": {"cle": "temp", "montee": 12, "palier": 15, "descente": 12, "defaut": 24},
}


class Simulateur(object):

    def __init__(self):
        self.en_cours = None

    def lancer(self, type_, t, pic=None, montee=None, palier=None, descente=None):
        """Demarre un scenario. Renvoie (True, etat) ou (False, raison)."""
        if type_ == "stop":
            self.en_cours = None
            return True, None
        if type_ not in SCENARIOS:
            return False, "type inconnu : gaz, temperature ou stop"
        s = dict(SCENARIOS[type_])
        if type_ == "gaz" and detecteur.gaz_ref is None:
            # sinon les fausses valeurs fausseraient la mesure de l'air normal
            return False, ("le MQ-2 n'est pas encore calibre (encore %d s)"
                           % detecteur.chauffe_restante(t))
        if pic is None:
            pic = detecteur.gaz_ref + GAZ_ECART_DANGER + 60 if type_ == "gaz" else 50
        for nom, valeur in (("montee", montee), ("palier", palier), ("descente", descente)):
            if valeur is not None:
                s[nom] = max(0.1, valeur)
        s.update(type=type_, pic=pic, debut=t)
        self.en_cours = s
        print("[simulation] %s : jusqu'a %s en %s s, palier %s s, descente %s s"
              % (type_, pic, s["montee"], s["palier"], s["descente"]))
        return True, self.etat(t)

    def _facteur(self, e):
        """0 -> 1 pendant la montee, 1 au palier, 1 -> 0 pendant la descente."""
        s = self.en_cours
        if e < s["montee"]:
            return e / float(s["montee"])
        if e < s["montee"] + s["palier"]:
            return 1.0
        return max(0.0, 1 - (e - s["montee"] - s["palier"]) / float(s["descente"]))

    def appliquer(self, brut, t):
        """Renvoie la mesure, avec la valeur simulee a la place de la vraie."""
        s = self.en_cours
        if s is None:
            return brut
        e = t - s["debut"]
        if e >= s["montee"] + s["palier"] + s["descente"]:
            print("[simulation] %s terminee" % s["type"])
            self.en_cours = None
            return brut
        reel = brut.get(s["cle"])
        base = reel if reel is not None else s["defaut"]
        if base is None:
            return brut
        brut = dict(brut)
        brut[s["cle"]] = int(round(base + (s["pic"] - base) * self._facteur(e)))
        return brut

    def etat(self, t):
        s = self.en_cours
        if s is None:
            return None
        reste = s["debut"] + s["montee"] + s["palier"] + s["descente"] - t
        return {"type": s["type"], "pic": s["pic"], "reste": max(0, int(round(reste)))}


simulateur = Simulateur()


def enregistrer(brut, t):
    """Nouvelle mesure (t = instant ou l'Arduino l'a envoyee) : alertes + etat partage."""
    with verrou:
        brut = simulateur.appliquer(brut, t)
        alertes = detecteur.mesurer(brut, t)
        simule = simulateur.etat(t)
        for a in alertes:
            if simule and a["type"] == simule["type"]:
                a["message"] += " (SIMULATION)"
                a["simulation"] = True
        if resume(alertes) != resume(etat.get("alertes", [])):
            print("[alerte] %s" % ("; ".join(a["message"] for a in alertes)
                                   or "fin des alertes"))
        etat["brut"] = brut
        etat["alertes"] = alertes
        etat["t"] = t
        etat["lignes"] += 1
        if etat["lignes"] % 200 == 1:
            print("[capteurs] %s  %s" % (etat["lignes"], brut))


# Cles ajoutees par un serveur (ancien ou nouveau) : ce ne sont pas des mesures
CLES_SERVEUR = ("age", "perime", "alertes", "gaz_etat", "gaz_ref", "gaz_chauffe_s", "seuils",
                "simulation")


def lire_distant():
    """Mode distant : lit le /capteurs du serveur qui tourne deja sur le robot."""
    try:
        from urllib.request import urlopen
    except ImportError:
        from urllib2 import urlopen
    url = SOURCE_HTTP + "/capteurs"
    print("[distant] lecture de %s" % url)
    dernier = None
    while True:
        try:
            donnees = json.loads(urlopen(url, timeout=1).read().decode("utf-8"))
            if dernier != "ok":
                print("[distant] robot joignable")
                dernier = "ok"
            if not donnees.get("perime", True):
                brut = dict((k, v) for k, v in donnees.items() if k not in CLES_SERVEUR)
                enregistrer(brut, time.time() - (donnees.get("age") or 0))
            # mesure perimee sur le robot : on n'enregistre rien, elle vieillit ici aussi
        except Exception as erreur:
            if dernier != "erreur":
                print("[distant] robot injoignable : %s" % erreur)
                dernier = "erreur"
            time.sleep(0.5)
        time.sleep(PERIODE_DISTANTE_S)


def lire_serie():
    while True:
        chemin = trouver_port()
        if not chemin:
            print("[serie] aucun Arduino (ttyACM*/ttyUSB*) : nouvelle tentative dans 2 s")
            time.sleep(2)
            continue
        print("[serie] lecture de %s (fabricant USB %s) a %d bauds"
              % (chemin, FABRICANTS_OK.get(fabricant(chemin), fabricant(chemin)), BAUDS))
        port = None
        try:
            port = ouvrir_port(chemin)
            fichier_brut = not hasattr(port, "in_waiting")   # pas pyserial
            tampon = b""
            while True:
                octet = port.read(1)
                if not octet:
                    # pyserial : simple delai depasse. Fichier brut : fin de
                    # fichier = Arduino debranche ; sans ce test on boucle a 100 % CPU.
                    if fichier_brut:
                        raise IOError("fin de fichier (Arduino debranche ?)")
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
                enregistrer(brut, time.time())
        except (OSError, IOError) as erreur:
            print("[serie] erreur %s, reconnexion dans 2 s" % erreur)
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass
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
                try:
                    import lib_ukit  # noqa: F401  (n'existe que sur le robot ; inutile ici)
                except ImportError:
                    import types
                    faux = types.ModuleType("lib_ukit")
                    faux.lib_send = None
                    sys.modules["lib_ukit"] = faux
                import YanAPI
                YanAPI.yan_api_init(ROBOT_IP)
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


def relayer_camera(chemin, methode="GET"):
    """Transmet la requete a cartographie.py. Renvoie (code, type, octets)."""
    try:
        from urllib.request import Request, urlopen
        from urllib.error import HTTPError
    except ImportError:
        from urllib2 import Request, urlopen, HTTPError
    requete = Request(CAMERA_HTTP + chemin, data=b"" if methode == "POST" else None)
    try:
        reponse = urlopen(requete, timeout=2)
        return reponse.getcode(), reponse.headers.get("Content-Type"), reponse.read()
    except HTTPError as erreur:
        return erreur.code, erreur.headers.get("Content-Type"), erreur.read()
    except Exception as erreur:
        corps = {"calibre": False, "robot": None, "obstacles": [],
                 "erreur": "cartographie.py injoignable sur %s (%s)" % (CAMERA_HTTP, erreur)}
        return 503, "application/json", json.dumps(corps).encode()


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        maintenant = time.time()
        with verrou:
            brut = dict(etat["brut"])
            alertes = list(etat.get("alertes", []))
            age = maintenant - etat["t"] if etat["t"] else None
            gaz = {"gaz_etat": detecteur.gaz_etat(maintenant), "gaz_ref": detecteur.gaz_ref,
                   "gaz_chauffe_s": detecteur.chauffe_restante(maintenant),
                   "seuils": detecteur.seuils(), "simulation": simulateur.etat(maintenant)}
        perime = age is None or age > AGE_MAX_S
        if self.path.startswith("/distance"):
            self._repondre(200, {"centre": distance(brut, "c"), "gauche": distance(brut, "g"),
                                 "droite": distance(brut, "d"), "age": age, "perime": perime})
        elif self.path.startswith("/capteurs"):
            brut.update(gaz)
            # mesure perimee : on ne garde pas une vieille alerte (ni une vieille absence d'alerte)
            brut.update({"alertes": [] if perime else alertes, "age": age, "perime": perime})
            self._repondre(200, brut)
        elif self.path.startswith("/batterie"):
            self._repondre(200, lire_batterie())
        elif self.path.startswith("/marche"):
            self._repondre(200, lire_marche())
        elif self.path.startswith("/carte"):
            self._envoyer(*relayer_camera(self.path))
        elif self.path == "/" or self.path.startswith("/interface"):
            self._envoyer_interface()
        else:
            self._repondre(404, {"erreur": "GET / , /capteurs , /distance , /batterie , /marche ou /carte"})

    def do_POST(self):
        """POST /simuler?type=gaz|temperature|stop[&pic=..&montee=..&palier=..&descente=..]"""
        chemin, _, requete = self.path.partition("?")
        if chemin == "/reference":
            self._envoyer(*relayer_camera(chemin, "POST"))
            return
        if chemin != "/simuler":
            self._repondre(404, {"erreur": "POST /simuler?type=gaz|temperature|stop"})
            return
        try:
            from urllib.parse import parse_qs
        except ImportError:
            from urlparse import parse_qs
        params = dict((k, v[-1]) for k, v in parse_qs(requete).items())
        nombres = {}
        for nom in ("pic", "montee", "palier", "descente"):
            if nom in params:
                nombres[nom] = nombre(params[nom])
                if nombres[nom] is None:
                    self._repondre(400, {"erreur": "%s doit etre un nombre" % nom})
                    return
        with verrou:
            ok, resultat = simulateur.lancer(params.get("type", ""), time.time(), **nombres)
        if ok:
            self._repondre(200, {"simulation": resultat})
        else:
            self._repondre(400, {"erreur": resultat})

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
        self._envoyer(code, "application/json", json.dumps(corps).encode())

    def _envoyer(self, code, type_, donnees):
        self.send_response(code)
        self.send_header("Content-Type", type_ or "application/octet-stream")
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
    threading.Thread(target=lire_distant if SOURCE_HTTP else lire_serie, daemon=True).start()
    serveur = ServeurMultiTache(("0.0.0.0", PORT_HTTP), Handler)
    print("[http] interface : http://0.0.0.0:%d/" % PORT_HTTP)
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
