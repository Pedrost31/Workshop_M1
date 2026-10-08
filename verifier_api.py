#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verifier_api.py - verifie que le serveur envoie bien ce que l'interface affiche

Interroge chaque route de capteurs_serveur.py (lecture seule) et controle que
chaque champ lu par interface.html est present, du bon type et plausible.

    python verifier_api.py                          (robot 10.124.7.2:8080)
    python verifier_api.py http://127.0.0.1:8080    (serveur lance sur le PC)
    python verifier_api.py --robot 10.124.7.2       (ajoute l'API du robot, port 9090)

Code de sortie : 0 si aucune ERREUR, 1 sinon. Compatible Python 3.5.
"""

import json
import sys
import time

try:
    from urllib.request import urlopen
    from urllib.error import HTTPError
except ImportError:
    from urllib2 import urlopen, HTTPError

ADRESSE = "http://10.124.7.2:8080"
ECHANTILLONS = 10        # lectures de /capteurs pour voir si les mesures bougent
ETATS_GAZ = ("absent", "chauffe", "calibration", "pret")
ETATS_MARCHE = ("demarrage", "marche", "obstacle", "bord", "perdu", "tourne", "libre", "alerte",
                "arret", "refus", "erreur", "termine", "jamais lance")
# cle : (minimum, maximum) ; les capteurs analogiques de l'Arduino vont de 0 a 1023
PLAGES = {"gaz": (0, 1023), "vapeur": (0, 1023), "lum": (0, 1023),
          "temp": (0, 50), "hum": (0, 100)}      # DHT11 : 0-50 C, 0-100 %


def est_nombre(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


class Rapport(object):
    def __init__(self):
        self.erreurs = 0
        self.attentions = 0

    def ok(self, texte):
        print("  OK         %s" % texte)

    def attention(self, texte):
        self.attentions += 1
        print("  ATTENTION  %s" % texte)

    def erreur(self, texte):
        self.erreurs += 1
        print("  ERREUR     %s" % texte)


def lire(url, timeout=3):
    """(code, donnees JSON ou None, duree en s, message d'erreur ou None)."""
    debut = time.time()
    try:
        reponse = urlopen(url, timeout=timeout)
        code, corps = reponse.getcode(), reponse.read()
    except HTTPError as e:
        code, corps = e.code, e.read()
    except Exception as e:
        return None, None, time.time() - debut, str(e)
    duree = time.time() - debut
    try:
        return code, json.loads(corps.decode("utf-8")), duree, None
    except ValueError:
        return code, None, duree, "reponse qui n'est pas du JSON : %r" % corps[:80]


# ---------------------------------------------------------------------------
# Controles : chaque fonction recoit la reponse et le rapport
# ---------------------------------------------------------------------------

def verifier_capteurs(d, r):
    if not isinstance(d.get("perime"), bool):
        r.erreur("perime absent ou pas un booleen : %r" % d.get("perime"))
    elif d["perime"]:
        r.erreur("perime = true : l'Arduino n'envoie plus rien (age %s s). "
                 "L'interface affichera 'Mesure perimee'." % d.get("age"))
    else:
        r.ok("mesures fraiches (age %.2f s)" % d.get("age", 0))
    if d.get("age") is not None and not est_nombre(d["age"]):
        r.erreur("age n'est pas un nombre : %r" % d["age"])

    c = d.get("c")
    if c is None:
        r.erreur("c (ultrason centre) absent : radar vide et le robot refusera de marcher")
    elif not isinstance(c, int):
        r.erreur("c doit etre un entier en cm : %r" % c)
    elif c == -1:
        r.ok("ultrason : pas d'echo (rien a moins de 4 m)")
    elif c == 0:
        r.attention("ultrason a 0 cm : mesure douteuse, compte comme obstacle")
    elif 0 < c <= 400:
        r.ok("ultrason : %d cm" % c)
    else:
        r.attention("ultrason hors plage (-1 ou 1..400) : %r" % c)

    for cle in ("gaz", "vapeur", "lum", "temp", "hum"):
        v = d.get(cle)
        bas, haut = PLAGES[cle]
        if v is None:
            r.attention("%s absent : la carte affichera '—'" % cle)
        elif not est_nombre(v):
            r.erreur("%s n'est pas un nombre : %r" % (cle, v))
        elif not bas <= v <= haut:
            r.attention("%s = %s hors de la plage attendue %s..%s" % (cle, v, bas, haut))
        else:
            r.ok("%s = %s" % (cle, v))

    alertes = d.get("alertes")
    if not isinstance(alertes, list):
        r.erreur("alertes doit etre une liste : %r" % alertes)
    else:
        for a in alertes:
            manquants = [k for k in ("type", "niveau", "message") if k not in a]
            if manquants:
                r.erreur("alerte incomplete (manque %s) : %r" % (", ".join(manquants), a))
            elif a["niveau"] not in ("alerte", "danger"):
                r.erreur("niveau d'alerte inconnu : %r" % a["niveau"])
        if alertes:
            r.attention("alertes en cours : %s" % "; ".join(a.get("message", "?") for a in alertes))
        else:
            r.ok("aucune alerte")

    etat = d.get("gaz_etat")
    if etat not in ETATS_GAZ:
        r.erreur("gaz_etat inconnu : %r (attendu %s). Ancien serveur sur le robot ?"
                 % (etat, ", ".join(ETATS_GAZ)))
    elif etat == "pret" and not est_nombre(d.get("gaz_ref")):
        r.erreur("gaz_etat = pret mais gaz_ref absent : %r" % d.get("gaz_ref"))
    elif etat in ("chauffe", "calibration"):
        r.attention("MQ-2 en %s, encore %s s" % (etat, d.get("gaz_chauffe_s")))
    else:
        r.ok("gaz_etat = %s, air normal = %s" % (etat, d.get("gaz_ref")))
    if not isinstance(d.get("seuils"), dict) or "temp_alerte" not in d["seuils"]:
        r.erreur("seuils absent ou incomplet : %r" % d.get("seuils"))


def verifier_distance(d, r):
    for cle in ("centre", "gauche", "droite"):
        v = d.get(cle, "absent")
        if v == "absent":
            r.erreur("%s absent de /distance" % cle)
        elif v is not None and not est_nombre(v):
            r.erreur("%s doit etre un nombre (m) ou null : %r" % (cle, v))
    if est_nombre(d.get("centre")):
        r.ok("distance centre = %.2f m (gauche %s, droite %s)"
             % (d["centre"], d.get("gauche"), d.get("droite")))


def verifier_batterie(d, r):
    if "erreur" in d:
        r.attention("batterie illisible : %s" % d["erreur"])
        return
    data = d.get("data", d)
    pct = data.get("percent", data.get("capacity")) if isinstance(data, dict) else None
    if not est_nombre(pct):
        r.erreur("pourcentage absent (data.percent) : %r" % d)
    elif not 0 <= pct <= 100:
        r.erreur("batterie hors 0..100 : %s" % pct)
    else:
        r.ok("batterie %s %%%s" % (pct, ", en charge" if data.get("charging") else ""))
        if pct < 20:
            r.attention("batterie faible : le robot peut refuser de marcher")


def verifier_marche(d, r):
    etat = d.get("etat")
    if etat not in ETATS_MARCHE:
        r.erreur("etat inconnu de l'interface : %r" % etat)
    elif etat == "jamais lance":
        r.ok("marche_obstacle.py jamais lance (normal hors demo)")
    else:
        r.ok("etat = %s, il y a %s s" % (etat, int(d.get("age") or 0)))
        if not isinstance(d.get("journal", []), list):
            r.erreur("journal doit etre une liste")


def verifier_carte(code, d, r):
    if code == 503:
        r.attention("cartographie.py injoignable (normal sans webcam) : %s" % d.get("erreur"))
        return
    if code == 404:
        r.erreur("/carte inconnu : le serveur est l'ancienne version, le recopier sur le robot")
        return
    if not d.get("calibre"):
        r.attention("zone pas calibree : %s" % d.get("erreur"))
        return
    zone = d.get("zone") or {}
    if not (est_nombre(zone.get("largeur")) and est_nombre(zone.get("hauteur"))):
        r.erreur("zone incomplete : %r" % zone)
        return
    r.ok("zone %s x %s cm, %d obstacle(s)%s" % (zone["largeur"], zone["hauteur"],
                                                len(d.get("obstacles") or []),
                                                "" if d.get("reference") else ", sol vide pas photographie"))
    robot = d.get("robot")
    if not robot:
        r.attention("robot (marqueur 10) jamais vu")
    elif robot.get("age", 99) > 1.5:
        r.attention("robot perdu de vue depuis %s s" % robot.get("age"))
    elif not (0 <= robot["x"] <= zone["largeur"] and 0 <= robot["y"] <= zone["hauteur"]):
        r.attention("robot hors de la zone : x=%s y=%s" % (robot["x"], robot["y"]))
    else:
        r.ok("robot x=%s y=%s cm, cap %s" % (robot["x"], robot["y"], robot["cap"]))


def verifier_flux(base, r):
    """Plusieurs lectures : les mesures arrivent-elles en continu ?"""
    lignes, ages, durees, valeurs = [], [], [], {}
    for _ in range(ECHANTILLONS):
        code, d, duree, err = lire(base + "/capteurs")
        if d is None:
            r.erreur("lecture ratee pendant le suivi : %s" % err)
            return
        durees.append(duree)
        ages.append(d.get("age") or 0)
        for cle in ("c", "gaz", "lum"):
            valeurs.setdefault(cle, set()).add(d.get(cle))
        time.sleep(0.2)
    r.ok("%d lectures : requete %.0f ms en moyenne (max %.0f), age max %.2f s"
         % (ECHANTILLONS, 1000 * sum(durees) / len(durees), 1000 * max(durees), max(ages)))
    if max(durees) > 1.0:
        r.attention("requetes lentes (> 1 s) : Wi-Fi faible ? le robot reagira en retard")
    figees = [cle for cle, v in valeurs.items() if len(v) == 1 and None not in v]
    if len(figees) == len(valeurs):
        r.attention("c, gaz et lum n'ont pas bouge en %d lectures : passer la main devant "
                    "l'ultrason pour verifier que les mesures sont en direct" % ECHANTILLONS)


def main():
    arguments = [a for a in sys.argv[1:] if not a.startswith("--")]
    base = (arguments[0] if arguments else ADRESSE).rstrip("/")
    robot_ip = sys.argv[sys.argv.index("--robot") + 1] if "--robot" in sys.argv else None
    r = Rapport()
    print("Serveur : %s" % base)

    controles = (("/capteurs", verifier_capteurs), ("/distance", verifier_distance),
                 ("/batterie", verifier_batterie), ("/marche", verifier_marche))
    for chemin, verifier in controles:
        print("\nGET %s" % chemin)
        code, d, duree, err = lire(base + chemin, timeout=12 if chemin == "/batterie" else 3)
        if d is None:
            r.erreur("pas de reponse : %s" % err)
            if chemin == "/capteurs":
                print("\nServeur injoignable : robot allume ? meme Wi-Fi ? capteurs_serveur.py lance ?")
                return 1
            continue
        if code != 200:
            r.erreur("HTTP %s : %r" % (code, d))
            continue
        verifier(d, r)

    print("\nGET /carte")
    code, d, _, err = lire(base + "/carte")
    if d is None:
        r.erreur("pas de reponse : %s" % err)
    else:
        verifier_carte(code, d, r)

    print("\nSuivi de /capteurs (%d lectures)" % ECHANTILLONS)
    verifier_flux(base, r)

    if robot_ip:
        print("\nAPI du robot http://%s:9090" % robot_ip)
        code, d, _, err = lire("http://%s:9090/v1/devices/battery" % robot_ip)
        if d is None:
            r.erreur("injoignable : %s" % err)
        else:
            verifier_batterie(d, r)

    print("\nBilan : %d erreur(s), %d attention(s)" % (r.erreurs, r.attentions))
    return 1 if r.erreurs else 0


if __name__ == "__main__":
    sys.exit(main())
