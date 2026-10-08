# -*- coding: utf-8 -*-
"""Tests de capteurs_serveur.py, sans robot ni Arduino : python -m pytest tests -v"""

import json
import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import capteurs_serveur as cs  # noqa: E402

try:
    from urllib.request import urlopen
except ImportError:
    from urllib2 import urlopen


# --- analyser / distances ---------------------------------------------------

def test_analyser_format_cle_valeur():
    ligne = "c=28 g=-1 d=-1 gaz=64 vapeur=26 lum=50 temp=24 hum=55\r"
    assert cs.analyser(ligne) == {"c": 28, "g": -1, "d": -1, "gaz": 64,
                                  "vapeur": 26, "lum": 50, "temp": 24, "hum": 55}


def test_analyser_ancien_csv():
    assert cs.analyser("87,-1,-1") == {"c": 87, "g": -1, "d": -1}


def test_analyser_ligne_illisible():
    assert cs.analyser("n'importe quoi") is None
    assert cs.analyser("") is None


def test_analyser_decimales():
    assert cs.analyser("temp_ntc=23.4")["temp_ntc"] == 23.4


def test_metres():
    assert cs.metres(87) == 0.87
    assert cs.metres(-1) == cs.PORTEE_MAX_M        # pas d'echo = libre
    assert cs.metres(None) is None


def test_distance_capteur_non_cable_est_none():
    brut = {"c": 87, "g": -1, "d": -1}
    assert cs.distance(brut, "c") == 0.87
    assert cs.distance(brut, "g") is None           # et pas 4.0 m "libre"
    assert cs.distance(brut, "d") is None


# --- alertes ----------------------------------------------------------------

def calibre(gaz_normal=70):
    """Un detecteur dont le MQ-2 a fini de chauffer et de se calibrer."""
    d = cs.DetecteurAlertes()
    fin = cs.GAZ_CHAUFFE_S + cs.GAZ_CALIBRATION_S
    t = 0.0
    while t <= fin:
        d.mesurer({"gaz": gaz_normal}, t)
        t += 0.5
    return d, t


def test_gaz_pas_d_alerte_pendant_la_chauffe():
    d = cs.DetecteurAlertes()
    assert d.mesurer({"gaz": 900}, 0) == []         # valeur enorme mais MQ-2 froid
    assert d.gaz_etat(10) == "chauffe"
    assert d.gaz_etat(cs.GAZ_CHAUFFE_S + 1) == "calibration"


def test_gaz_calibration_donne_la_reference():
    d, _ = calibre(70)
    assert d.gaz_ref == 70
    assert d.gaz_etat(1e6) == "pret"
    assert d.seuils()["gaz_alerte"] == 70 + cs.GAZ_ECART_ALERTE


def test_gaz_niveaux():
    d, t = calibre(70)
    assert d.mesurer({"gaz": 100}, t) == []
    a = d.mesurer({"gaz": 70 + cs.GAZ_ECART_ALERTE}, t)
    assert [(x["type"], x["niveau"]) for x in a] == [("gaz", "alerte")]
    a = d.mesurer({"gaz": 70 + cs.GAZ_ECART_DANGER}, t)
    assert [(x["type"], x["niveau"]) for x in a] == [("gaz", "danger")]


def test_gaz_hysteresis():
    d, t = calibre(70)
    seuil = 70 + cs.GAZ_ECART_ALERTE
    assert d.mesurer({"gaz": seuil}, t)              # alerte
    assert d.mesurer({"gaz": seuil - 5}, t)          # juste sous le seuil : toujours en alerte
    assert d.mesurer({"gaz": seuil - cs.HYSTERESIS_GAZ}, t) == []   # nettement dessous : fini


def test_temperature():
    d = cs.DetecteurAlertes()
    assert d.mesurer({"temp": 25}, 0) == []
    assert d.mesurer({"temp": cs.TEMP_ALERTE_C}, 0)[0]["niveau"] == "alerte"
    assert d.mesurer({"temp": cs.TEMP_DANGER_C}, 0)[0]["niveau"] == "danger"


def test_sans_capteur_gaz_ni_dht():
    d = cs.DetecteurAlertes()
    assert d.mesurer({"c": 30}, 0) == []
    assert d.gaz_etat(0) == "absent"


# --- HTTP -------------------------------------------------------------------

@pytest.fixture
def serveur():
    s = cs.ServeurMultiTache(("127.0.0.1", 0), cs.Handler)
    threading.Thread(target=s.serve_forever, daemon=True).start()
    yield "http://127.0.0.1:%d" % s.server_address[1]
    s.shutdown()
    s.server_close()


def get(url):
    return json.loads(urlopen(url, timeout=2).read().decode("utf-8"))


def test_http_capteurs_et_distance(serveur):
    with cs.verrou:
        cs.etat.update(brut={"c": 28, "g": -1, "d": -1, "gaz": 64},
                       alertes=[{"type": "gaz", "niveau": "alerte", "message": "x"}],
                       t=time.time())
    d = get(serveur + "/capteurs")
    assert d["c"] == 28 and d["perime"] is False
    assert d["alertes"][0]["type"] == "gaz"
    assert "seuils" in d and "gaz_etat" in d
    dist = get(serveur + "/distance")
    assert dist["centre"] == 0.28 and dist["gauche"] is None


def test_http_mesure_perimee_sans_alerte(serveur):
    with cs.verrou:
        cs.etat.update(brut={"c": 28}, alertes=[{"type": "gaz", "niveau": "danger",
                                                 "message": "x"}], t=time.time() - 10)
    d = get(serveur + "/capteurs")
    assert d["perime"] is True and d["alertes"] == []


# --- simulation -------------------------------------------------------------

def post(url):
    try:
        from urllib.request import Request
        reponse = urlopen(Request(url, data=b"", method="POST"), timeout=2)
        return reponse.getcode(), json.loads(reponse.read().decode("utf-8"))
    except Exception as erreur:          # HTTPError 400 : on lit quand meme le corps
        return erreur.code, json.loads(erreur.read().decode("utf-8"))


@pytest.fixture
def frais(monkeypatch):
    """Detecteur et simulateur neufs, MQ-2 deja calibre a 70."""
    d, _ = calibre(70)
    monkeypatch.setattr(cs, "detecteur", d)
    monkeypatch.setattr(cs, "simulateur", cs.Simulateur())
    return d


def test_simulation_gaz_refusee_avant_calibration(monkeypatch):
    monkeypatch.setattr(cs, "detecteur", cs.DetecteurAlertes())
    ok, raison = cs.Simulateur().lancer("gaz", 0)
    assert not ok and "calibre" in raison


def test_simulation_gaz_monte_tient_et_redescend(frais):
    sim = cs.simulateur
    ok, _ = sim.lancer("gaz", 0, pic=470, montee=10, palier=10, descente=10)
    assert ok
    assert sim.appliquer({"gaz": 70}, 5)["gaz"] == 270        # mi-montee
    assert sim.appliquer({"gaz": 70}, 15)["gaz"] == 470       # palier
    assert sim.appliquer({"gaz": 70}, 25)["gaz"] == 270       # mi-descente
    assert sim.appliquer({"gaz": 70}, 31) == {"gaz": 70}      # fini : vraie valeur
    assert sim.en_cours is None


def test_simulation_temperature_sans_dht11(frais):
    sim = cs.simulateur
    sim.lancer("temperature", 0, montee=1, palier=5, descente=1)
    assert sim.appliquer({"c": 50}, 2)["temp"] == 50          # cree la mesure, 24 -> 50


def test_simulation_declenche_les_alertes(frais):
    cs.simulateur.lancer("gaz", 0, montee=1, palier=10, descente=1)
    cs.enregistrer({"gaz": 70}, 5)
    alertes = cs.etat["alertes"]
    assert alertes[0]["type"] == "gaz" and alertes[0]["niveau"] == "danger"
    assert alertes[0]["simulation"] and "SIMULATION" in alertes[0]["message"]


def test_http_simuler(serveur, frais):
    code, d = post(serveur + "/simuler?type=temperature&pic=45")
    assert code == 200 and d["simulation"]["type"] == "temperature"
    with cs.verrou:
        cs.etat.update(t=time.time())
    assert get(serveur + "/capteurs")["simulation"]["pic"] == 45
    assert post(serveur + "/simuler?type=stop") == (200, {"simulation": None})
    assert post(serveur + "/simuler?type=feu")[0] == 400
    assert post(serveur + "/simuler?type=gaz&pic=abc")[0] == 400


# --- relais de la carte (cartographie.py) -------------------------------------

@pytest.fixture
def fausse_camera(monkeypatch):
    """Un faux cartographie.py qui repond a /carte et /reference."""
    from http.server import BaseHTTPRequestHandler, HTTPServer

    class Camera(BaseHTTPRequestHandler):
        def do_GET(self):
            self._envoyer(json.dumps({"calibre": True, "chemin": self.path}).encode())

        def do_POST(self):
            self._envoyer(json.dumps({"message": "photo du sol vide enregistree"}).encode())

        def _envoyer(self, donnees):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(donnees)))
            self.end_headers()
            self.wfile.write(donnees)

        def log_message(self, *args):
            pass

    s = HTTPServer(("127.0.0.1", 0), Camera)
    threading.Thread(target=s.serve_forever, daemon=True).start()
    monkeypatch.setattr(cs, "CAMERA_HTTP", "http://127.0.0.1:%d" % s.server_address[1])
    yield
    s.shutdown()
    s.server_close()


def test_http_carte_relayee(serveur, fausse_camera):
    d = get(serveur + "/carte")
    assert d["calibre"] is True and d["chemin"] == "/carte"
    reponse = urlopen(serveur + "/reference", data=b"", timeout=2)       # POST
    assert "enregistree" in json.loads(reponse.read().decode("utf-8"))["message"]


def test_http_carte_camera_absente(serveur, monkeypatch):
    try:
        from urllib.error import HTTPError
    except ImportError:
        from urllib2 import HTTPError
    monkeypatch.setattr(cs, "CAMERA_HTTP", "http://127.0.0.1:9")     # personne n'ecoute
    with pytest.raises(HTTPError) as erreur:
        urlopen(serveur + "/carte", timeout=5)
    assert erreur.value.code == 503
    assert "cartographie.py injoignable" in json.loads(erreur.value.read().decode("utf-8"))["erreur"]
