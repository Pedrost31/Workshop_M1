# -*- coding: utf-8 -*-
"""Tests des decisions de marche_obstacle.py, sans robot : python -m pytest tests -v"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import marche_obstacle as mo  # noqa: E402


def test_obstacle():
    assert mo.obstacle({"c": 10}) is True
    assert mo.obstacle({"c": mo.SEUIL_CM}) is False
    assert mo.obstacle({"c": -1}) is False          # pas d'echo = libre
    assert mo.obstacle({"c": 0}) is True            # mesure douteuse : prudence
    assert mo.obstacle({}) is True                  # pas de capteur : on ne marche pas


def test_libre_plus_exigeant_qu_obstacle():
    c = (mo.SEUIL_CM + mo.SEUIL_LIBRE_CM) // 2
    assert mo.obstacle({"c": c}) is False
    assert mo.libre({"c": c}) is False
    assert mo.libre({"c": mo.SEUIL_LIBRE_CM}) is True
    assert mo.libre({"c": -1}) is True


def test_reussi():
    assert mo.reussi({"code": 0})
    assert not mo.reussi({"code": 1})
    assert mo.reussi(True) and not mo.reussi(False)


GAZ = {"type": "gaz", "niveau": "alerte", "message": "Gaz detecte : 200 (air normal 70)"}
CHAUD = {"type": "temperature", "niveau": "danger", "message": "Temperature DANGER : 49 C"}


def test_cle_alertes_ignore_la_valeur():
    autre_valeur = dict(GAZ, message="Gaz detecte : 230 (air normal 70)")
    assert mo.cle_alertes({"alertes": [GAZ]}) == mo.cle_alertes({"alertes": [autre_valeur]})
    assert mo.cle_alertes({"alertes": []}) == ""
    assert mo.cle_alertes({}) == ""


def test_phrase_alerte():
    assert mo.phrase_alerte({"alertes": [GAZ]}) == "Attention, gaz detecte"
    assert mo.phrase_alerte({"alertes": [GAZ, CHAUD]}) == \
        "Danger, gaz detecte et temperature trop elevee"


def test_lecture_tolerante(monkeypatch):
    reponses = [None, None, {"c": 50}]
    monkeypatch.setattr(mo, "lire_capteurs", lambda: reponses.pop(0))
    monkeypatch.setattr(mo.time, "sleep", lambda s: None)
    assert mo.lire_capteurs_tolerant(0) == ({"c": 50}, 0)


def test_lecture_tolerante_abandonne(monkeypatch):
    monkeypatch.setattr(mo, "lire_capteurs", lambda: None)
    monkeypatch.setattr(mo.time, "sleep", lambda s: None)
    donnees, echecs = mo.lire_capteurs_tolerant(0)
    assert donnees is None and echecs == mo.ECHECS_MAX


# --- mode --carte (webcam) ----------------------------------------------------

ZONE = {"largeur": 200, "hauteur": 150}


def carte(x, y, cap, obstacles=()):
    return {"zone": ZONE, "robot": {"x": x, "y": y, "cap": cap, "age": 0.1},
            "obstacles": list(obstacles)}


def test_milieu_de_zone_libre():
    assert mo.danger_carte(carte(100, 75, 0)) is None
    assert mo.danger_carte(carte(100, 75, 135)) is None


def test_bord_devant():
    # x=160 vers +x : 30 cm plus loin, x=190 > 200 - marge
    raison, sens = mo.danger_carte(carte(160, 75, 0))
    assert raison == "bord"
    # le centre (100, 75) est vers +y par rapport au cap -> a droite
    assert mo.danger_carte(carte(160, 60, 0))[1] == "right"
    assert mo.danger_carte(carte(160, 90, 0))[1] == "left"


def test_bord_derriere_ne_bloque_pas():
    assert mo.danger_carte(carte(170, 75, 180)) is None     # dos au bord


def test_dans_la_marge_mais_rentre_dans_la_zone():
    assert mo.danger_carte(carte(10, 75, 0)) is None         # pose trop pres du bord, repart vers le centre
    assert mo.danger_carte(carte(10, 75, 180))[0] == "bord"  # s'en va encore plus dehors


def test_obstacle_sur_la_carte():
    boite = {"x": 120, "y": 60, "l": 30, "h": 30}            # centre (135, 75)
    assert mo.danger_carte(carte(100, 75, 0, [boite]))[0] == "obstacle"
    assert mo.danger_carte(carte(100, 75, 180, [boite])) is None
    assert mo.danger_carte(carte(100, 120, 0, [boite])) is None   # passe a cote
    # obstacle un peu a droite (vers +y) du cap : on tourne a gauche
    assert mo.danger_carte(carte(100, 70, 0, [boite]))[1] == "left"
    assert mo.danger_carte(carte(100, 80, 0, [boite]))[1] == "right"


def test_sens_vers():
    robot = {"x": 0, "y": 0, "cap": 0}
    assert mo.sens_vers(robot, 10, 5) == "right"
    assert mo.sens_vers(robot, 10, -5) == "left"
    assert mo.sens_vers({"x": 0, "y": 0, "cap": 170}, -10, -5) == "right"   # passe par 180


class Reponse(object):
    def __init__(self, donnees):
        self.donnees = donnees

    def read(self):
        import json
        return json.dumps(self.donnees).encode("utf-8")


def test_lire_carte(monkeypatch):
    bonne = dict(carte(100, 75, 0), calibre=True)
    monkeypatch.setattr(mo, "urlopen", lambda url, timeout: Reponse(bonne))
    assert mo.lire_carte()["robot"]["x"] == 100
    monkeypatch.setattr(mo, "urlopen", lambda url, timeout: Reponse(dict(bonne, calibre=False)))
    assert mo.lire_carte() is None
    perdu = dict(bonne, robot=dict(bonne["robot"], age=mo.CARTE_AGE_MAX_S + 1))
    monkeypatch.setattr(mo, "urlopen", lambda url, timeout: Reponse(perdu))
    assert mo.lire_carte() is None
    monkeypatch.setattr(mo, "urlopen", lambda url, timeout: Reponse(dict(bonne, robot=None)))
    assert mo.lire_carte() is None
