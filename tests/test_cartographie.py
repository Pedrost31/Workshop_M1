# -*- coding: utf-8 -*-
"""Tests de cartographie.py, sans webcam : images de synthese. python -m pytest tests -v"""

import math
import os
import sys

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cartographie as ca  # noqa: E402

LARGEUR, HAUTEUR = 200.0, 150.0
PX = 4                    # sol de synthese : 4 px par cm
COTE_CM = 16              # marqueurs de 16 cm
DICO = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def coller_marqueur(sol, ident, x, y, cap=0):
    """Marqueur centre en (x, y) cm, son haut tourne vers le cap (degres)."""
    cote = COTE_CM * PX
    marqueur = cv2.cvtColor(cv2.aruco.generateImageMarker(DICO, ident, cote), cv2.COLOR_GRAY2BGR)
    # le haut du marqueur regarde vers -y ; on le tourne pour qu'il regarde vers le cap
    rotation = cv2.getRotationMatrix2D((cote / 2.0, cote / 2.0), -(cap + 90), 1.0)
    rotation[:, 2] += (x * PX - cote / 2.0, y * PX - cote / 2.0)
    masque = cv2.warpAffine(np.full((cote, cote), 255, np.uint8), rotation, sol.shape[1::-1])
    tourne = cv2.warpAffine(marqueur, rotation, sol.shape[1::-1])
    sol[masque > 0] = tourne[masque > 0]


def scene(robot=None, obstacle=None):
    """Sol gris clair avec la zone, vu par une camera un peu de biais (1280x720)."""
    marge = 20
    sol = np.full((int((HAUTEUR + 2 * marge) * PX), int((LARGEUR + 2 * marge) * PX), 3), 200, np.uint8)
    for ident, (x, y) in zip(ca.COINS, [(0, 0), (LARGEUR, 0), (LARGEUR, HAUTEUR), (0, HAUTEUR)]):
        cote = COTE_CM * PX
        cv2.rectangle(sol, (int((x + marge) * PX - cote * 0.7), int((y + marge) * PX - cote * 0.7)),
                      (int((x + marge) * PX + cote * 0.7), int((y + marge) * PX + cote * 0.7)),
                      (255, 255, 255), cv2.FILLED)       # bord blanc autour du marqueur
        coller_marqueur(sol, ident, x + marge, y + marge)
    if obstacle:
        x, y, l, h = obstacle
        cv2.rectangle(sol, (int((x + marge) * PX), int((y + marge) * PX)),
                      (int((x + l + marge) * PX), int((y + h + marge) * PX)), (40, 60, 90), cv2.FILLED)
    if robot:
        x, y, cap = robot
        cv2.circle(sol, (int((x + marge) * PX), int((y + marge) * PX)), 12 * PX, (255, 255, 255), cv2.FILLED)
        coller_marqueur(sol, ca.ID_ROBOT, x + marge, y + marge, cap)
    # perspective : la camera n'est pas parfaitement a la verticale
    h, w = sol.shape[:2]
    depart = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    arrivee = np.float32([[120, 40], [1180, 70], [1230, 700], [60, 680]])
    return cv2.warpPerspective(sol, cv2.getPerspectiveTransform(depart, arrivee), (1280, 720),
                               borderValue=(200, 200, 200))


def ecart_angle(a, b):
    return abs(ca.normaliser(a - b))


def test_lire_zone_et_normaliser():
    assert ca.lire_zone("200x150") == (200.0, 150.0)
    assert ca.normaliser(190) == -170
    assert ca.normaliser(-180) == -180


def test_calibration_et_position_du_robot():
    carto = ca.Cartographe(LARGEUR, HAUTEUR)
    carto.traiter(scene(robot=(120, 50, 30)), t=10.0)
    carte = carto.carte(10.0)
    assert carte["calibre"] and carte["erreur"] is None
    r = carte["robot"]
    assert abs(r["x"] - 120) < 2 and abs(r["y"] - 50) < 2
    assert ecart_angle(r["cap"], 30) < 3


@pytest.mark.parametrize("cap", [0, 90, 180, -90, -135])
def test_cap_du_robot(cap):
    carto = ca.Cartographe(LARGEUR, HAUTEUR)
    carto.traiter(scene(robot=(100, 75, cap)), t=0.0)
    assert ecart_angle(carto.carte(0.0)["robot"]["cap"], cap) < 3


def test_coin_manquant_au_depart():
    carto = ca.Cartographe(LARGEUR, HAUTEUR)
    image = scene()
    image[:, 1000:] = 200                       # cache les marqueurs 1 et 2
    carto.traiter(image, t=0.0)
    carte = carto.carte(0.0)
    assert not carte["calibre"]
    assert "1" in carte["erreur"] and "2" in carte["erreur"]


def test_coin_cache_ensuite_garde_la_calibration():
    carto = ca.Cartographe(LARGEUR, HAUTEUR)
    carto.traiter(scene(), t=0.0)
    image = scene(robot=(60, 60, 0))
    image[:, 1000:] = 200
    carto.traiter(image, t=1.0)
    carte = carto.carte(1.0)
    assert carte["calibre"] and abs(carte["robot"]["x"] - 60) < 2


def test_sens_inverse_refuse():
    carto = ca.Cartographe(LARGEUR, HAUTEUR)
    carto.traiter(scene(), t=0.0)
    m = carto.marqueurs                          # 1 et 3 echanges au sol
    carto.marqueurs = {0: m[0], 1: m[3], 2: m[2], 3: m[1]}
    carto._calibrer()
    assert carto.H is None and "echanger" in carto.erreur


@pytest.fixture(autouse=True)
def reference_temporaire(monkeypatch, tmp_path):
    """La photo du sol vide va dans un dossier temporaire, pas dans le depot."""
    monkeypatch.setattr(ca, "FICHIER_REFERENCE", str(tmp_path / "sol.png"))


def test_obstacle_detecte_et_robot_ignore():
    carto = ca.Cartographe(LARGEUR, HAUTEUR)
    carto.traiter(scene(), t=0.0)
    ok, _ = carto.prendre_reference(0.0)
    assert ok
    carto.traiter(scene(robot=(40, 110, 0), obstacle=(120, 40, 40, 30)), t=1.0)
    carte = carto.carte(1.0)
    assert len(carte["obstacles"]) == 1          # le robot n'est pas un obstacle
    o = carte["obstacles"][0]
    assert abs(o["x"] - 120) < 4 and abs(o["y"] - 40) < 4
    assert abs(o["l"] - 40) < 6 and abs(o["h"] - 30) < 6
    lignes = carte["grille"]["lignes"]
    assert len(lignes) == 15 and len(lignes[0]) == 20
    assert lignes[5][13] == "#" and lignes[12][3] == "."


def test_reference_refusee_si_robot_visible():
    carto = ca.Cartographe(LARGEUR, HAUTEUR)
    carto.traiter(scene(robot=(100, 75, 0)), t=0.0)
    ok, raison = carto.prendre_reference(0.0)
    assert not ok and "robot" in raison


def test_parallaxe():
    # tete a 37 cm, camera a 250 cm, 100 cm du nadir : en vrai 85.2 cm
    x, y = ca.corriger_parallaxe(150, 50, (50, 50), 250, 37)
    assert abs(x - (50 + 100 * (1 - 37 / 250.0))) < 1e-6 and y == 50
    assert ca.corriger_parallaxe(150, 50, (50, 50), 0, 37) == (150, 50)   # desactivee


def test_carte_publiee_age(monkeypatch):
    carto = ca.Cartographe(LARGEUR, HAUTEUR)
    carto.traiter(scene(robot=(100, 75, 0)), t=10.0)
    monkeypatch.setitem(ca.partage, "carte", carto.carte(10.0))
    carte = ca.carte_publiee(12.5)
    assert carte["age"] == 2.5 and carte["robot"]["age"] == 2.5
    assert "t" not in carte and "t" not in carte["robot"]
