# -*- coding: utf-8 -*-
"""Tests de verifier_api.py contre le vrai capteurs_serveur.py (sans robot) : python -m pytest tests -v"""

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import capteurs_serveur as cs  # noqa: E402
import verifier_api as va  # noqa: E402

BONNES = {"c": 87, "gaz": 64, "vapeur": 26, "lum": 50, "temp": 24, "hum": 55}


@pytest.fixture
def serveur(monkeypatch):
    s = cs.ServeurMultiTache(("127.0.0.1", 0), cs.Handler)
    threading.Thread(target=s.serve_forever, daemon=True).start()
    monkeypatch.setattr(cs, "lire_batterie", lambda: {"data": {"percent": 80, "charging": 0}})
    monkeypatch.setattr(cs, "CAMERA_HTTP", "http://127.0.0.1:9")     # pas de webcam
    monkeypatch.setattr(va, "ECHANTILLONS", 2)
    yield "http://127.0.0.1:%d" % s.server_address[1]
    s.shutdown()
    s.server_close()


def mesure(brut):
    with cs.verrou:
        cs.etat.update(brut=brut, alertes=[], t=time.time() + 60)    # reste fraiche pendant le test


def test_serveur_en_bon_etat(serveur, monkeypatch, capsys):
    mesure(BONNES)
    monkeypatch.setattr(sys, "argv", ["verifier_api.py", serveur])
    assert va.main() == 0
    sortie = capsys.readouterr().out
    assert "0 erreur(s)" in sortie and "ultrason : 87 cm" in sortie


def test_mesure_perimee_et_valeurs_fausses(serveur, monkeypatch, capsys):
    with cs.verrou:
        cs.etat.update(brut=dict(BONNES, c="?"), alertes=[], t=time.time() - 10)
    monkeypatch.setattr(sys, "argv", ["verifier_api.py", serveur])
    assert va.main() == 1
    sortie = capsys.readouterr().out
    assert "perime = true" in sortie and "c doit etre un entier" in sortie


def test_serveur_injoignable(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["verifier_api.py", "http://127.0.0.1:9"])
    assert va.main() == 1
    assert "Serveur injoignable" in capsys.readouterr().out


def test_controles_unitaires():
    r = va.Rapport()
    va.verifier_capteurs(dict(BONNES, perime=False, age=0.1, alertes=[{"type": "gaz"}],
                              gaz_etat="pret", gaz_ref=None, seuils={}), r)
    assert r.erreurs == 3            # alerte incomplete, gaz_ref absent, seuils incomplet
    r = va.Rapport()
    va.verifier_batterie({"data": {"percent": 150}}, r)
    assert r.erreurs == 1
    r = va.Rapport()
    va.verifier_carte(404, {"erreur": "x"}, r)
    assert r.erreurs == 1            # ancien serveur sans /carte
