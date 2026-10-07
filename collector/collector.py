"""
Collecteur Yanshee
==================

- Sonde periodiquement le robot (/capteurs, /batterie, /marche)
- Enregistre chaque mesure dans TimescaleDB (table "readings")
- Expose les metriques Prometheus sur /metrics
- Sert l'interface live (interface.html) et proxy les endpoints du robot
  pour que la page fonctionne en meme-origine, sans CORS.
"""

import json
import logging
import os
import threading
import time

import psycopg2
import requests
from flask import Flask, Response, jsonify, request, send_from_directory
from prometheus_client import CONTENT_TYPE_LATEST, Gauge, generate_latest
from waitress import serve

# --- Configuration -----------------------------------------------------------
ROBOT_URL = os.environ.get("ROBOT_URL", "http://10.124.7.2:8080").rstrip("/")
POLL_INTERVAL = float(os.environ.get("POLL_INTERVAL", "2"))
REQUEST_TIMEOUT = float(os.environ.get("REQUEST_TIMEOUT", "1.5"))
PORTEE_CM = float(os.environ.get("PORTEE_CM", "400"))
DB_DSN = os.environ.get(
    "DB_DSN", "postgresql://yanshee:yanshee@timescaledb:5432/yanshee"
)
STATIC_DIR = os.environ.get("STATIC_DIR", "/app/static")
PORT = int(os.environ.get("PORT", "8000"))

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("collector")

KNOWN_STATES = [
    "demarrage", "marche", "obstacle", "arret",
    "refus", "erreur", "termine", "jamais lance",
]

# --- Metriques Prometheus ----------------------------------------------------
g_up = Gauge("yanshee_up", "Robot joignable (1) ou hors ligne (0)")
g_perime = Gauge("yanshee_mesure_perime", "Mesure Arduino perimee (1) ou fraiche (0)")
g_dist = Gauge("yanshee_distance_cm", "Distance ultrason (cm)", ["capteur"])
g_gaz = Gauge("yanshee_gaz", "Capteur gaz MQ-2 (0-1023)")
g_vapeur = Gauge("yanshee_vapeur", "Capteur vapeur/eau steam (0-1023)")
g_lum = Gauge("yanshee_lumiere", "Capteur lumiere LDR (0-1023)")
g_batt = Gauge("yanshee_batterie_pourcent", "Niveau batterie (%)")
g_charge = Gauge("yanshee_batterie_charge", "Batterie en charge (1) ou non (0)")
g_lat = Gauge("yanshee_latence_ms", "Latence de la requete /capteurs (ms)")
g_mv = Gauge("yanshee_mouvement", "Etat du mouvement (1 = etat actif)", ["etat"])

NAN = float("nan")

# Dernieres reponses du robot, pour servir l'UI si le robot flanche brievement
last_cache = {"/capteurs": None, "/batterie": None, "/marche": None}


def num(v):
    """Convertit en float si possible, sinon None."""
    try:
        if v is None:
            return None
        return float(v)
    except (TypeError, ValueError):
        return None


def dist_value(raw):
    """Valeur brute ultrason -> cm. -1 (pas d'echo) = libre = PORTEE_CM. None sinon."""
    n = num(raw)
    if n is None:
        return None
    if n < 0:
        return PORTEE_CM
    return n


# --- Base de donnees ---------------------------------------------------------
def get_conn():
    conn = psycopg2.connect(DB_DSN)
    conn.autocommit = True
    return conn


INSERT_SQL = """
    INSERT INTO readings
        (time, robot_up, perime, dist_g, dist_c, dist_d,
         gaz, vapeur, lum, battery_pct, charging, mv_etat, latency_ms)
    VALUES
        (now(), %(robot_up)s, %(perime)s, %(dist_g)s, %(dist_c)s, %(dist_d)s,
         %(gaz)s, %(vapeur)s, %(lum)s, %(battery_pct)s, %(charging)s,
         %(mv_etat)s, %(latency_ms)s)
"""


def insert_row(conn, row):
    with conn.cursor() as cur:
        cur.execute(INSERT_SQL, row)


# --- Mise a jour des metriques ----------------------------------------------
def update_metrics(up, capteurs, batt, charging, latency, mv_etat):
    g_up.set(1 if up else 0)

    if up:
        g_perime.set(1 if capteurs.get("perime") else 0)
        for k in ("g", "c", "d"):
            dv = dist_value(capteurs.get(k))
            g_dist.labels(capteur=k).set(dv if dv is not None else NAN)
        for key, gauge in (("gaz", g_gaz), ("vapeur", g_vapeur), ("lum", g_lum)):
            v = num(capteurs.get(key))
            gauge.set(v if v is not None else NAN)
        g_lat.set(latency if latency is not None else NAN)
    else:
        # Robot hors ligne : on neutralise les mesures pour eviter les
        # fausses alertes sur des valeurs perimees (RobotDown couvre ce cas).
        g_perime.set(NAN)
        for k in ("g", "c", "d"):
            g_dist.labels(capteur=k).set(NAN)
        for gauge in (g_gaz, g_vapeur, g_lum, g_lat):
            gauge.set(NAN)

    if batt is not None:
        g_batt.set(batt)
    g_charge.set(1 if charging else 0 if charging is not None else NAN)

    for st in KNOWN_STATES:
        g_mv.labels(etat=st).set(1 if st == mv_etat else 0)


# --- Boucle de sondage -------------------------------------------------------
def poll_once():
    """Interroge le robot une fois et renvoie un tuple exploitable."""
    up = False
    capteurs = {}
    latency = None
    try:
        t0 = time.time()
        r = requests.get(ROBOT_URL + "/capteurs", timeout=REQUEST_TIMEOUT)
        latency = (time.time() - t0) * 1000.0
        capteurs = r.json()
        last_cache["/capteurs"] = capteurs
        up = True
    except Exception as e:  # noqa: BLE001
        log.warning("Robot injoignable (/capteurs): %s", e)

    batt_pct = None
    charging = None
    try:
        rb = requests.get(ROBOT_URL + "/batterie", timeout=REQUEST_TIMEOUT)
        bj = rb.json()
        last_cache["/batterie"] = bj
        data = bj.get("data", bj) if isinstance(bj, dict) else {}
        batt_pct = num(data.get("percent", data.get("capacity")))
        charging = data.get("charging")
    except Exception:  # noqa: BLE001
        pass

    mv_etat = None
    try:
        rm = requests.get(ROBOT_URL + "/marche", timeout=REQUEST_TIMEOUT)
        mj = rm.json()
        last_cache["/marche"] = mj
        if isinstance(mj, dict):
            mv_etat = mj.get("etat")
    except Exception:  # noqa: BLE001
        pass

    return up, capteurs, batt_pct, charging, latency, mv_etat


def poller():
    conn = None
    while True:
        start = time.time()
        up, capteurs, batt_pct, charging, latency, mv_etat = poll_once()
        update_metrics(up, capteurs, batt_pct, charging, latency, mv_etat)

        row = {
            "robot_up": up,
            "perime": bool(capteurs.get("perime")) if up else None,
            "dist_g": dist_value(capteurs.get("g")) if up else None,
            "dist_c": dist_value(capteurs.get("c")) if up else None,
            "dist_d": dist_value(capteurs.get("d")) if up else None,
            "gaz": num(capteurs.get("gaz")) if up else None,
            "vapeur": num(capteurs.get("vapeur")) if up else None,
            "lum": num(capteurs.get("lum")) if up else None,
            "battery_pct": batt_pct,
            "charging": bool(charging) if charging is not None else None,
            "mv_etat": mv_etat,
            "latency_ms": latency,
        }
        try:
            if conn is None or conn.closed:
                conn = get_conn()
            insert_row(conn, row)
        except Exception as e:  # noqa: BLE001
            log.error("Ecriture DB impossible: %s", e)
            conn = None  # force une reconnexion au prochain tour

        elapsed = time.time() - start
        time.sleep(max(0.0, POLL_INTERVAL - elapsed))


# --- API HTTP ----------------------------------------------------------------
app = Flask(__name__)


def _proxy(path):
    """Proxy live vers le robot ; repli sur le dernier cache si injoignable."""
    try:
        r = requests.get(ROBOT_URL + path, timeout=REQUEST_TIMEOUT)
        last_cache[path] = r.json()
        return Response(
            r.content, status=r.status_code,
            content_type=r.headers.get("Content-Type", "application/json"),
        )
    except Exception:  # noqa: BLE001
        cached = last_cache.get(path)
        if cached is not None:
            return Response(json.dumps(cached), content_type="application/json")
        return Response(
            json.dumps({"erreur": "robot injoignable"}),
            status=502, content_type="application/json",
        )


@app.route("/")
def index():
    return send_from_directory(STATIC_DIR, "interface.html")


@app.route("/capteurs")
def capteurs():
    return _proxy("/capteurs")


@app.route("/batterie")
def batterie():
    return _proxy("/batterie")


@app.route("/marche")
def marche():
    return _proxy("/marche")


@app.route("/metrics")
def metrics():
    return Response(generate_latest(), mimetype=CONTENT_TYPE_LATEST)


@app.route("/healthz")
def healthz():
    return jsonify(status="ok", robot_url=ROBOT_URL)


@app.route("/api/history")
def history():
    """Historique brut depuis la DB : /api/history?minutes=60"""
    minutes = int(request.args.get("minutes", "60"))
    try:
        conn = get_conn()
        with conn.cursor() as cur:
            cur.execute(
                """
                SELECT time, robot_up, dist_c, gaz, vapeur, lum,
                       battery_pct, mv_etat
                FROM readings
                WHERE time > now() - (%s || ' minutes')::interval
                ORDER BY time
                """,
                (minutes,),
            )
            cols = [d[0] for d in cur.description]
            rows = [dict(zip(cols, r)) for r in cur.fetchall()]
        conn.close()
        return app.response_class(
            json.dumps(rows, default=str), mimetype="application/json"
        )
    except Exception as e:  # noqa: BLE001
        return jsonify(erreur=str(e)), 500


def main():
    # On neutralise les metriques au demarrage tant que le robot n'a pas repondu
    update_metrics(False, {}, None, None, None, None)
    threading.Thread(target=poller, daemon=True).start()
    log.info("Collecteur demarre - robot=%s, intervalle=%ss", ROBOT_URL, POLL_INTERVAL)
    serve(app, host="0.0.0.0", port=PORT, threads=8)


if __name__ == "__main__":
    main()
