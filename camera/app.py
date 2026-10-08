"""
Surveillance Logitech C270 — salle serveurs
===========================================

Tourne sur le PC hote (Windows) : Docker Desktop n'a pas acces a l'USB.

- Detection de mouvement (difference de frames)
- Detection de visages (Haar)
- Reconnaissance (LBPH) a partir de camera/known_faces/<nom>/*.jpg
- Flux MJPEG annote, metriques Prometheus, snapshots d'evenements

Lancer :
  python camera/app.py
  ou  .\\camera\\start-camera.ps1
"""

from __future__ import annotations

import logging
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request
from prometheus_client import CONTENT_TYPE_LATEST, Counter, Gauge, generate_latest
from waitress import serve

HERE = Path(__file__).resolve().parent
KNOWN_DIR = Path(os.environ.get("KNOWN_FACES_DIR", HERE / "known_faces"))
EVENTS_DIR = Path(os.environ.get("EVENTS_DIR", HERE / "events"))

CAMERA_INDEX = os.environ.get("CAMERA_INDEX", "auto")
# Nom a chercher (C270, Logitech, Logi...). Ignore la webcam integree HP.
CAMERA_NAME = os.environ.get("CAMERA_NAME", "C270").strip().lower()
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
PORT = int(os.environ.get("CAMERA_PORT", "8001"))

MOTION_THRESHOLD = int(os.environ.get("MOTION_THRESHOLD", "25"))
MOTION_MIN_AREA = int(os.environ.get("MOTION_MIN_AREA", "1200"))
MOTION_HOLD_S = float(os.environ.get("MOTION_HOLD_S", "20"))
FACE_HOLD_S = float(os.environ.get("FACE_HOLD_S", "25"))
# LBPH : plus le score est BAS, plus c'est un bon match. ~50-70 = connu.
LBPH_MAX_DISTANCE = float(os.environ.get("LBPH_MAX_DISTANCE", "70"))
SNAPSHOT_COOLDOWN_S = float(os.environ.get("SNAPSHOT_COOLDOWN_S", "20"))
# Duree de l'animation d'analyse avant "VISAGE NON DETECTE"
FACE_ANALYZE_S = float(os.environ.get("FACE_ANALYZE_S", "7"))

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("camera")

EVENTS_DIR.mkdir(parents=True, exist_ok=True)

g_up = Gauge("yanshee_camera_up", "Webcam joignable (1) ou non (0)")
g_motion = Gauge("yanshee_camera_motion", "Mouvement detecte (1) ou salle calme (0)")
g_faces = Gauge("yanshee_camera_faces", "Nombre de visages dans le cadre")
g_unknown = Gauge("yanshee_camera_unknown_faces", "Visages inconnus dans le cadre")
g_known = Gauge("yanshee_camera_known_faces", "Visages reconnus dans le cadre")
c_motion = Counter("yanshee_camera_motion_events", "Evenements de mouvement")
c_unknown = Counter("yanshee_camera_unknown_events", "Detections de visages inconnus")

lock = threading.Lock()
state = {
    "up": False,
    "index": None,
    "motion": False,
    "faces": 0,
    "known": [],
    "unknown": 0,
    "last_event": None,
    "known_people": [],
    "error": None,
    "fps": 0.0,
    "devices": [],
    "name": None,
}
latest_jpeg = None
latest_bgr = None
last_face_boxes = []
last_motion_t = 0.0
last_unknown_t = 0.0
last_snapshot_t = 0.0
motion_armed = True

recognizer = None
label_names = {}  # id -> nom
haar = None


def dshow_names():
    """Noms DirectShow dans le meme ordre que les index OpenCV CAP_DSHOW."""
    try:
        from pygrabber.dshow_graph import FilterGraph
        return list(FilterGraph().get_input_devices())
    except Exception as e:  # noqa: BLE001
        log.warning("Impossible de lister les noms de webcam: %s", e)
        return []


def pick_camera_index(names):
    if CAMERA_INDEX != "auto":
        return [int(CAMERA_INDEX)]
    if names:
        preferred = []
        others = []
        for i, name in enumerate(names):
            n = name.lower()
            log.info("Webcam %s : %s", i, name)
            if CAMERA_NAME and CAMERA_NAME in n:
                preferred.append(i)
            elif any(k in n for k in ("c270", "logitech", "logi")):
                preferred.append(i)
            else:
                others.append(i)
        if preferred:
            return preferred
        return others or list(range(0, 6))
    return list(range(0, 6))


def open_camera():
    """Ouvre la C270. Sur Windows, DirectShow est plus fiable que MSMF."""
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    names = dshow_names() if os.name == "nt" else []
    with lock:
        state["devices"] = names
    indices = pick_camera_index(names)
    for attempt in range(8):
        for idx in indices:
            cap = cv2.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, 20)
            ok, frame = cap.read()
            if ok and frame is not None:
                label = names[idx] if idx < len(names) else "?"
                log.info("Webcam ouverte (index %s, %s, %sx%s)", idx, label, frame.shape[1], frame.shape[0])
                return cap, idx
            cap.release()
        time.sleep(0.4)
    return None, None


def load_haar():
    bundled = HERE / "models" / "haarcascade_frontalface_default.xml"
    candidates = [bundled]
    data_dir = getattr(cv2, "data", None)
    if data_dir is not None:
        candidates.append(Path(data_dir.haarcascades) / "haarcascade_frontalface_default.xml")
    for path in candidates:
        if path.exists():
            cascade = cv2.CascadeClassifier(str(path))
            if not cascade.empty():
                log.info("Cascade visages : %s", path)
                return cascade
    raise RuntimeError("Cascade Haar introuvable (place models/haarcascade_frontalface_default.xml)")


def train_faces():
    """Entraine LBPH a partir de known_faces/<nom>/*.jpg. Retourne (recognizer, {id: nom})."""
    global recognizer, label_names
    names = {}
    images, labels = [], []
    next_id = 0
    if not KNOWN_DIR.exists():
        KNOWN_DIR.mkdir(parents=True, exist_ok=True)

    for person_dir in sorted(p for p in KNOWN_DIR.iterdir() if p.is_dir()):
        files = [
            f for f in person_dir.iterdir()
            if f.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp")
        ]
        if not files:
            continue
        pid = next_id
        next_id += 1
        names[pid] = person_dir.name
        for f in files:
            img = cv2.imdecode(np.fromfile(f, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                log.warning("Photo illisible: %s", f.name)
                continue
            h0, w0 = img.shape[:2]
            if max(h0, w0) > 800:
                scale = 800 / max(h0, w0)
                img = cv2.resize(img, (int(w0 * scale), int(h0 * scale)))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            try:
                detected = haar.detectMultiScale(gray, 1.1, 5, minSize=(60, 60))
            except cv2.error:
                detected = []
            if len(detected):
                x, y, w, h = max(detected, key=lambda r: r[2] * r[3])
                roi = cv2.resize(gray[y:y + h, x:x + w], (160, 160))
            else:
                roi = cv2.resize(gray, (160, 160))
            images.append(roi)
            labels.append(pid)

    if not images:
        recognizer = None
        label_names = {}
        log.info("Aucun visage connu : tout visage detecte sera 'inconnu'.")
        return

    rec = cv2.face.LBPHFaceRecognizer_create()
    rec.train(images, np.array(labels, dtype=np.int32))
    recognizer = rec
    label_names = names
    log.info(
        "Reconnaissance prete : %d photo(s), personnes = %s",
        len(images), ", ".join(names.values()),
    )


def identify(gray_face):
    if recognizer is None:
        return "inconnu", 999.0
    roi = cv2.resize(gray_face, (160, 160))
    label, dist = recognizer.predict(roi)
    if dist <= LBPH_MAX_DISTANCE:
        return label_names.get(label, "inconnu"), float(dist)
    return "inconnu", float(dist)


def save_snapshot(frame, kind):
    global last_snapshot_t
    now = time.time()
    if now - last_snapshot_t < SNAPSHOT_COOLDOWN_S:
        return None
    last_snapshot_t = now
    name = datetime.now().strftime(f"{kind}_%Y%m%d_%H%M%S.jpg")
    path = EVENTS_DIR / name
    cv2.imwrite(str(path), frame)
    log.info("Snapshot %s -> %s", kind, path.name)
    return str(path.name)


def _text(img, s, org, scale=0.5, color=(255, 255, 255), thick=1):
    """Texte blanc type CCTV, avec contour noir pour rester lisible."""
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def _corners(img, x, y, w, h, color, length=None, thickness=2):
    """Coins de visee (pas un rectangle plein)."""
    L = length if length is not None else max(10, min(w, h) // 5)
    x2, y2 = x + w, y + h
    cv2.line(img, (x, y), (x + L, y), color, thickness)
    cv2.line(img, (x, y), (x, y + L), color, thickness)
    cv2.line(img, (x2, y), (x2 - L, y), color, thickness)
    cv2.line(img, (x2, y), (x2, y + L), color, thickness)
    cv2.line(img, (x, y2), (x + L, y2), color, thickness)
    cv2.line(img, (x, y2), (x, y2 - L), color, thickness)
    cv2.line(img, (x2, y2), (x2 - L, y2), color, thickness)
    cv2.line(img, (x2, y2), (x2, y2 - L), color, thickness)


def _scan_line(img, x, y, w, h, color):
    """Ligne de scan qui descend puis remonte en boucle."""
    cycle = (time.time() * 0.7) % 2.0
    frac = cycle if cycle <= 1.0 else 2.0 - cycle
    sy = y + 2 + int((h - 4) * frac)
    overlay = img.copy()
    cv2.line(overlay, (x + 2, sy), (x + w - 2, sy), color, 2)
    # halo leger au-dessus de la ligne
    glow = max(8, h // 10)
    y1, y2 = max(y, sy - glow), min(y + h, sy + 2)
    cv2.rectangle(overlay, (x + 2, y1), (x + w - 2, y2), color, -1)
    cv2.addWeighted(overlay, 0.35, img, 0.65, 0, img)


def annotate(frame, motion, faces_info, cam_name="CAM-01", analyze_age=0.0):
    vis = frame.copy()
    H, W = vis.shape[:2]
    t = time.time()
    cyan = (220, 220, 40)
    green = (80, 220, 80)
    red = (50, 50, 255)
    white = (255, 255, 255)
    failed = analyze_age >= FACE_ANALYZE_S

    # --- Bandeau haut : REC + statut ---
    if int(t * 2) % 2 == 0:
        cv2.circle(vis, (16, 18), 6, (0, 0, 220), -1)
    else:
        cv2.circle(vis, (16, 18), 6, (0, 0, 90), -1)
    _text(vis, "REC", (28, 24), 0.5, white, 1)
    _text(vis, cam_name, (78, 24), 0.45, white, 1)
    if motion:
        _text(vis, "MOUVEMENT", (W - 168, 24), 0.5, (0, 160, 255), 1)

    # --- Visages : visee + scan ---
    for (x, y, w, h, name, dist) in faces_info:
        known = name != "inconnu"
        if known:
            color = green
        elif failed:
            color = red
        else:
            color = cyan
        _corners(vis, x, y, w, h, color, thickness=2)
        _scan_line(vis, x, y, w, h, color)

        # points de tracking sur le contour
        n = 8
        for i in range(n):
            a = (t * 2.2 + i * (6.283 / n)) % 6.283
            px = int(x + w / 2 + (w / 2 - 4) * np.cos(a) * 0.92)
            py = int(y + h / 2 + (h / 2 - 4) * np.sin(a) * 0.92)
            cv2.circle(vis, (px, py), 2, color, -1)

        if known:
            label = name.upper()
            sub = "IDENTIFIE"
            fill_ratio = 1.0
        elif failed:
            label = "VISAGE NON DETECTE"
            sub = "ECHEC IDENTIFICATION"
            fill_ratio = 1.0
        else:
            label = "SCAN VISAGE"
            dots = "." * (1 + int(t * 3) % 3)
            remain = max(0.0, FACE_ANALYZE_S - analyze_age)
            sub = "ANALYSE" + dots + "  " + str(int(remain) + 1) + "s"
            fill_ratio = min(1.0, analyze_age / FACE_ANALYZE_S)
        _text(vis, label, (x, max(16, y - 10)), 0.45, color, 1)
        _text(vis, sub, (x, min(H - 8, y + h + 16)), 0.4, color, 1)

        bar_w = w
        fill = int(bar_w * fill_ratio)
        by = min(H - 6, y + h + 22)
        cv2.rectangle(vis, (x, by), (x + bar_w, by + 4), (40, 40, 40), -1)
        cv2.rectangle(vis, (x, by), (x + max(2, fill), by + 4), color, -1)

    # --- Bandeau bas gauche : date + heure en blanc ---
    stamp = datetime.now().strftime("%d/%m/%Y  %H:%M:%S")
    _text(vis, stamp, (12, H - 28), 0.55, white, 1)
    _text(vis, "SALLE SERVEURS", (12, H - 10), 0.4, (200, 200, 200), 1)
    return vis


def worker():
    global latest_jpeg, last_motion_t, last_unknown_t, motion_armed
    cap, idx = open_camera()
    with lock:
        state["up"] = cap is not None
        state["index"] = idx
        devices = state.get("devices") or []
        state["name"] = devices[idx] if idx is not None and idx < len(devices) else None
        state["error"] = None if cap else "webcam introuvable"
        state["known_people"] = list(label_names.values())
    g_up.set(1 if cap else 0)
    if cap is None:
        log.error("Aucune webcam. Branche la C270 et relance.")
        return

    prev_gray = None
    frames = 0
    t_fps = time.time()
    analyze_since = None
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            log.warning("Perte de la webcam, reconnexion...")
            cap.release()
            g_up.set(0)
            with lock:
                state["up"] = False
                state["error"] = "webcam deconnectee"
            time.sleep(2)
            cap, idx = open_camera()
            if cap is None:
                time.sleep(3)
                continue
            prev_gray = None
            g_up.set(1)
            with lock:
                state["up"] = True
                state["index"] = idx
                state["error"] = None
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (21, 21), 0)

        motion = False
        if prev_gray is not None:
            delta = cv2.absdiff(prev_gray, gray_blur)
            _, thresh = cv2.threshold(delta, MOTION_THRESHOLD, 255, cv2.THRESH_BINARY)
            thresh = cv2.dilate(thresh, None, iterations=2)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            motion = any(cv2.contourArea(c) >= MOTION_MIN_AREA for c in contours)
        prev_gray = gray_blur

        now = time.time()
        if motion:
            if motion_armed:
                c_motion.inc()
                motion_armed = False
                save_snapshot(frame, "motion")
                with lock:
                    state["last_event"] = "mouvement " + datetime.now().strftime("%H:%M:%S")
            last_motion_t = now
        elif now - last_motion_t > MOTION_HOLD_S:
            motion_armed = True

        motion_held = (now - last_motion_t) < MOTION_HOLD_S

        faces_raw = haar.detectMultiScale(gray, scaleFactor=1.15, minNeighbors=5, minSize=(50, 50))
        faces_info = []
        known_now, unknown_now = [], 0
        for (x, y, w, h) in faces_raw:
            name, dist = identify(gray[y:y + h, x:x + w])
            faces_info.append((x, y, w, h, name, dist))
            if name == "inconnu":
                unknown_now += 1
            else:
                known_now.append(name)

        if unknown_now:
            if analyze_since is None:
                analyze_since = now
            if now - last_unknown_t > FACE_HOLD_S:
                c_unknown.inc()
                save_snapshot(frame, "unknown")
                with lock:
                    state["last_event"] = "visage inconnu " + datetime.now().strftime("%H:%M:%S")
            last_unknown_t = now
        elif analyze_since is not None and now - last_unknown_t > 1.0:
            analyze_since = None

        analyze_age = (now - analyze_since) if analyze_since is not None else 0.0

        unknown_held = 1 if (now - last_unknown_t) < FACE_HOLD_S else 0

        vis = annotate(
            frame, motion_held, faces_info,
            cam_name="CAM-01", analyze_age=analyze_age,
        )
        ok_jpg, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok_jpg:
            with lock:
                latest_jpeg = buf.tobytes()
                latest_bgr = frame.copy()
                last_face_boxes = [(int(x), int(y), int(w), int(h)) for (x, y, w, h, _n, _d) in faces_info]

        frames += 1
        if now - t_fps >= 2:
            fps = frames / (now - t_fps)
            frames = 0
            t_fps = now
        else:
            fps = state["fps"]

        g_motion.set(1 if motion_held else 0)
        g_faces.set(len(faces_info))
        g_unknown.set(unknown_held if unknown_held else unknown_now)
        g_known.set(len(known_now))
        with lock:
            state.update({
                "motion": motion_held,
                "faces": len(faces_info),
                "known": known_now,
                "unknown": unknown_now if unknown_now else unknown_held,
                "fps": round(fps, 1),
                "known_people": list(label_names.values()),
            })


app = Flask(__name__)


@app.after_request
def cors(resp):
    resp.headers["Access-Control-Allow-Origin"] = "*"
    return resp


@app.route("/healthz")
def healthz():
    with lock:
        return jsonify(status="ok" if state["up"] else "down", **{k: state[k] for k in state})


@app.route("/status")
def status():
    with lock:
        return jsonify(dict(state))


@app.route("/metrics")
def metrics():
    return Response(generate_latest(), mimetype=CONTENT_TYPE_LATEST)


@app.route("/video")
def video():
    def gen():
        while True:
            with lock:
                frame = latest_jpeg
            if frame is None:
                time.sleep(0.1)
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.07)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/snapshot")
def snapshot():
    with lock:
        frame = latest_jpeg
    if frame is None:
        return jsonify(erreur="pas d'image"), 503
    return Response(frame, mimetype="image/jpeg")


@app.route("/reload-faces", methods=["GET", "POST"])
def reload_faces():
    train_faces()
    with lock:
        state["known_people"] = list(label_names.values())
    return jsonify(ok=True, personnes=list(label_names.values()))


def _safe_name(raw):
    raw = (raw or "").strip()
    cleaned = "".join(c if (c.isalnum() or c in "-_ ") else "" for c in raw).strip()
    return cleaned[:40]


@app.route("/enroll", methods=["POST"])
def enroll():
    """Capture le visage actuel et l'associe a un prenom."""
    payload = request.get_json(silent=True) or {}
    nom = _safe_name(payload.get("name") or request.args.get("name") or request.form.get("name"))
    if not nom:
        return jsonify(ok=False, erreur="Indique un prenom."), 400

    with lock:
        frame = None if latest_bgr is None else latest_bgr.copy()
        boxes = list(last_face_boxes)

    if frame is None:
        return jsonify(ok=False, erreur="Pas d'image camera."), 503
    if not boxes:
        return jsonify(ok=False, erreur="Aucun visage devant la camera. Place-toi face a l'objectif."), 400

    x, y, w, h = max(boxes, key=lambda r: r[2] * r[3])
    pad = int(0.25 * max(w, h))
    H, W = frame.shape[:2]
    x1, y1 = max(0, x - pad), max(0, y - pad)
    x2, y2 = min(W, x + w + pad), min(H, y + h + pad)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return jsonify(ok=False, erreur="Recadrage du visage impossible."), 400

    dest = KNOWN_DIR / nom
    dest.mkdir(parents=True, exist_ok=True)
    fname = datetime.now().strftime("enroll_%Y%m%d_%H%M%S.jpg")
    path = dest / fname
    cv2.imwrite(str(path), crop)
    train_faces()
    with lock:
        state["known_people"] = list(label_names.values())
    n = len(list(dest.glob("*.jpg"))) + len(list(dest.glob("*.png")))
    log.info("Visage enregistre : %s (%s) — %d photo(s)", nom, fname, n)
    return jsonify(ok=True, nom=nom, fichier=fname, photos=n, personnes=list(label_names.values()))


@app.route("/")
def index():
    return (
        "<html><body style='background:#0b0f14;color:#e6edf3;font-family:sans-serif'>"
        "<h2>Surveillance C270</h2>"
        "<p><a href='/video' style='color:#5cc8ff'>/video</a> · "
        "<a href='/status' style='color:#5cc8ff'>/status</a> · "
        "<a href='/metrics' style='color:#5cc8ff'>/metrics</a></p>"
        "<img src='/video' style='max-width:100%;border-radius:12px'/>"
        "</body></html>"
    )


def main():
    global haar
    haar = load_haar()
    train_faces()
    threading.Thread(target=worker, daemon=True).start()
    log.info("Camera HTTP sur http://127.0.0.1:%s", PORT)
    serve(app, host="0.0.0.0", port=PORT, threads=6)


if __name__ == "__main__":
    main()
