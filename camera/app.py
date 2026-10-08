"""
Surveillance Logitech C270 — salle serveurs
===========================================

Tourne sur le PC hote (Windows) : Docker Desktop n'a pas acces a l'USB.

- Detection de mouvement (difference de frames)
- Detection de visages : YuNet (DNN OpenCV), repli Haar si modele absent
- Reconnaissance : SFace (embeddings + similarite cosinus), repli LBPH,
  a partir de camera/known_faces/<nom>/*.jpg
- Suivi des visages + vote sur plusieurs images (pas de decision sur 1 frame)
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
import urllib.request
from collections import Counter as Tally, deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, abort, jsonify, request, send_from_directory
from prometheus_client import CONTENT_TYPE_LATEST, Counter, Gauge, generate_latest
from waitress import serve

HERE = Path(__file__).resolve().parent
KNOWN_DIR = Path(os.environ.get("KNOWN_FACES_DIR", HERE / "known_faces"))
EVENTS_DIR = Path(os.environ.get("EVENTS_DIR", HERE / "events"))

CAMERA_INDEX = os.environ.get("CAMERA_INDEX", "auto")
# Nom a chercher (C270, Logitech, Logi...). Ignore la webcam integree HP.
CAMERA_NAME = os.environ.get("CAMERA_NAME", "C270").strip().lower()
# Identifiant fabricant USB de la webcam voulue (046D = Logitech).
CAMERA_USB_VID = os.environ.get("CAMERA_USB_VID", "046D").strip()
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
PORT = int(os.environ.get("CAMERA_PORT", "8001"))

MOTION_THRESHOLD = int(os.environ.get("MOTION_THRESHOLD", "25"))
MOTION_MIN_AREA = int(os.environ.get("MOTION_MIN_AREA", "1200"))
MOTION_HOLD_S = float(os.environ.get("MOTION_HOLD_S", "20"))
FACE_HOLD_S = float(os.environ.get("FACE_HOLD_S", "25"))
# LBPH (repli) : plus le score est BAS, plus c'est un bon match. ~50-70 = connu.
LBPH_MAX_DISTANCE = float(os.environ.get("LBPH_MAX_DISTANCE", "70"))
# SFace : similarite cosinus, plus c'est HAUT mieux c'est (seuil officiel 0.363).
SFACE_MIN_SCORE = float(os.environ.get("SFACE_MIN_SCORE", "0.40"))
# Ecart minimal entre la 1re et la 2e personne pour eviter les confusions.
SFACE_MARGIN = float(os.environ.get("SFACE_MARGIN", "0.05"))
YUNET_SCORE = float(os.environ.get("YUNET_SCORE", "0.75"))
FACE_MIN_SIZE = int(os.environ.get("FACE_MIN_SIZE", "40"))
# Vote : nb d'images pour confirmer une identite / un inconnu.
VOTE_WINDOW = int(os.environ.get("VOTE_WINDOW", "10"))
VOTE_KNOWN = int(os.environ.get("VOTE_KNOWN", "3"))
VOTE_UNKNOWN = int(os.environ.get("VOTE_UNKNOWN", "6"))
# Visage deja confirme : re-verification de l'identite toutes les REID_S secondes.
REID_S = float(os.environ.get("REID_S", "0.5"))
# Detection sur une image reduite a ce cote max (px) : bien plus rapide.
DETECT_MAX_SIDE = int(os.environ.get("DETECT_MAX_SIDE", "320"))
SNAPSHOT_COOLDOWN_S = float(os.environ.get("SNAPSHOT_COOLDOWN_S", "20"))
# Duree de l'animation d'analyse avant "VISAGE INCONNU"
FACE_ANALYZE_S = float(os.environ.get("FACE_ANALYZE_S", "7"))
# Mode enregistrement auto : nb de photos, delai entre deux, duree max.
ENROLL_COUNT = int(os.environ.get("ENROLL_COUNT", "8"))
ENROLL_INTERVAL_S = float(os.environ.get("ENROLL_INTERVAL_S", "0.6"))
ENROLL_TIMEOUT_S = float(os.environ.get("ENROLL_TIMEOUT_S", "60"))
ENROLL_MIN_FACE = int(os.environ.get("ENROLL_MIN_FACE", "90"))
# Nettete minimale (variance du Laplacien sur le visage ramene a 112x112).
ENROLL_MIN_SHARPNESS = float(os.environ.get("ENROLL_MIN_SHARPNESS", "70"))
# Au-dela de cette similarite avec une photo deja prise, la pose est jugee identique.
ENROLL_MAX_SIMILARITY = float(os.environ.get("ENROLL_MAX_SIMILARITY", "0.93"))

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
c_access = Counter("yanshee_camera_access_decisions", "Decisions d'acces par visage", ["decision"])

ACCESS_DIR = EVENTS_DIR / "access"
ACCESS_DIR.mkdir(parents=True, exist_ok=True)
ACCESS_BANNER_S = float(os.environ.get("ACCESS_BANNER_S", "4"))
# Journal des decisions d'acces (le plus recent en dernier), protege par `lock`.
access_log = deque(maxlen=int(os.environ.get("ACCESS_LOG_SIZE", "50")))
next_access_id = 1

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
    "engine": None,
    "enroll": None,
    "last_access": None,
    "fallback": False,
}
# Session du mode enregistrement auto (None si inactif), protegee par `lock`.
enroll_session = None
latest_jpeg = None
latest_bgr = None
last_face_boxes = []
last_motion_t = 0.0
last_unknown_t = 0.0
last_snapshot_t = 0.0
motion_armed = True

MODELS_DIR = HERE / "models"
YUNET_PATH = MODELS_DIR / "face_detection_yunet_2023mar.onnx"
SFACE_PATH = MODELS_DIR / "face_recognition_sface_2021dec.onnx"
ZOO = "https://github.com/opencv/opencv_zoo/raw/main/models/"
MODEL_URLS = {
    YUNET_PATH: ZOO + "face_detection_yunet/face_detection_yunet_2023mar.onnx",
    SFACE_PATH: ZOO + "face_recognition_sface/face_recognition_sface_2021dec.onnx",
}

# Les reseaux DNN ne sont pas thread-safe : worker et /enroll partagent ce verrou.
model_lock = threading.Lock()
engine = "haar"   # "dnn" (YuNet + SFace) ou "haar" (Haar + LBPH)
yunet = None
sface = None
haar = None
recognizer = None  # LBPH (repli)
label_names = {}   # id -> nom (LBPH)
gallery = {}       # nom -> matrice (n, 128) d'embeddings normalises (SFace)
tracks = []
next_track_id = 0


def dshow_names():
    """Noms DirectShow dans le meme ordre que les index OpenCV CAP_DSHOW."""
    try:
        from pygrabber.dshow_graph import FilterGraph
        return list(FilterGraph().get_input_devices())
    except Exception as e:  # noqa: BLE001
        log.warning("Impossible de lister les noms de webcam: %s", e)
        return []


def usb_camera_names(vid):
    """Noms Windows des cameras USB d'un fabricant (VID), ex. 046D = Logitech.

    Windows nomme souvent la C270 "USB Video Device" : le nom seul ne suffit pas
    a la reconnaitre, l'identifiant USB si.
    """
    if os.name != "nt" or not vid:
        return set()
    import subprocess
    cmd = (
        "Get-CimInstance Win32_PnPEntity | Where-Object { "
        "($_.PNPClass -eq 'Camera' -or $_.PNPClass -eq 'Image') -and "
        "$_.PNPDeviceID -like 'USB\\VID_%s*' } | ForEach-Object { $_.Name }" % vid.upper()
    )
    try:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", cmd],
            capture_output=True, text=True, timeout=15,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        ).stdout
    except Exception as e:  # noqa: BLE001
        log.warning("Impossible d'interroger les peripheriques USB: %s", e)
        return set()
    return {line.strip().lower() for line in out.splitlines() if line.strip()}


def pick_camera_index(names):
    """Index a essayer, la Logitech d'abord. Renvoie (indices, nb_preferes)."""
    if CAMERA_INDEX != "auto":
        return [int(CAMERA_INDEX)], 1
    if not names:
        return list(range(0, 6)), 0
    usb_logi = usb_camera_names(CAMERA_USB_VID)
    preferred, others = [], []
    for i, name in enumerate(names):
        n = name.lower()
        if (CAMERA_NAME and CAMERA_NAME in n) or n in usb_logi \
                or any(k in n for k in ("c270", "logitech", "logi")):
            log.info("Webcam %s : %s  <- Logitech", i, name)
            preferred.append(i)
        else:
            log.info("Webcam %s : %s", i, name)
            others.append(i)
    return preferred + others, len(preferred)


def open_camera():
    """Ouvre la C270. Sur Windows, DirectShow est plus fiable que MSMF."""
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    names = dshow_names() if os.name == "nt" else []
    with lock:
        state["devices"] = names
    indices, n_preferred = pick_camera_index(names)
    # On insiste sur la Logitech (elle peut mettre un moment a s'initialiser)
    # avant de se rabattre sur une autre webcam.
    for attempt in range(8):
        for idx in indices[:n_preferred] if attempt < 6 and n_preferred else indices:
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
                fallback = idx not in indices[:n_preferred]
                log.info("Webcam ouverte (index %s, %s, %sx%s)", idx, label, frame.shape[1], frame.shape[0])
                if fallback:
                    log.warning("Logitech introuvable : webcam de secours utilisee (%s)", label)
                with lock:
                    state["fallback"] = fallback
                return cap, idx
            cap.release()
        time.sleep(0.4)
    return None, None


def ensure_models():
    """Telecharge YuNet / SFace s'ils manquent (une seule fois)."""
    for path, url in MODEL_URLS.items():
        if path.exists() and path.stat().st_size > 100_000:
            continue
        try:
            log.info("Telechargement %s ...", path.name)
            tmp = path.with_suffix(".part")
            urllib.request.urlretrieve(url, tmp)
            tmp.replace(path)
        except Exception as e:  # noqa: BLE001
            log.warning("Telechargement de %s impossible : %s", path.name, e)


def load_haar():
    bundled = MODELS_DIR / "haarcascade_frontalface_default.xml"
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


def load_models():
    """YuNet + SFace si possible (bien plus fiable), sinon Haar + LBPH."""
    global engine, yunet, sface, haar
    ensure_models()
    if hasattr(cv2, "FaceDetectorYN") and YUNET_PATH.exists() and SFACE_PATH.exists():
        try:
            yunet = cv2.FaceDetectorYN.create(str(YUNET_PATH), "", (320, 320), YUNET_SCORE, 0.3, 5000)
            sface = cv2.FaceRecognizerSF.create(str(SFACE_PATH), "")
            engine = "dnn"
            log.info("Moteur visages : YuNet + SFace")
            return
        except cv2.error as e:
            log.warning("Modeles DNN inutilisables (%s), repli Haar + LBPH", e)
    haar = load_haar()
    engine = "haar"
    if not hasattr(cv2, "face"):
        log.warning("opencv-contrib absent : reconnaissance LBPH indisponible")
    log.info("Moteur visages : Haar + LBPH (moins fiable)")


def detect_faces(bgr, min_size=FACE_MIN_SIZE, score=None, max_side=None):
    """Liste de (x, y, w, h, reperes|None), boites bornees a l'image.

    max_side : detecte sur une copie reduite (bien plus rapide), coordonnees
    remises a l'echelle de l'image d'origine.
    """
    H, W = bgr.shape[:2]
    out = []
    if engine == "dnn":
        k = 1.0
        small = bgr
        if max_side and max(W, H) > max_side:
            k = max(W, H) / max_side
            small = cv2.resize(bgr, (int(W / k), int(H / k)), interpolation=cv2.INTER_AREA)
        with model_lock:
            yunet.setInputSize((small.shape[1], small.shape[0]))
            if score is not None:
                yunet.setScoreThreshold(score)
            try:
                _, faces = yunet.detect(small)
            finally:
                if score is not None:
                    yunet.setScoreThreshold(YUNET_SCORE)
        if faces is not None and k != 1.0:
            faces = faces.copy()
            faces[:, :14] *= k  # boite + 5 reperes ; la derniere colonne est le score
        for f in (faces if faces is not None else []):
            x, y, w, h = (int(round(v)) for v in f[:4])
            x1, y1 = max(0, x), max(0, y)
            x2, y2 = min(W, x + w), min(H, y + h)
            if min(x2 - x1, y2 - y1) < min_size:
                continue
            out.append((x1, y1, x2 - x1, y2 - y1, f))
        return out
    gray = cv2.equalizeHist(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    for (x, y, w, h) in haar.detectMultiScale(gray, 1.1, 6, minSize=(min_size, min_size)):
        out.append((int(x), int(y), int(w), int(h), None))
    return out


def _lbph_roi(gray, x, y, w, h):
    return cv2.equalizeHist(cv2.resize(gray[y:y + h, x:x + w], (160, 160)))


def embed(bgr, face):
    """Embedding SFace normalise (128,) d'un visage aligne sur ses reperes."""
    with model_lock:
        aligned = sface.alignCrop(bgr, face)
        feat = sface.feature(aligned)
    v = feat.flatten().astype(np.float32)
    return v / (np.linalg.norm(v) + 1e-9)


def _read_image(f):
    img = cv2.imdecode(np.fromfile(f, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    h0, w0 = img.shape[:2]
    if max(h0, w0) > 1280:
        scale = 1280 / max(h0, w0)
        img = cv2.resize(img, (int(w0 * scale), int(h0 * scale)))
    return img


def train_faces():
    """Apprend les visages de known_faces/<nom>/*.jpg (SFace ou LBPH)."""
    global recognizer, label_names, gallery
    if not KNOWN_DIR.exists():
        KNOWN_DIR.mkdir(parents=True, exist_ok=True)

    new_gallery = {}
    images, labels, names = [], [], {}
    total = 0
    for person_dir in sorted(p for p in KNOWN_DIR.iterdir() if p.is_dir()):
        files = [
            f for f in person_dir.iterdir()
            if f.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp")
        ]
        feats = []
        for f in files:
            img = _read_image(f)
            if img is None:
                log.warning("Photo illisible: %s", f.name)
                continue
            # Photos d'enrolement parfois petites : detection plus souple.
            found = detect_faces(img, min_size=24, score=0.6)
            if not found:
                # Ne JAMAIS apprendre une image sans visage : ca pollue le modele.
                log.warning("Aucun visage dans %s/%s : photo ignoree", person_dir.name, f.name)
                continue
            x, y, w, h, lm = max(found, key=lambda r: r[2] * r[3])
            if engine == "dnn":
                feats.append(embed(img, lm))
                flipped = cv2.flip(img, 1)
                ff = detect_faces(flipped, min_size=24, score=0.6)
                if ff:
                    feats.append(embed(flipped, max(ff, key=lambda r: r[2] * r[3])[4]))
            else:
                roi = _lbph_roi(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), x, y, w, h)
                feats += [roi, cv2.flip(roi, 1)]
        if not feats:
            continue
        total += len(feats)
        if engine == "dnn":
            new_gallery[person_dir.name] = np.stack(feats)
        else:
            pid = len(names)
            names[pid] = person_dir.name
            images.extend(feats)
            labels.extend([pid] * len(feats))

    if engine == "dnn":
        gallery = new_gallery
    elif images and hasattr(cv2, "face"):
        rec = cv2.face.LBPHFaceRecognizer_create()
        rec.train(images, np.array(labels, dtype=np.int32))
        recognizer, label_names = rec, names
    else:
        recognizer, label_names = None, {}

    people = known_people()
    if not people:
        log.info("Aucun visage connu : tout visage detecte sera 'inconnu'.")
    else:
        log.info("Reconnaissance prete : %d echantillon(s), personnes = %s", total, ", ".join(people))


def known_people():
    return list(gallery) if engine == "dnn" else list(label_names.values())


def identify(bgr, gray, face):
    """(nom | 'inconnu', score) pour UNE image ; la decision finale est votee."""
    x, y, w, h, lm = face
    if engine == "dnn":
        g = gallery
        if not g:
            return "inconnu", 0.0
        v = embed(bgr, lm)
        scores = sorted(((float(np.max(m @ v)), n) for n, m in g.items()), reverse=True)
        best, name = scores[0]
        second = scores[1][0] if len(scores) > 1 else -1.0
        if best >= SFACE_MIN_SCORE and best - second >= SFACE_MARGIN:
            return name, best
        return "inconnu", best
    if recognizer is None:
        return "inconnu", 999.0
    label, dist = recognizer.predict(_lbph_roi(gray, x, y, w, h))
    if dist <= LBPH_MAX_DISTANCE:
        return label_names.get(label, "inconnu"), float(dist)
    return "inconnu", float(dist)


def _iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    union = aw * ah + bw * bh - inter
    return inter / union if union else 0.0


def update_tracks(faces, now, identify_fn):
    """Associe les visages aux pistes existantes et vote l'identite.

    faces : sortie de detect_faces. identify_fn(face) -> (nom, score) n'est
    appele que si utile : une piste deja confirmee n'est re-verifiee que toutes
    les REID_S secondes (la reconnaissance coute bien plus que la detection).
    Renvoie (visages, decisions) :
    - visages : [(x, y, w, h, nom, score)] avec le nom CONFIRME : un prenom,
      'inconnu' (confirme) ou 'analyse' (pas encore sur) ;
    - decisions : [(boite, nom, score)] pour chaque piste dont la decision
      vient de changer (nouvelle personne autorisee / refusee).
    """
    global tracks, next_track_id
    tracks = [t for t in tracks if now - t["seen"] < 1.0]
    free = list(tracks)
    result = []
    changes = []
    for face in faces:
        x, y, w, h = face[:4]
        box = (x, y, w, h)
        best = max(free, key=lambda t: _iou(t["box"], box), default=None)
        if best is None or _iou(best["box"], box) < 0.25:
            best = {"id": next_track_id, "votes": deque(maxlen=VOTE_WINDOW),
                    "decided": "analyse", "id_t": 0.0, "score": 0.0}
            next_track_id += 1
            tracks.append(best)
        else:
            free.remove(best)
        best["box"] = box
        best["seen"] = now
        if best["decided"] == "analyse" or now - best["id_t"] >= REID_S:
            name, best["score"] = identify_fn(face)
            best["id_t"] = now
            best["votes"].append(name)
        score = best["score"]

        tally = Tally(best["votes"])
        n_unknown = tally.pop("inconnu", 0)
        top = tally.most_common(1)
        if top and top[0][1] >= VOTE_KNOWN and top[0][1] >= n_unknown:
            decided = top[0][0]
        elif n_unknown >= VOTE_UNKNOWN:
            decided = "inconnu"
        else:
            decided = "analyse"
        if decided != best["decided"] and decided != "analyse":
            changes.append((box, decided, score))
        best["decided"] = decided
        result.append((x, y, w, h, decided, score))
    return result, changes


def record_access(frame, box, name, score, now):
    """Ajoute une decision au journal d'acces, avec la photo du visage."""
    global next_access_id
    approved = name != "inconnu"
    x, y, w, h = box
    pad = int(0.3 * max(w, h))
    H, W = frame.shape[:2]
    crop = frame[max(0, y - pad):min(H, y + h + pad), max(0, x - pad):min(W, x + w + pad)]
    image = None
    if crop.size:
        k = 160 / crop.shape[0]
        thumb = cv2.resize(crop, (max(1, int(crop.shape[1] * k)), 160))
        ok, buf = cv2.imencode(".jpg", thumb, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if ok:
            image = datetime.fromtimestamp(now).strftime("access_%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
            buf.tofile(str(ACCESS_DIR / image))
    c_access.labels("autorise" if approved else "refuse").inc()
    with lock:
        event = {
            "id": next_access_id, "t": now, "name": name, "approved": approved,
            "score": round(float(score), 3), "image": image,
        }
        next_access_id += 1
        access_log.append(event)
        state["last_access"] = event
    log.info("Acces %s : %s (score %.3f)", "AUTORISE" if approved else "REFUSE", name, score)


train_lock = threading.Lock()


def retrain():
    with train_lock:
        train_faces()
    with lock:
        state["known_people"] = known_people()


def save_face(frame, box, nom):
    """Enregistre le visage (avec marge) dans known_faces/<nom>/. Renvoie le fichier ou None."""
    x, y, w, h = box
    # Marge large : le visage doit rester re-detectable dans la photo a l'entrainement.
    pad = int(0.6 * max(w, h))
    H, W = frame.shape[:2]
    crop = frame[max(0, y - pad):min(H, y + h + pad), max(0, x - pad):min(W, x + w + pad)]
    if crop.size == 0 or not detect_faces(crop, min_size=24, score=0.6):
        return None
    dest = KNOWN_DIR / nom
    dest.mkdir(parents=True, exist_ok=True)
    fname = datetime.now().strftime("enroll_%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
    # imencode + tofile : cv2.imwrite echoue sur les chemins accentues sous Windows.
    ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    if not ok:
        return None
    buf.tofile(str(dest / fname))
    return fname


def count_photos(nom):
    d = KNOWN_DIR / nom
    if not d.exists():
        return 0
    return sum(1 for f in d.iterdir() if f.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp"))


def enroll_public():
    """Vue JSON de la session d'enregistrement (a appeler sous `lock`)."""
    s = enroll_session
    if s is None:
        return None
    return {k: s[k] for k in ("name", "saved", "target", "hint", "done", "result")}


def finish_enroll(result):
    """Termine la session et re-entraine en arriere-plan (sans bloquer la video)."""
    with lock:
        s = enroll_session
        if s is None or s["done"]:
            return
        s.update(done=True, result=result, ended=time.time())
        if s["saved"]:
            s["hint"] = "%d photo(s) enregistree(s), apprentissage..." % s["saved"]
        else:
            s["hint"] = "Aucune photo enregistree"
        saved, name = s["saved"], s["name"]
    log.info("Enregistrement %s : %s (%d photo(s))", name, result, saved)
    if saved:
        threading.Thread(target=retrain, daemon=True).start()


def enroll_step(frame, gray, faces, now):
    """Une image du mode enregistrement auto : capture si la pose est nouvelle et nette."""
    global enroll_session
    with lock:
        s = enroll_session
    if s is None:
        return
    if s["done"]:
        # On laisse le resultat affiche quelques secondes puis on ferme la session.
        if now - s["ended"] > 6:
            with lock:
                enroll_session = None
        return
    if now < s["start_at"]:
        s["hint"] = "Prepare-toi... %d" % (int(s["start_at"] - now) + 1)
        return
    if now - s["start_at"] > ENROLL_TIMEOUT_S:
        finish_enroll("temps ecoule")
        return

    if len(faces) != 1:
        s["hint"] = "Place-toi face a la camera" if not faces else "Une seule personne devant la camera"
        return
    x, y, w, h, lm = faces[0]
    if min(w, h) < ENROLL_MIN_FACE:
        s["hint"] = "Approche-toi de la camera"
        return
    # Nettete mesuree a taille fixe (112 px, l'entree de SFace) : independante de la distance.
    roi = cv2.resize(gray[y:y + h, x:x + w], (112, 112))
    if cv2.Laplacian(roi, cv2.CV_64F).var() < ENROLL_MIN_SHARPNESS:
        s["hint"] = "Image floue : reste immobile un instant"
        return
    if now - s["last_t"] < ENROLL_INTERVAL_S:
        return

    v = None
    if engine == "dnn":
        v = embed(frame, lm)
        # Si la personne bouge peu, on assouplit apres 3 s pour ne jamais bloquer
        # (mais on refuse toujours une image quasi identique).
        limit = ENROLL_MAX_SIMILARITY if now - s["last_t"] < 3.0 else 0.98
        if s["feats"] and max(float(f @ v) for f in s["feats"]) > limit:
            s["hint"] = "Tourne legerement la tete (gauche, droite, haut, bas)"
            return

    fname = save_face(frame, (x, y, w, h), s["name"])
    if fname is None:
        s["hint"] = "Regarde l'objectif"
        return
    with lock:
        if v is not None:
            s["feats"].append(v)
        s["saved"] += 1
        s["last_t"] = now
        s["hint"] = "Photo %d/%d OK - change un peu d'angle" % (s["saved"], s["target"])
        complete = s["saved"] >= s["target"]
    if complete:
        finish_enroll("termine")


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
    # Contour par decalages (meme epaisseur) : avec OpenCV 5, un trait plus epais
    # change l'espacement des lettres et le contour ne tombe plus sous le texte.
    x, y = org
    for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1), (0, 2), (2, 0), (0, -2), (-2, 0)):
        cv2.putText(img, s, (x + dx, y + dy), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick, cv2.LINE_AA)
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


def annotate(frame, motion, faces_info, cam_name="CAM-01", analyze_age=0.0,
             enrolling=None, access=None):
    vis = frame.copy()
    H, W = vis.shape[:2]
    t = time.time()
    cyan = (220, 220, 40)
    green = (80, 220, 80)
    red = (50, 50, 255)
    white = (255, 255, 255)

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
        known = name not in ("inconnu", "analyse")
        refused = name == "inconnu"
        if known:
            color = green
        elif refused:
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
            sub = "ACCES AUTORISE  %.2f" % dist
            fill_ratio = 1.0
        elif refused:
            label = "VISAGE INCONNU"
            sub = "ACCES REFUSE"
            fill_ratio = 1.0
        else:
            label = "SCAN VISAGE"
            sub = "ANALYSE" + "." * (1 + int(t * 3) % 3)
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

    if enrolling:
        _enroll_overlay(vis, enrolling)
    elif access:
        _access_overlay(vis, access)
    return vis


def _access_overlay(vis, a):
    """Bandeau de la derniere decision : ACCES AUTORISE (vert) / REFUSE (rouge)."""
    H, W = vis.shape[:2]
    color = (60, 170, 60) if a["approved"] else (40, 40, 210)
    overlay = vis.copy()
    cv2.rectangle(overlay, (0, 34), (W, 84), color, -1)
    cv2.addWeighted(overlay, 0.7, vis, 0.3, 0, vis)
    if a["approved"]:
        title = "ACCES AUTORISE - " + a["name"].upper()
    else:
        title = "ACCES REFUSE - VISAGE INCONNU"
    _text(vis, title, (12, 58), 0.65, (255, 255, 255), 2)
    _text(vis, "score %.2f  -  %s" % (a["score"], datetime.fromtimestamp(a["t"]).strftime("%H:%M:%S")),
          (12, 77), 0.42, (235, 235, 235), 1)


def _enroll_overlay(vis, e):
    """Bandeau du mode enregistrement : prenom, progression, consigne."""
    H, W = vis.shape[:2]
    yellow = (0, 215, 255)
    green = (80, 220, 80)
    color = green if e["done"] and e["saved"] else yellow
    overlay = vis.copy()
    cv2.rectangle(overlay, (0, 34), (W, 96), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, vis, 0.45, 0, vis)
    title = "ENREGISTREMENT %s  %d/%d" % (e["name"].upper(), e["saved"], e["target"])
    _text(vis, title, (12, 56), 0.6, color, 2)
    _text(vis, e["hint"] or "", (12, 82), 0.5, (255, 255, 255), 1)
    fill = int((W - 24) * min(1.0, e["saved"] / max(1, e["target"])))
    cv2.rectangle(vis, (12, 90), (W - 12, 93), (60, 60, 60), -1)
    cv2.rectangle(vis, (12, 90), (12 + max(2, fill), 93), color, -1)


def worker():
    # latest_bgr / last_face_boxes doivent etre globaux, sinon /enroll ne voit jamais l'image.
    global latest_jpeg, latest_bgr, last_face_boxes
    global last_motion_t, last_unknown_t, motion_armed
    cap, idx = open_camera()
    with lock:
        state["up"] = cap is not None
        state["index"] = idx
        devices = state.get("devices") or []
        state["name"] = devices[idx] if idx is not None and idx < len(devices) else None
        state["error"] = None if cap else "webcam introuvable"
        state["known_people"] = known_people()
        state["engine"] = engine
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
                devices = state.get("devices") or []
                state["name"] = devices[idx] if idx < len(devices) else None
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

        faces_raw = detect_faces(frame, max_side=DETECT_MAX_SIDE)
        enroll_step(frame, gray, faces_raw, now)
        with lock:
            enrolling = enroll_public()
        faces_info, decisions = update_tracks(faces_raw, now, lambda f: identify(frame, gray, f))
        enrolling_active = bool(enrolling and not enrolling["done"])
        for box, name, score in decisions:
            # Pendant un enregistrement, la personne est forcement "inconnue" : pas de refus logge.
            if not (enrolling_active and name == "inconnu"):
                record_access(frame, box, name, score, now)
        known_now, unknown_now, pending_now = [], 0, 0
        for (_x, _y, _w, _h, name, _s) in faces_info:
            if name == "inconnu":
                unknown_now += 1
            elif name == "analyse":
                pending_now += 1
            else:
                known_now.append(name)

        if (unknown_now or pending_now) and analyze_since is None:
            analyze_since = now
        if unknown_now and enrolling_active:
            # La personne en cours d'enregistrement est forcement "inconnue" : pas d'alerte.
            unknown_now = 0
        if unknown_now:
            if now - last_unknown_t > FACE_HOLD_S:
                c_unknown.inc()
                save_snapshot(frame, "unknown")
                with lock:
                    state["last_event"] = "visage inconnu " + datetime.now().strftime("%H:%M:%S")
            last_unknown_t = now
        elif not pending_now and analyze_since is not None and now - last_unknown_t > 1.0:
            analyze_since = None

        analyze_age = (now - analyze_since) if analyze_since is not None else 0.0

        unknown_held = 1 if (now - last_unknown_t) < FACE_HOLD_S else 0

        with lock:
            last_access = state["last_access"]
        if last_access and now - last_access["t"] > ACCESS_BANNER_S:
            last_access = None
        vis = annotate(
            frame, motion_held, faces_info,
            cam_name="CAM-01", analyze_age=analyze_age, enrolling=enrolling,
            access=None if enrolling_active else last_access,
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
                "known_people": known_people(),
                "enroll": enroll_public(),
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


@app.route("/access")
def access():
    """Journal des decisions d'acces, la plus recente en premier."""
    with lock:
        events = list(access_log)[::-1]
    return jsonify(events=events, engine=engine)


@app.route("/access/<fichier>")
def access_image(fichier):
    # Seules les photos referencees par le journal sont servies (pas de parcours de dossier).
    with lock:
        known = any(e["image"] == fichier for e in access_log)
    if not known:
        abort(404)
    return send_from_directory(ACCESS_DIR, fichier, mimetype="image/jpeg", max_age=3600)


@app.route("/snapshot")
def snapshot():
    with lock:
        frame = latest_jpeg
    if frame is None:
        return jsonify(erreur="pas d'image"), 503
    return Response(frame, mimetype="image/jpeg")


@app.route("/reload-faces", methods=["GET", "POST"])
def reload_faces():
    retrain()
    return jsonify(ok=True, personnes=known_people())


def _safe_name(raw):
    raw = (raw or "").strip()
    cleaned = "".join(c if (c.isalnum() or c in "-_ ") else "" for c in raw).strip()
    return cleaned[:40]


def _nom_requete():
    payload = request.get_json(silent=True) or {}
    return payload, _safe_name(payload.get("name") or request.args.get("name") or request.form.get("name"))


@app.route("/enroll", methods=["POST"])
def enroll():
    """Capture le visage actuel (une photo) et l'associe a un prenom."""
    _payload, nom = _nom_requete()
    if not nom:
        return jsonify(ok=False, erreur="Indique un prenom."), 400

    with lock:
        frame = None if latest_bgr is None else latest_bgr.copy()
        boxes = list(last_face_boxes)

    if frame is None:
        return jsonify(ok=False, erreur="Pas d'image camera."), 503
    if not boxes:
        return jsonify(ok=False, erreur="Aucun visage devant la camera. Place-toi face a l'objectif."), 400
    if len(boxes) > 1:
        return jsonify(ok=False, erreur="Plusieurs visages visibles : une seule personne devant la camera."), 400

    fname = save_face(frame, boxes[0], nom)
    if fname is None:
        return jsonify(ok=False, erreur="Visage flou ou de profil. Regarde l'objectif et reessaie."), 400
    retrain()
    n = count_photos(nom)
    log.info("Visage enregistre : %s (%s) - %d photo(s)", nom, fname, n)
    return jsonify(ok=True, nom=nom, fichier=fname, photos=n, personnes=known_people())


@app.route("/enroll/start", methods=["POST"])
def enroll_start():
    """Mode enregistrement auto : la camera prend elle-meme N photos variees."""
    global enroll_session
    payload, nom = _nom_requete()
    if not nom:
        return jsonify(ok=False, erreur="Indique un prenom."), 400
    try:
        target = max(1, min(30, int(payload.get("count") or ENROLL_COUNT)))
    except (TypeError, ValueError):
        target = ENROLL_COUNT
    with lock:
        if not state["up"]:
            return jsonify(ok=False, erreur="Camera hors ligne."), 503
        if enroll_session is not None and not enroll_session["done"]:
            return jsonify(ok=False, erreur="Enregistrement deja en cours (%s)." % enroll_session["name"]), 409
        now = time.time()
        enroll_session = {
            "name": nom, "target": target, "saved": 0, "feats": [],
            "start_at": now + 2.0, "last_t": 0.0, "ended": 0.0,
            "hint": "Prepare-toi...", "done": False, "result": None,
        }
        info = enroll_public()
    log.info("Mode enregistrement : %s (%d photos)", nom, target)
    return jsonify(ok=True, enroll=info)


@app.route("/enroll/stop", methods=["POST"])
def enroll_stop():
    finish_enroll("arrete")
    with lock:
        return jsonify(ok=True, enroll=enroll_public())


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
    load_models()
    train_faces()
    threading.Thread(target=worker, daemon=True).start()
    log.info("Camera HTTP sur http://127.0.0.1:%s", PORT)
    serve(app, host="0.0.0.0", port=PORT, threads=6)


if __name__ == "__main__":
    main()
