"""
identification.py — MODULE 2 : identification des obstacles avec YOLOv8n
=======================================================================

Ajoute à la cartographie (Module 1) la RECONNAISSANCE des objets :
  - YOLOv8n (modèle pré-entraîné, téléchargé automatiquement la 1re fois, ~6 Mo)
    analyse l'IMAGE BRUTE de la caméra (YOLO reconnaît mal les objets vus du dessus) ;
  - "personne" et "chaise" sont reconnues, tout autre objet détecté devient "obstacle" ;
  - chaque objet est reporté sur la carte (cercle de couleur + libellé) et compte
    comme obstacle pour le calcul du trajet ;
  - la détection par différence d'images du Module 1 reste active : un obstacle
    que YOLO ne reconnaît pas reste quand même rouge sur la carte (filet de sécurité).

YOLO tourne en arrière-plan sur le processeur (CPU) : la carte reste fluide même si
YOLO n'analyse que 2 à 5 images par seconde.

Lancement (mêmes touches que cartographie.py) :
    python identification.py coin       (ou dalle)
Test de YOLO seul, sans calibration ni marqueurs :
    python identification.py --test
"""
import sys
import threading
import time

import cv2
import numpy as np

import cartographie as carto

# =============================== PARAMÈTRES ===============================
MODELE = "yolov8n.pt"          # "n" = nano, le plus léger (adapté au CPU)
CONFIANCE_MIN = 0.35           # en dessous, la détection est ignorée
TAILLE_YOLO = 640              # taille d'analyse (416 = plus rapide, moins précis)
PAUSE_ENTRE_ANALYSES_S = 0.0   # mettre 0.2 pour laisser du CPU au reste si le PC rame

# Classes COCO reconnues : numéro -> (libellé, couleur BGR, rayon d'encombrement au sol en cm)
CATEGORIES = {
    0:  ("personne", (200, 0, 200), 25),     # violet
    56: ("chaise",   (0, 200, 230), 30),     # jaune
}
AUTRE = ("obstacle", (90, 90, 90), 20)        # toute autre classe détectée (gris)

# Quel point de la boîte YOLO touche le sol ?
#   coin  (vue en biais) : le bas de la boîte = les pieds de la personne / de la chaise
#   dalle (vue du dessus) : le centre de la boîte
POINT_SOL = {"coin": "bas", "dalle": "centre"}
IGNORER_PRES_ROBOT_CM = 30     # le Yanshee peut être pris pour une "personne" : on l'ignore
HORS_ZONE_TOLERANCE_CM = 10    # objets un peu en dehors des marqueurs quand même gardés
# ==========================================================================


class IdentificateurYOLO:
    def __init__(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            raise SystemExit("ultralytics n'est pas installe : pip install ultralytics  (voir README.md)")
        print("Chargement de YOLO (la 1re fois : telechargement du modele)...")
        self.modele = YOLO(MODELE)
        self.ips = 0.0                       # images analysées par seconde
        self._image = None
        self._detections = []
        self._verrou = threading.Lock()
        threading.Thread(target=self._boucle, daemon=True).start()

    # --- appelé par la boucle de la cartographie à chaque image ---
    def soumettre(self, frame):
        """Donne la dernière image à YOLO (s'il est occupé, l'ancienne est simplement remplacée)."""
        with self._verrou:
            self._image = frame

    def _boucle(self):
        """Tourne en arrière-plan : analyse la dernière image reçue, encore et encore."""
        while True:
            with self._verrou:
                img, self._image = self._image, None
            if img is None:
                time.sleep(0.01); continue
            t0 = time.time()
            res = self.modele.predict(img, imgsz=TAILLE_YOLO, conf=CONFIANCE_MIN, device="cpu", verbose=False)[0]
            dets = []
            for boite, conf, cls in zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist(), res.boxes.cls.tolist()):
                libelle, couleur, rayon = CATEGORIES.get(int(cls), AUTRE)
                dets.append({"boite": boite, "conf": conf, "libelle": libelle, "couleur": couleur,
                             "r": rayon, "classe_yolo": res.names[int(cls)]})
            with self._verrou:
                self._detections = dets
            self.ips = 0.7 * self.ips + 0.3 / max(time.time() - t0, 1e-3)
            time.sleep(PAUSE_ENTRE_ANALYSES_S)

    def detections(self):
        with self._verrou:
            return list(self._detections)

    def objets_cm(self, H, pose, position):
        """Objets reconnus, convertis en positions au sol (cm) sur la carte."""
        objets = []
        for d in self.detections():
            x1, y1, x2, y2 = d["boite"]
            p = ((x1 + x2) / 2, y2) if POINT_SOL.get(position) == "bas" else ((x1 + x2) / 2, (y1 + y2) / 2)
            x, y = carto.vers_cm([p], H)[0]
            t = HORS_ZONE_TOLERANCE_CM
            if not (-t <= x <= carto.ROOM_W + t and -t <= y <= carto.ROOM_H + t):
                continue                                         # hors de la zone surveillée
            if pose and np.hypot(x - pose[0][0], y - pose[0][1]) < IGNORER_PRES_ROBOT_CM:
                continue                                         # c'est sans doute le robot lui-même
            objets.append({**d, "x": float(np.clip(x, 0, carto.ROOM_W)), "y": float(np.clip(y, 0, carto.ROOM_H))})
        return objets

    # --- dessin ---
    def dessiner_camera(self, view):
        for d in self.detections():
            x1, y1, x2, y2 = map(int, d["boite"])
            cv2.rectangle(view, (x1, y1), (x2, y2), d["couleur"], 2)
            texte = f"{d['libelle']} {d['conf']:.0%}" + ("" if d["libelle"] != "obstacle" else f" ({d['classe_yolo']})")
            cv2.putText(view, texte, (x1, max(y1 - 6, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, d["couleur"], 2)
        cv2.putText(view, f"YOLO {self.ips:.1f} img/s", (10, view.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    def dessiner_carte(self, carte, objets):
        k = carto.MAP_PX / carto.CELL                            # pixels de carte par cm
        for o in objets:
            c = (int(o["x"] * k), int(o["y"] * k))
            cv2.circle(carte, c, int(o["r"] * k), o["couleur"], 2)
            cv2.circle(carte, c, 4, o["couleur"], -1)
            cv2.putText(carte, o["libelle"], (c[0] + 6, c[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 4)
            cv2.putText(carte, o["libelle"], (c[0] + 6, c[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, o["couleur"], 1)


def test_seul():
    """YOLO sur l'image de la caméra, sans cartographie : pour vérifier l'installation."""
    cap = cv2.VideoCapture(carto.CAMERA_SOURCE, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, carto.CAMERA_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, carto.CAMERA_H)
    if not cap.isOpened():
        raise SystemExit("Camera introuvable : changez CAMERA_SOURCE dans cartographie.py")
    ident = IdentificateurYOLO()
    print("Test YOLO : q pour quitter")
    while True:
        frame = carto.grab(cap)
        if frame is None: continue
        ident.soumettre(frame)
        ident.dessiner_camera(frame)
        cv2.imshow("test YOLO", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"): break
    cap.release(); cv2.destroyAllWindows()


if __name__ == "__main__":
    if "--test" in sys.argv:
        test_seul()
    else:
        carto.main(sys.argv[1] if len(sys.argv) > 1 else None, identificateur=IdentificateurYOLO())
