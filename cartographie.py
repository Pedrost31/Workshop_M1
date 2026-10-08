#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cartographie.py - A LANCER SUR LE PC (webcam Logitech branchee en USB)

La webcam, fixee au-dessus de la zone de marche, regarde le sol :
  - 4 marqueurs ArUco (dictionnaire 4x4_50, ID 0 a 3) poses aux coins
    definissent un rectangle de LARGEUR x HAUTEUR cm ;
  - le marqueur ID 10, pose a plat sur la tete du robot, donne sa position et
    son orientation (cap) ;
  - les grands obstacles poses dans la zone sont reperes par difference avec
    une photo du sol vide (touche R ou bouton de l'interface).

Reperes (en cm), vus d'en haut :

    0 ------- x -------> 1      0, 1, 2, 3 dans le sens des aiguilles d'une montre
    |                    |      LARGEUR = distance entre les CENTRES de 0 et 1
    y       zone         |      HAUTEUR = distance entre les CENTRES de 1 et 2
    v                    |
    3 ------------------ 2      cap du robot : 0 = vers +x, 90 = vers +y (a droite)

La carte est publiee en HTTP (port 8081) ; capteurs_serveur.py la relaie sur son
port 8080 pour l'interface web et marche_obstacle.py :
    GET  /carte       {"zone": {"largeur": 200, "hauteur": 150}, "calibre": true,
                       "robot": {"x": 87.2, "y": 40.1, "cap": 93, "age": 0.05} | null,
                       "obstacles": [{"x": 120, "y": 30, "l": 40, "h": 35}],
                       "grille": {"cellule": 10, "lignes": ["....##..", ...]},
                       "reference": true, "erreur": null, "age": 0.03}
    GET  /carte.jpg   vue de dessus annotee (pour regler la camera)
    POST /reference   photographie le sol vide (zone degagee, robot hors de vue)

    python -m pip install opencv-python
    $env:ZONE="200x150"; python cartographie.py           (PowerShell)
    ZONE=200x150 python3 cartographie.py
    Options : --sans-fenetre ; CAMERA=1 (autre webcam) ; CAMERA=photo.jpg (test sans webcam)
    Fenetres : R = photo du sol vide, Q = quitter.
"""

import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("OpenCV manquant : python -m pip install opencv-python")

PORT_HTTP = int(os.environ.get("PORT_CARTE", "8081"))
CAMERA = os.environ.get("CAMERA", "0")             # numero de la webcam, ou image / video pour tester
RESOLUTION = os.environ.get("RESOLUTION", "1920x1080")
ZONE = os.environ.get("ZONE", "200x150")           # largeur x hauteur en cm, entre les centres des marqueurs

COINS = (0, 1, 2, 3)     # marqueurs des coins, dans le sens des aiguilles d'une montre vu d'en haut
ID_ROBOT = 10            # marqueur sur la tete du robot, le haut du marqueur vers l'avant du robot
# Marqueur colle de travers : poser le robot face au marqueur 1, lire le cap affiche
# (par exemple 85) et lancer avec CAP_DECALAGE_DEG=-85.
CAP_DECALAGE_DEG = float(os.environ.get("CAP_DECALAGE_DEG", "0"))
# Le marqueur du robot est en hauteur : vu d'en haut, il parait decale vers
# l'exterieur de l'image. Donner la hauteur de la camera (cm) pour corriger.
HAUTEUR_CAMERA_CM = float(os.environ.get("HAUTEUR_CAMERA_CM", "0"))   # 0 = pas de correction
HAUTEUR_MARQUEUR_CM = float(os.environ.get("HAUTEUR_MARQUEUR_CM", "37"))  # tete du Yanshee

PX_PAR_CM = 2            # resolution de la vue de dessus
SEUIL_DIFF = 35          # difference de gris (0-255) avec le sol vide pour compter comme obstacle
OBSTACLE_AIRE_MIN_CM2 = 150   # plus petit : bruit, ombre, scotch...
ROBOT_MASQUE_CM = 25     # le robot lui-meme n'est pas un obstacle
ROBOT_OUBLI_S = 3        # on masque sa derniere position connue pendant 3 s
CELLULE_CM = 10          # taille des cases de la grille d'occupation

DOSSIER = os.path.dirname(os.path.abspath(__file__))
FICHIER_REFERENCE = os.path.join(DOSSIER, "sol_reference.png")


def lire_zone(texte):
    """"200x150" -> (200.0, 150.0)."""
    largeur, _, hauteur = texte.lower().partition("x")
    return float(largeur), float(hauteur)


def normaliser(angle):
    """Angle en degres ramene dans [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def vers_sol(points, H):
    """Points image (px) -> points sol (cm) par l'homographie H."""
    return cv2.perspectiveTransform(np.float32(points).reshape(-1, 1, 2), H).reshape(-1, 2)


def pose_robot(coins, H):
    """Coins du marqueur du robot dans l'image (ordre ArUco : haut-gauche,
    haut-droit, bas-droit, bas-gauche) -> (x, y, cap) sur le sol."""
    centre = coins.mean(axis=0)
    avant = (coins[0] + coins[1]) / 2.0       # milieu du bord haut du marqueur
    (x, y), (ax, ay) = vers_sol([centre, avant], H)
    cap = normaliser(math.degrees(math.atan2(ay - y, ax - x)) + CAP_DECALAGE_DEG)
    return float(x), float(y), cap


def corriger_parallaxe(x, y, nadir, h_camera, h_marqueur):
    """Un point a h_marqueur du sol parait plus loin du nadir (point du sol
    sous la camera) qu'il ne l'est : on le ramene vers le nadir."""
    if h_camera <= h_marqueur or h_camera <= 0:
        return x, y
    k = 1 - h_marqueur / h_camera
    return nadir[0] + (x - nadir[0]) * k, nadir[1] + (y - nadir[1]) * k


def gris(image):
    return cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (5, 5), 0)


def detecter_obstacles(vue, reference, ignorer=None):
    """Compare la vue de dessus au sol vide. Renvoie (obstacles, masque) :
    obstacles = [{"x", "y", "l", "h"} en cm], masque = pixels occupes (0 / 255)."""
    a, b = gris(vue), gris(reference)
    # l'exposition automatique de la webcam change la luminosite globale : on la compense
    decalage = float(np.median(b)) - float(np.median(a))
    a = np.clip(a.astype(np.int16) + int(round(decalage)), 0, 255).astype(np.uint8)
    _, masque = cv2.threshold(cv2.absdiff(a, b), SEUIL_DIFF, 255, cv2.THRESH_BINARY)
    if ignorer is not None:
        masque[ignorer > 0] = 0
    masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    masque = cv2.morphologyEx(masque, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacles = []
    garde = np.zeros_like(masque)
    for contour in contours:
        if cv2.contourArea(contour) / PX_PAR_CM ** 2 < OBSTACLE_AIRE_MIN_CM2:
            continue
        x, y, l, h = cv2.boundingRect(contour)
        obstacles.append({"x": round(x / float(PX_PAR_CM), 1), "y": round(y / float(PX_PAR_CM), 1),
                          "l": round(l / float(PX_PAR_CM), 1), "h": round(h / float(PX_PAR_CM), 1)})
        cv2.drawContours(garde, [contour], -1, 255, cv2.FILLED)
    return obstacles, garde


def grille(masque, largeur, hauteur):
    """Masque d'occupation -> ["..##..", ...] : une case de CELLULE_CM, '#' si
    plus de 20 % de la case est occupee."""
    colonnes = max(1, int(math.ceil(largeur / CELLULE_CM)))
    lignes = max(1, int(math.ceil(hauteur / CELLULE_CM)))
    petit = cv2.resize(masque, (colonnes, lignes), interpolation=cv2.INTER_AREA)
    return ["".join("#" if v > 0.2 * 255 else "." for v in ligne) for ligne in petit]


class Cartographe(object):
    """Transforme les images de la webcam en carte. Sans camera ni HTTP : testable."""

    def __init__(self, largeur, hauteur, reference=None):
        self.largeur, self.hauteur = largeur, hauteur
        self.taille_vue = (int(round(largeur * PX_PAR_CM)), int(round(hauteur * PX_PAR_CM)))
        parametres = cv2.aruco.DetectorParameters()
        parametres.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detecteur = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), parametres)
        self.H = None             # image (px) -> sol (cm), garde si un coin est cache
        self.coins_px = None      # centres des marqueurs 0..3 dans l'image
        self.erreur = "marqueurs de coin %s pas encore vus" % ", ".join(map(str, COINS))
        self.marqueurs = {}       # id -> coins (px) dans la derniere image
        self.robot = None         # {"x", "y", "cap", "t", "tete"}
        self.vue = None           # derniere vue de dessus (BGR, PX_PAR_CM px par cm)
        self.reference = reference if self._reference_valide(reference) else None
        self.obstacles = []
        self.lignes = None        # grille d'occupation

    def _reference_valide(self, reference):
        return reference is not None and reference.shape[1::-1] == self.taille_vue

    def traiter(self, image, t):
        coins, ids, _ = self.detecteur.detectMarkers(image)
        self.marqueurs = {} if ids is None else dict(
            (int(i), c.reshape(4, 2)) for i, c in zip(ids.flatten(), coins))
        self._calibrer()
        if self.H is None:
            return
        if ID_ROBOT in self.marqueurs:
            x, y, cap = pose_robot(self.marqueurs[ID_ROBOT], self.H)
            hauteur_px, largeur_px = image.shape[:2]
            nadir = vers_sol([[largeur_px / 2.0, hauteur_px / 2.0]], self.H)[0]
            xc, yc = corriger_parallaxe(x, y, nadir, HAUTEUR_CAMERA_CM, HAUTEUR_MARQUEUR_CM)
            self.robot = {"x": xc, "y": yc, "cap": cap, "t": t, "tete": (x, y)}
        echelle = np.diag([PX_PAR_CM, PX_PAR_CM, 1.0])
        self.vue = cv2.warpPerspective(image, echelle.dot(self.H), self.taille_vue)
        if self.reference is not None:
            self.obstacles, occupe = detecter_obstacles(self.vue, self.reference, self._masque_robot(t))
            self.lignes = grille(occupe, self.largeur, self.hauteur)
        else:
            self.obstacles, self.lignes = [], None

    def _calibrer(self):
        if not all(i in self.marqueurs for i in COINS):
            if self.H is None:
                manquants = [str(i) for i in COINS if i not in self.marqueurs]
                self.erreur = "marqueurs de coin invisibles : %s" % ", ".join(manquants)
            return                # camera fixe : on garde la calibration precedente
        centres = np.float32([self.marqueurs[i].mean(axis=0) for i in COINS])
        x1, y1 = centres[1] - centres[0]
        x3, y3 = centres[3] - centres[0]
        if x1 * y3 - y1 * x3 <= 0:
            self.H = None
            self.erreur = ("marqueurs 0, 1, 2, 3 poses dans le sens inverse des aiguilles "
                           "d'une montre : echanger les marqueurs 1 et 3")
            return
        sol = np.float32([[0, 0], [self.largeur, 0], [self.largeur, self.hauteur], [0, self.hauteur]])
        self.H = cv2.getPerspectiveTransform(centres, sol)
        self.coins_px = centres
        self.erreur = None

    def _masque_robot(self, t):
        """Pixels de la vue a ne pas compter comme obstacle : autour du robot
        (pieds et tete, qui ne sont pas au meme endroit vus d'en haut)."""
        masque = np.zeros(self.taille_vue[::-1], np.uint8)
        if self.robot and t - self.robot["t"] < ROBOT_OUBLI_S:
            for x, y in ((self.robot["x"], self.robot["y"]), self.robot["tete"]):
                cv2.circle(masque, (int(x * PX_PAR_CM), int(y * PX_PAR_CM)),
                           int(ROBOT_MASQUE_CM * PX_PAR_CM), 255, cv2.FILLED)
        return masque

    def prendre_reference(self, t):
        """Photo du sol vide. Renvoie (True, message) ou (False, raison)."""
        if self.vue is None:
            return False, "zone pas encore calibree : %s" % self.erreur
        if ID_ROBOT in self.marqueurs:
            return False, "le robot est visible : le sortir de la zone avant la photo du sol vide"
        self.reference = self.vue.copy()
        try:
            cv2.imwrite(FICHIER_REFERENCE, self.reference)
        except cv2.error:
            pass
        print("[carte] photo du sol vide enregistree (%s)" % FICHIER_REFERENCE)
        return True, "photo du sol vide enregistree"

    def carte(self, t):
        """Etat publie sur /carte. robot["age"] est recalcule a chaque requete."""
        robot = None
        if self.robot:
            robot = {"x": round(self.robot["x"], 1), "y": round(self.robot["y"], 1),
                     "cap": int(round(self.robot["cap"])), "t": self.robot["t"]}
        return {"zone": {"largeur": self.largeur, "hauteur": self.hauteur},
                "calibre": self.H is not None, "erreur": self.erreur, "robot": robot,
                "obstacles": self.obstacles, "reference": self.reference is not None,
                "grille": {"cellule": CELLULE_CM, "lignes": self.lignes} if self.lignes else None,
                "marqueurs": sorted(self.marqueurs), "t": t}

    def dessiner_vue(self, t):
        """Vue de dessus annotee : obstacles en rouge, robot en vert (gris si perdu)."""
        if self.vue is None:
            return None
        image = self.vue.copy()
        s = PX_PAR_CM
        for o in self.obstacles:
            cv2.rectangle(image, (int(o["x"] * s), int(o["y"] * s)),
                          (int((o["x"] + o["l"]) * s), int((o["y"] + o["h"]) * s)), (60, 60, 255), 2)
        if self.robot:
            couleur = (100, 220, 60) if t - self.robot["t"] < 1 else (150, 150, 150)
            x, y, a = self.robot["x"] * s, self.robot["y"] * s, math.radians(self.robot["cap"])
            cv2.circle(image, (int(x), int(y)), int(12 * s), couleur, 2)
            cv2.arrowedLine(image, (int(x), int(y)),
                            (int(x + 30 * s * math.cos(a)), int(y + 30 * s * math.sin(a))), couleur, 2)
        if self.reference is None:
            cv2.putText(image, "R : photo du sol vide", (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 200, 255), 2)
        return image

    def dessiner_camera(self, image):
        """Image de la webcam avec les marqueurs vus et le contour de la zone."""
        image = image.copy()
        for i, coins in self.marqueurs.items():
            cv2.polylines(image, [coins.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.putText(image, str(i), tuple(int(v) for v in coins.mean(axis=0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        if self.coins_px is not None:
            cv2.polylines(image, [self.coins_px.astype(np.int32)], True, (255, 200, 0), 2)
        if self.erreur:
            cv2.putText(image, self.erreur, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        return image


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

class ImageFixe(object):
    """CAMERA=photo.jpg : rejoue la meme image, pour tester sans webcam."""

    def __init__(self, chemin):
        self.image = cv2.imread(chemin)

    def isOpened(self):
        return self.image is not None

    def read(self):
        time.sleep(0.1)
        return True, self.image.copy()


def ouvrir_camera(source):
    if source.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
        return ImageFixe(source)
    if not source.isdigit():
        return cv2.VideoCapture(source)            # fichier video ou flux
    # DirectShow : la webcam s'ouvre en 1 s au lieu de 10 sous Windows
    camera = cv2.VideoCapture(int(source), cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
    largeur, _, hauteur = RESOLUTION.partition("x")
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # 30 i/s en 1080p
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(largeur))
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(hauteur))
    return camera


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

verrou = threading.Lock()         # le cartographe (image en cours de traitement)
partage = {"carte": None, "vue": None, "t": 0.0}   # dernier etat publie
verrou_partage = threading.Lock()


def carte_publiee(maintenant):
    with verrou_partage:
        carte = partage["carte"]
    if carte is None:
        return {"calibre": False, "erreur": "pas encore d'image de la webcam", "robot": None,
                "obstacles": [], "age": None}
    carte = dict(carte)
    carte["age"] = round(maintenant - carte.pop("t"), 2)
    if carte["robot"]:
        robot = dict(carte["robot"])
        robot["age"] = round(maintenant - robot.pop("t"), 2)
        carte["robot"] = robot
    return carte


class Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path.startswith("/carte.jpg"):
            with verrou_partage:
                vue = partage["vue"]
            if vue is None:
                self._repondre(404, {"erreur": "zone pas encore calibree"})
                return
            self._envoyer(200, "image/jpeg", cv2.imencode(".jpg", vue)[1].tobytes())
        elif self.path.startswith("/carte"):
            self._repondre(200, carte_publiee(time.time()))
        else:
            self._repondre(404, {"erreur": "GET /carte , /carte.jpg ou POST /reference"})

    def do_POST(self):
        if not self.path.startswith("/reference"):
            self._repondre(404, {"erreur": "POST /reference"})
            return
        with verrou:
            ok, message = cartographe.prendre_reference(time.time())
        self._repondre(200 if ok else 409, {"message": message} if ok else {"erreur": message})

    def _repondre(self, code, corps):
        self._envoyer(code, "application/json", json.dumps(corps).encode())

    def _envoyer(self, code, type_, donnees):
        self.send_response(code)
        self.send_header("Content-Type", type_)
        self.send_header("Content-Length", str(len(donnees)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(donnees)

    def log_message(self, *args):
        pass


class ServeurMultiTache(ThreadingMixIn, HTTPServer):
    daemon_threads = True


cartographe = None


def main():
    global cartographe
    fenetre = "--sans-fenetre" not in sys.argv
    largeur, hauteur = lire_zone(ZONE)
    reference = cv2.imread(FICHIER_REFERENCE) if os.path.exists(FICHIER_REFERENCE) else None
    cartographe = Cartographe(largeur, hauteur, reference)
    if reference is not None and cartographe.reference is None:
        print("[carte] %s ignore : prise pour une autre taille de zone" % FICHIER_REFERENCE)
    camera = ouvrir_camera(CAMERA)
    if not camera.isOpened():
        sys.exit("Webcam %s introuvable (essayer CAMERA=1)" % CAMERA)
    serveur = ServeurMultiTache(("0.0.0.0", PORT_HTTP), Handler)
    threading.Thread(target=serveur.serve_forever, daemon=True).start()
    print("Zone %g x %g cm, webcam %s, carte : http://localhost:%d/carte"
          % (largeur, hauteur, CAMERA, PORT_HTTP))
    deja_dit = None
    while True:
        ok, image = camera.read()
        if not ok:
            print("[camera] plus d'image : webcam debranchee ?")
            time.sleep(0.5)
            continue
        t = time.time()
        with verrou:
            cartographe.traiter(image, t)
            carte = cartographe.carte(t)
            vue = cartographe.dessiner_vue(t)
        with verrou_partage:
            partage.update(carte=carte, vue=vue, t=t)
        if carte["erreur"] != deja_dit:
            print("[carte] %s" % (carte["erreur"] or "zone calibree"))
            deja_dit = carte["erreur"]
        if fenetre:
            cv2.imshow("Camera", cartographe.dessiner_camera(image))
            if vue is not None:
                cv2.imshow("Carte (vue de dessus)", vue)
            touche = cv2.waitKey(1) & 0xFF
            if touche in (ord("q"), 27):
                break
            if touche == ord("r"):
                with verrou:
                    print("[carte] %s" % cartographe.prendre_reference(t)[1])


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
