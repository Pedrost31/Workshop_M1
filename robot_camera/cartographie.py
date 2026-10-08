"""
cartographie.py — MODULE 1 : cartographie de la zone et guidage du robot
=======================================================================

Ce que fait ce script :
  1. Lit l'image de la webcam C270.
  2. Repère les 4 marqueurs ArUco posés aux coins (IDs 0,1,2,3) et calcule une
     "homographie" : une transformation qui redresse l'image en VUE DU DESSUS,
     que la caméra soit au plafond (position "dalle") ou en biais dans un coin
     (position "coin").
  3. Compare l'image actuelle à une image de référence de la zone vide :
     tout ce qui a changé est un obstacle (le robot, marqueur 10, est masqué).
  4. Construit une grille (cases libres / occupées), l'élargit d'une marge de
     sécurité réglable, et cherche le trajet le plus court avec l'algorithme A*.
  5. Affiche une carte "type Google Maps" et guide le robot en BOUCLE FERMÉE :
     le robot fait un petit tronçon, la caméra le revoit, le trajet est recalculé.

Lancement (terminal de VS Code, dans le dossier robot_camera) :
    python cartographie.py coin      -> caméra dans le coin (vue inclinée)
    python cartographie.py dalle     -> caméra sous la dalle (vue du dessus)

Touches (cliquer d'abord sur une des fenêtres OpenCV pour qu'elle reçoive le clavier) :
    c     = calibrer (les 4 marqueurs de coin visibles) + enregistrer dans calib_<position>.json
    1 / 2 = charger calib_coin.json / calib_dalle.json
    r     = prendre la référence "zone vide" (à refaire à chaque séance, sans le robot si possible)
    g     = calculer le trajet et lancer le guidage du robot
    s     = stopper le guidage (et envoyer "stop" au robot)
    + / - = augmenter / diminuer la marge de sécurité (aussi p / m)
    q     = quitter
    clic gauche sur la carte = nouvelle destination
"""
import cv2, numpy as np, heapq, json, os, sys, time

import liaison_robot  # envoi des ordres au robot (mode simulation par défaut, voir ce fichier)

# =============================== PARAMÈTRES ===============================
CAMERA_SOURCE = 0              # 1 = C270 (USB) ; 0 = webcam intégrée du PC  <-- REMETTRE 1 quand la C270 est rebranchée
CAMERA_W, CAMERA_H = 1280, 720 # résolution demandée à la caméra

ROOM_W, ROOM_H = 76, 35  # cm, largeur x longueur ENTRE LES CENTRES des 4 marqueurs (à re-mesurer !)
SCALE = 3                      # pixels par cm dans la vue du dessus
CELL = 5                       # cm par case de la grille
# Position des marqueurs au sol, vus d'au-dessus, dans le sens des aiguilles d'une montre :
#   0 = haut-gauche (origine), 1 = haut-droite, 2 = bas-droite, 3 = bas-gauche.
CORNERS = {0: (0, 0), 1: (ROOM_W, 0), 2: (ROOM_W, ROOM_H), 3: (0, ROOM_H)}
ROBOT_ID = 10
DIFF_THRESH, MIN_AREA = 40, 150   # sensibilité de la différence d'images (seuil de gris, surface mini en pixels)
GOAL = (100, 130)              # cm, destination par défaut (modifiable au clic)

# --- Marge de sécurité autour des obstacles (réglable en direct avec + / -) ---
MARGE_SECURITE_CM = 25         # élargie par rapport à la 1re version (20) : la vue inclinée est moins précise
MARGE_MIN, MARGE_MAX, MARGE_PAS = 5, 60, 5

# --- Robot ---
ROBOT_HAUTEUR_CM = 37          # hauteur du Yanshee : sert à masquer toute sa silhouette (pas seulement ses pieds)
ROBOT_LARGEUR_CM = 30          # largeur du Yanshee + un peu de marge (largeur de la silhouette masquée)
MARQUEUR_HAUTEUR_CM = 0        # hauteur du marqueur 10 au-dessus du sol. MESUREZ-LA et indiquez-la :
                               # sinon la position du robot est décalée (effet de parallaxe). 0 = pas de correction.
ROBOT_CAP_DECALAGE_DEG = 0     # 0 si le HAUT du marqueur imprimé pointe vers l'avant du robot.
                               # Si la flèche bleue part sur le côté : 90 ou -90 ; à l'envers : 180.

# --- Les deux positions de la caméra ---
#   fichier          : où la calibration est enregistrée
#   masque_robot_cm  : rayon de la zone "robot" ignorée par la détection d'obstacles
#   camera_cm        : (x, y, hauteur) de l'objectif en cm dans le repère de la zone
#                      (origine = centre du marqueur 0, x vers le marqueur 1, y vers le marqueur 3).
#                      None = estimée automatiquement au calibrage (suffisant dans la plupart des cas).
POSITIONS = {
    "coin":  {"fichier": "calib_coin.json",  "masque_robot_cm": 25, "camera_cm": None},
    "dalle": {"fichier": "calib_dalle.json", "masque_robot_cm": 22, "camera_cm": None},
}
CAMERA_FOCALE_PX = 1400        # focale approximative de la C270 en 1280x720 (sert à estimer où est la caméra)

# --- Détection des marqueurs éloignés ---
CALIB_DUREE_S = 2.0            # le calibrage observe la scène 2 s et prend la position médiane de chaque marqueur
AGRANDIR_SI_ABSENT = 2.0       # robot non vu ? on réessaie sur l'image agrandie x2 (1 = désactivé, plus rapide)
ALERTE_DEPLACEMENT_PX = 15     # un marqueur de coin a bougé de plus de 15 px depuis le calibrage => alerte

# --- Carte et guidage ---
MAP_PX = 16                    # pixels par case sur la carte affichée
ARRIVEE_CM = 10                # le robot est "arrivé" à moins de 10 cm de la destination
PAUSE_APRES_MOUVEMENT_S = 1.0  # attente après chaque mouvement avant de revoir le robot (image stable)
ROBOT_PERDU_S = 5.0            # guidage arrêté si le robot n'est plus vu pendant 5 s
# ==========================================================================

aruco = cv2.aruco
_params = aruco.DetectorParameters()
_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX  # coins plus précis => homographie plus juste
_params.minMarkerPerimeterRate = 0.015       # accepte des marqueurs plus petits, donc plus lointains (défaut 0.03)
_params.adaptiveThreshWinSizeMax = 53        # seuillage adapté aux petits ET aux grands marqueurs (défaut 23)
_params.polygonalApproxAccuracyRate = 0.05   # tolère la déformation des marqueurs vus en biais (défaut 0.03)
detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_4X4_50), _params)


# ------------------------------------------------------------------ caméra et marqueurs
def grab(cap):
    """Lit l'image la plus récente (on jette les images en attente dans le tampon)."""
    for _ in range(3): cap.grab()
    ok, f = cap.read()
    return f if ok else None

def markers(frame, agrandir_si_absent=None):
    """Renvoie {id: 4 coins en pixels}. Si le marqueur `agrandir_si_absent` n'est pas trouvé,
    on refait une détection sur l'image agrandie (les marqueurs lointains y sont plus gros)."""
    corners, ids, _ = detector.detectMarkers(frame)
    m = {int(i): c[0] for c, i in zip(corners, ids.flatten())} if ids is not None else {}
    if agrandir_si_absent is not None and agrandir_si_absent not in m and AGRANDIR_SI_ABSENT > 1:
        grand = cv2.resize(frame, None, fx=AGRANDIR_SI_ABSENT, fy=AGRANDIR_SI_ABSENT)
        corners, ids, _ = detector.detectMarkers(grand)
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                m.setdefault(int(i), c[0] / AGRANDIR_SI_ABSENT)
    return m

def homography(m):
    """Homographie image -> vue du dessus. `m` = {id: coins} ou {id: centre}, pour les IDs 0..3."""
    if not all(k in m for k in CORNERS): return None
    src = np.float32([np.reshape(m[k], (-1, 2)).mean(axis=0) for k in CORNERS])
    dst = np.float32([[x * SCALE, y * SCALE] for x, y in CORNERS.values()])
    return cv2.getPerspectiveTransform(src, dst)

def topdown(frame, H):
    return cv2.warpPerspective(frame, H, (ROOM_W * SCALE, ROOM_H * SCALE))

def vers_cm(points_px, H):
    """Convertit des points de l'image brute (pixels) en positions au sol (cm)."""
    pts = np.float32(points_px).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2) / SCALE


# ------------------------------------------------------------------ calibration (enregistrée / rechargée)
def calibrer(cap):
    """Observe la scène CALIB_DUREE_S secondes et garde la position médiane de chaque marqueur
    de coin : plus stable qu'une seule image, et un marqueur lointain vu par moments suffit."""
    vus = {k: [] for k in CORNERS}
    t0 = time.time()
    while time.time() - t0 < CALIB_DUREE_S:
        f = grab(cap)
        if f is None: continue
        for k, c in markers(f).items():
            if k in vus: vus[k].append(c.mean(axis=0))
        cv2.waitKey(1)
    manquants = [k for k, v in vus.items() if len(v) < 3]
    if manquants: return None, manquants
    return {k: np.median(v, axis=0) for k, v in vus.items()}, []

def estimer_camera(centres, taille_image):
    """Estime où se trouve la caméra (x, y, hauteur en cm) à partir des 4 marqueurs.
    Sert à corriger la parallaxe du marqueur du robot et à masquer sa silhouette."""
    w, h = taille_image
    f = CAMERA_FOCALE_PX * w / 1280
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], np.float64)
    obj = np.float64([[x, y, 0] for x, y in CORNERS.values()])
    img = np.float64([centres[k] for k in CORNERS])
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE)
    if not ok: return None
    R, _ = cv2.Rodrigues(rvec)
    C = (-R.T @ tvec).ravel()
    return [round(float(C[0]), 1), round(float(C[1]), 1), round(abs(float(C[2])), 1)]

def sauver_calib(position, centres, taille_image):
    cam = estimer_camera(centres, taille_image)
    data = {
        "position": position,
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "taille_image": list(taille_image),
        "zone_cm": [ROOM_W, ROOM_H],
        "centres_marqueurs_px": {str(k): [float(v[0]), float(v[1])] for k, v in centres.items()},
        "camera_cm_estimee": cam,
        "homographie": homography(centres).tolist(),   # pour information (recalculée au chargement)
    }
    with open(POSITIONS[position]["fichier"], "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    return cam

def charger_calib(position, taille_image=None):
    """Recharge calib_<position>.json. L'homographie est recalculée avec les dimensions
    ROOM_W / ROOM_H actuelles : si vous re-mesurez la zone, inutile de recalibrer."""
    fichier = POSITIONS[position]["fichier"]
    if not os.path.exists(fichier):
        print(f"Pas de {fichier} : placez la camera puis appuyez sur c pour calibrer.")
        return None
    with open(fichier, encoding="utf-8") as fh:
        data = json.load(fh)
    centres = {int(k): np.array(v) for k, v in data["centres_marqueurs_px"].items()}
    if data.get("zone_cm") != [ROOM_W, ROOM_H]:
        print(f"Note : zone {data.get('zone_cm')} cm dans {fichier}, {ROOM_W}x{ROOM_H} cm maintenant (homographie recalculee).")
    if taille_image and list(taille_image) != data.get("taille_image"):
        print(f"ATTENTION : calibration faite en {data.get('taille_image')}, camera actuelle {list(taille_image)}. Recalibrez (c).")
    print(f"Calibration chargee : {fichier} (du {data.get('date')}), camera estimee a {data.get('camera_cm_estimee')} cm")
    return {"centres": centres, "camera_cm": data.get("camera_cm_estimee")}


# ------------------------------------------------------------------ robot
def corriger_parallaxe(pts_cm, hauteur_cm, cam):
    """Un point SITUÉ EN HAUTEUR (le marqueur sur le robot) est vu par l'homographie du sol
    plus loin de la caméra qu'il ne l'est vraiment. On le ramène vers le pied de la caméra."""
    if cam is None or hauteur_cm <= 0 or cam[2] <= hauteur_cm: return pts_cm
    pied = np.array(cam[:2], np.float32)
    return pied + (pts_cm - pied) * (cam[2] - hauteur_cm) / cam[2]

def robot_pose(m, H, cam=None):
    """Position (cm) et cap (degrés) du robot. Cap 0 = vers le marqueur 1, 90 = vers le marqueur 3."""
    if ROBOT_ID not in m: return None
    pts = corriger_parallaxe(vers_cm(m[ROBOT_ID], H), MARQUEUR_HAUTEUR_CM, cam)
    c = pts.mean(axis=0)
    v = (pts[0] + pts[1]) / 2 - (pts[3] + pts[2]) / 2     # du bas vers le haut du marqueur
    cap = (np.degrees(np.arctan2(v[1], v[0])) + ROBOT_CAP_DECALAGE_DEG + 180) % 360 - 180
    return c, float(cap)

def masquer_robot(mask, robot_xy, rayon_cm, cam):
    """Efface le robot du masque d'obstacles. En vue inclinée, le corps du robot (37 cm de haut)
    "s'étale" au sol à l'opposé de la caméra : on efface donc toute cette trace, pas juste un cercle."""
    p = (int(robot_xy[0] * SCALE), int(robot_xy[1] * SCALE))
    r = int(rayon_cm * SCALE)
    cv2.circle(mask, p, r, 0, -1)
    if cam is not None and cam[2] > ROBOT_HAUTEUR_CM:
        k = cam[2] / (cam[2] - ROBOT_HAUTEUR_CM)
        haut = (int((cam[0] + (robot_xy[0] - cam[0]) * k) * SCALE),
                int((cam[1] + (robot_xy[1] - cam[1]) * k) * SCALE))
        cv2.line(mask, p, haut, 0, int(ROBOT_LARGEUR_CM * SCALE))
        cv2.circle(mask, haut, int(ROBOT_LARGEUR_CM * SCALE / 2), 0, -1)


# ------------------------------------------------------------------ grille d'occupation
def build_grid(ref, cur, robot_xy, marge_cm=MARGE_SECURITE_CM, masque_robot_cm=20, cam=None, obstacles_en_plus=()):
    """Renvoie (obstacles, obstacles+marge) : deux grilles de booléens (True = case occupée).
    `obstacles_en_plus` = liste de cercles (x_cm, y_cm, rayon_cm), ex. les objets reconnus par YOLO."""
    d = cv2.absdiff(cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY), cv2.cvtColor(cur, cv2.COLOR_BGR2GRAY))
    d = cv2.GaussianBlur(d, (7, 7), 0)
    _, mask = cv2.threshold(d, DIFF_THRESH, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    if robot_xy is not None:
        masquer_robot(mask, robot_xy, masque_robot_cm, cam)
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] < MIN_AREA: mask[lab == i] = 0
    for x, y, r in obstacles_en_plus:
        cv2.circle(mask, (int(x * SCALE), int(y * SCALE)), int(r * SCALE), 255, -1)
    obst = cv2.resize(mask, (ROOM_W // CELL, ROOM_H // CELL), interpolation=cv2.INTER_AREA) > 40
    k = int(round(marge_cm / CELL))
    noyau = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))  # marge ronde
    marge = cv2.dilate(obst.astype(np.uint8), noyau).astype(bool)
    return obst, marge


# ------------------------------------------------------------------ trajet
def astar(grid, start, goal):
    """A* sur la grille, 8 directions (diagonales autorisées sans couper les coins d'obstacle)."""
    D2 = 2 ** 0.5
    def h(a):
        dx, dy = abs(a[0] - goal[0]), abs(a[1] - goal[1])
        return max(dx, dy) + (D2 - 1) * min(dx, dy)
    pq, came, cost = [(h(start), 0, start)], {start: None}, {start: 0}
    while pq:
        _, g, cur = heapq.heappop(pq)
        if cur == goal: break
        if g > cost[cur]: continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nb = (cur[0] + dx, cur[1] + dy)
            if 0 <= nb[0] < grid.shape[1] and 0 <= nb[1] < grid.shape[0] and not grid[nb[1], nb[0]]:
                if dx and dy and (grid[cur[1], nb[0]] or grid[nb[1], cur[0]]): continue
                ng = g + (D2 if dx and dy else 1)
                if nb not in cost or ng < cost[nb]:
                    cost[nb], came[nb] = ng, cur
                    heapq.heappush(pq, (ng + h(nb), ng, nb))
    if goal not in came: return None
    path, p = [], goal
    while p: path.append(p); p = came[p]
    return path[::-1]

def ligne_libre(grid, a, b):
    """True si la ligne droite entre les cases a et b ne traverse aucune case occupée."""
    n = 2 * max(abs(b[0] - a[0]), abs(b[1] - a[1])) + 1
    for t in np.linspace(0, 1, n):
        if grid[int(round(a[1] + (b[1] - a[1]) * t)), int(round(a[0] + (b[0] - a[0]) * t))]: return False
    return True

def waypoints(path, grid=None):
    """Réduit le chemin à quelques points de passage.
    Avec `grid` : on va en ligne droite tant que c'est possible (moins de virages pour le robot).
    Sans `grid` : on garde chaque changement de direction (méthode de la 1re version)."""
    if not path or len(path) < 3: return path
    if grid is None:
        out = [path[0]]
        for a, b, c in zip(path, path[1:], path[2:]):
            if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]): out.append(b)
        return out + [path[-1]]
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not ligne_libre(grid, path[i], path[j]): j -= 1
        out.append(path[j]); i = j
    return out

def calculer_trajet(obst, marge, pose, goal_cell, marge_cm):
    """Renvoie (chemin en cases, waypoints en cm, message)."""
    gh, gw = marge.shape
    s = (min(max(int(pose[0][0] // CELL), 0), gw - 1), min(max(int(pose[0][1] // CELL), 0), gh - 1))
    g = (min(max(goal_cell[0], 0), gw - 1), min(max(goal_cell[1], 0), gh - 1))
    if obst[g[1], g[0]]: return None, None, "Destination sur un obstacle"
    if marge[g[1], g[0]]: return None, None, "Destination trop pres d'un obstacle (marge)"
    # Si le robot a derivé dans la marge, on l'autorise à en sortir : autour de lui,
    # seules les vraies cases d'obstacle restent bloquées.
    nav = marge.copy()
    k = int(round(marge_cm / CELL))
    ys, xs = np.ogrid[:gh, :gw]
    nav[((xs - s[0]) ** 2 + (ys - s[1]) ** 2 <= k * k) & ~obst] = False
    nav[s[1], s[0]] = False
    path = astar(nav, s, g)
    if path is None: return None, None, "Aucun chemin trouve"
    wps = [(x * CELL + CELL / 2, y * CELL + CELL / 2) for x, y in waypoints(path, nav)]
    wps[0] = (round(float(pose[0][0]), 1), round(float(pose[0][1]), 1))   # départ = position réelle
    return path, wps, "OK"


# ------------------------------------------------------------------ affichage
def draw_map(grid, pose, path, goal_cell, obst=None, wps=None, infos=""):
    """Carte type Google Maps : obstacles rouges, marge rose, trajet vert, robot bleu, destination orange."""
    P = MAP_PX
    gh, gw = grid.shape
    base = np.full((gh, gw, 3), 245, np.uint8)
    base[grid] = (205, 205, 250)                                          # marge de sécurité (rose)
    base[obst if obst is not None else grid] = (60, 60, 230)               # obstacles rouges
    img = np.full((gh * P + 28, gw * P, 3), 255, np.uint8)
    img[:gh * P] = cv2.resize(base, (gw * P, gh * P), interpolation=cv2.INTER_NEAREST)
    for x in range(0, gw * P, P * 4): cv2.line(img, (x, 0), (x, gh * P), (225, 225, 225), 1)   # quadrillage 20 cm
    for y in range(0, gh * P, P * 4): cv2.line(img, (0, y), (gw * P, y), (225, 225, 225), 1)
    centre = lambda c: (int(c[0] * P + P / 2), int(c[1] * P + P / 2))
    if wps:   # trajet vert = lignes droites entre waypoints (ce que le robot va réellement faire)
        pts = [(int(x * P / CELL), int(y * P / CELL)) for x, y in wps]
        for a, b in zip(pts, pts[1:]): cv2.line(img, a, b, (0, 180, 0), 3)
        for p in pts: cv2.circle(img, p, 4, (0, 120, 0), -1)
    elif path:
        for a, b in zip(path, path[1:]):
            cv2.line(img, centre(a), centre(b), (0, 180, 0), 3)                                 # trajet vert
    if goal_cell:
        cv2.circle(img, centre(goal_cell), 8, (0, 140, 255), -1)                               # but orange
    if pose:
        c = (int(pose[0][0] * P / CELL), int(pose[0][1] * P / CELL))
        cv2.circle(img, c, 9, (230, 120, 0), -1)                                                # robot bleu
        a = np.radians(pose[1])
        cv2.arrowedLine(img, c, (int(c[0] + 28 * np.cos(a)), int(c[1] + 28 * np.sin(a))), (230, 120, 0), 3, tipLength=0.4)
    cv2.putText(img, infos, (5, gh * P + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 40, 40), 1)
    return img

def carte_vide(message):
    """Carte grise affichée tant que la calibration ou la référence manque."""
    vide = np.zeros((ROOM_H // CELL, ROOM_W // CELL), bool)
    img = draw_map(vide, None, None, None, infos=message)
    cv2.putText(img, message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 200), 2)
    return img


# ------------------------------------------------------------------ liaison robot
def send_to_robot(lien, pose, wps_cm):
    """Envoie au robot sa position actuelle + les waypoints (cm), sans bloquer l'affichage.
    Le robot fait UN petit tronçon puis répond ; la boucle principale le revoit et renvoie la suite."""
    message = {"type": "waypoints",
               "pose": {"x": round(float(pose[0][0]), 1), "y": round(float(pose[0][1]), 1), "cap": round(pose[1], 1)},
               "waypoints": [[round(x, 1), round(y, 1)] for x, y in wps_cm]}
    print("Waypoints (cm):", message["waypoints"])
    return lien.envoyer_async(message)


_dernier_message = {"texte": "", "t": 0.0}
def info(*morceaux):
    """Affiche un message dans le terminal ET en bas de la fenêtre camera (pendant 8 s)."""
    texte = " ".join(str(m) for m in morceaux)
    print(texte)
    _dernier_message["texte"], _dernier_message["t"] = texte, time.time()

def dessiner_bandeau(view, m, calibre):
    """Bandeau en bas de l'image camera : coins vus / manquants + dernier message."""
    h, w = view.shape[:2]
    vus = [k for k in CORNERS if k in m]
    manquants = [k for k in CORNERS if k not in m]
    cv2.rectangle(view, (0, h - 75), (w, h), (30, 30, 30), -1)
    ligne1 = f"Coins vus : {vus}   Robot (10) : {'oui' if ROBOT_ID in m else 'non'}   " \
             f"Calibration : {'OK' if calibre else 'a faire (c)'}"
    if manquants: ligne1 += f"   MANQUANTS : {manquants}"
    cv2.putText(view, ligne1, (10, h - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (80, 80, 255) if manquants and not calibre else (255, 255, 255), 2)
    if time.time() - _dernier_message["t"] < 8:
        cv2.putText(view, _dernier_message["texte"][:110], (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


goal_cm = list(GOAL)
def on_mouse(event, x, y, flags, param):
    global goal_cm
    if event == cv2.EVENT_LBUTTONDOWN:
        cx, cy = x // MAP_PX, y // MAP_PX
        if 0 <= cx < ROOM_W // CELL and 0 <= cy < ROOM_H // CELL:      # clic hors carte ignoré
            goal_cm = [cx * CELL + CELL // 2, cy * CELL + CELL // 2]
            print("Nouvelle destination (cm):", goal_cm)


# ------------------------------------------------------------------ programme principal
def main(position=None, identificateur=None):
    """`identificateur` (facultatif) = module YOLO (identification.py) qui ajoute les objets reconnus."""
    if position is None:
        position = sys.argv[1] if len(sys.argv) > 1 else "coin"
    if position not in POSITIONS:
        raise SystemExit(f"Position inconnue '{position}'. Utilisez : python cartographie.py coin  (ou dalle)")

    cap = cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_H)
    if not cap.isOpened():
        raise SystemExit("Camera introuvable : changez CAMERA_SOURCE (0, 1, 2).")
    taille = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    lien = liaison_robot.LiaisonRobot()
    marge_cm = MARGE_SECURITE_CM
    centres = H = cam = ref = None

    def appliquer(pos, calib):
        """Met en place une calibration : homographie + position de la caméra."""
        nonlocal centres, H, cam
        centres = calib["centres"] if calib else None
        H = homography(centres) if calib else None
        cam = POSITIONS[pos]["camera_cm"] or (calib or {}).get("camera_cm")

    appliquer(position, charger_calib(position, taille))
    guidage, t_mouvement, dernier_vu = False, 0.0, time.time()
    pose = wps_cm = None

    cv2.namedWindow("carte")
    cv2.setMouseCallback("carte", on_mouse)
    info(f"Position camera : {position}")
    info("c = calibrer | 1/2 = charger calib coin/dalle | r = reference (zone vide) | g = trajet + guidage | "
          "s = stop | +/- = marge | clic sur la carte = destination | q = quitter")

    echecs = 0
    while True:
        frame = grab(cap)
        if frame is None:
            echecs += 1
            if echecs == 50:   # environ 2 à 3 s sans image
                info("AUCUNE IMAGE recue de la camera. Fermez test_live.py et toute appli qui utilise "
                      "la webcam (Teams, Zoom, appli Camera), puis relancez. Sinon verifiez CAMERA_SOURCE.")
                cv2.imshow("carte", carte_vide("Pas d'image camera (voir terminal)"))
            if echecs > 300:
                raise SystemExit("Toujours aucune image de la camera : arret.")
            cv2.waitKey(10); continue
        echecs = 0
        m = markers(frame, ROBOT_ID if H is not None else None)
        view = frame.copy()
        for i, mk in m.items():
            cv2.polylines(view, [mk.astype(int)], True, (0, 255, 0), 2)
            cv2.putText(view, str(i), tuple(mk[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # La caméra a-t-elle bougé depuis le calibrage ? (comparaison avec les coins visibles)
        if centres is not None:
            ecarts = [np.linalg.norm(m[k].mean(axis=0) - centres[k]) for k in CORNERS if k in m]
            if ecarts and max(ecarts) > ALERTE_DEPLACEMENT_PX:
                cv2.putText(view, "CAMERA DEPLACEE ? recalibrez (c)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        if identificateur:
            identificateur.soumettre(frame)
            identificateur.dessiner_camera(view)
        dessiner_bandeau(view, m, H is not None)
        cv2.imshow("camera", view)

        pose = robot_pose(m, H, cam) if H is not None else None
        if pose: dernier_vu = time.time()
        gc = (goal_cm[0] // CELL, goal_cm[1] // CELL)
        infos = f"{position} | marge {marge_cm} cm | {'SIMULATION' if lien.simulation else 'ROBOT ' + lien.ip}" \
                f"{' | GUIDAGE' if guidage else ''}"

        if H is not None and ref is not None:
            td = topdown(frame, H)
            objets = identificateur.objets_cm(H, pose, position) if identificateur else []
            obst, marge = build_grid(ref, td, pose[0] if pose else None, marge_cm,
                                     POSITIONS[position]["masque_robot_cm"], cam,
                                     [(o["x"], o["y"], o["r"]) for o in objets])
            path, wps_cm, msg = calculer_trajet(obst, marge, pose, gc, marge_cm) if pose else (None, None, "Robot non vu")
            if msg != "OK": infos += " | " + msg
            carte = draw_map(marge, pose, path, gc, obst, wps_cm, infos)
            if identificateur: identificateur.dessiner_carte(carte, objets)
            cv2.imshow("carte", carte)
            # Vue redressée (contrôle visuel de la calibration) avec les obstacles teintés en rouge
            rouge = cv2.resize(obst.astype(np.uint8), (td.shape[1], td.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
            td[rouge] = (td[rouge] * 0.5 + np.array([0, 0, 255]) * 0.5).astype(np.uint8)
            cv2.imshow("vue du dessus", td)
        else:
            wps_cm = None
            cv2.imshow("carte", carte_vide("Appuyez sur c (calibrer)" if H is None else "Appuyez sur r (zone vide)"))
            if H is not None: cv2.imshow("vue du dessus", topdown(frame, H))

        # ---- Guidage en boucle fermée : un tronçon, on revoit le robot, on recalcule ----
        if guidage and not lien.occupe:
            rep = lien.recuperer_reponse()
            if rep is not None:
                t_mouvement = time.time()
                if not rep.get("ok"):
                    info("Erreur robot :", rep.get("erreur")); guidage = False
                elif rep.get("alerte"):
                    info("ARRET DE SECURITE du robot :", rep["alerte"]); guidage = False
                elif rep.get("arrive"):
                    info("Robot arrive a destination."); guidage = False
            if guidage and time.time() - t_mouvement > PAUSE_APRES_MOUVEMENT_S:
                if pose is None:
                    if time.time() - dernier_vu > ROBOT_PERDU_S:
                        info("Robot perdu de vue : guidage arrete."); guidage = False; lien.stop()
                elif np.hypot(pose[0][0] - goal_cm[0], pose[0][1] - goal_cm[1]) < ARRIVEE_CM:
                    info("Robot arrive a destination."); guidage = False
                elif wps_cm is None:
                    info("Plus de trajet possible : guidage arrete."); guidage = False; lien.stop()
                else:
                    send_to_robot(lien, pose, wps_cm)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): break
        if key == ord("c"):
            info(f"Calibrage ({CALIB_DUREE_S:.0f} s), ne bougez pas la camera...")
            res, manquants = calibrer(cap)
            if res is None:
                info(f"Marqueurs de coin pas assez vus : {manquants}. Verifiez qu'ils sont dans l'image.")
            else:
                c = sauver_calib(position, res, taille)
                appliquer(position, {"centres": res, "camera_cm": c}); ref = None
                info(f"Calibrage OK, enregistre dans {POSITIONS[position]['fichier']}. "
                      f"Camera estimee a (x, y, hauteur) = {c} cm. Appuyez maintenant sur r.")
        if key in (ord("1"), ord("2")):
            position = "coin" if key == ord("1") else "dalle"
            appliquer(position, charger_calib(position, taille)); ref = None; guidage = False
            info(f"Position camera : {position}. Appuyez sur r pour la reference.")
        if key == ord("r"):
            if H is None: info("r impossible : calibrez d'abord avec c (les 4 coins 0,1,2,3 doivent etre visibles)")
            else: ref = topdown(frame, H); info("Reference enregistree. Posez le robot, cliquez une destination, puis g.")
        if key == ord("g") and (H is None or ref is None):
            info("g impossible : faites d'abord c (calibrer) puis r (reference zone vide)")
        if key in (ord("+"), ord("="), ord("p")):
            marge_cm = min(MARGE_MAX, marge_cm + MARGE_PAS); info("Marge de securite :", marge_cm, "cm")
        if key in (ord("-"), ord("m")):
            marge_cm = max(MARGE_MIN, marge_cm - MARGE_PAS); info("Marge de securite :", marge_cm, "cm")
        if key == ord("s"):
            guidage = False; lien.stop(); info("Guidage arrete.")
        if key == ord("g") and H is not None and ref is not None:
            if pose is None: info("Robot (marqueur 10) non vu"); continue
            if wps_cm is None: info("Aucun chemin trouve (voir la barre en bas de la carte)"); continue
            if lien.occupe: info("Le robot termine encore un mouvement, reessayez dans un instant."); continue
            lien.recuperer_reponse()   # oublie une éventuelle vieille réponse (ex. après un stop)
            with open("map.json", "w", encoding="utf-8") as fh:
                json.dump({"grid": marge.tolist(), "waypoints": wps_cm, "robot": [float(v) for v in pose[0]],
                           "destination": goal_cm}, fh)
            info("Trajet calcule (enregistre dans map.json). Guidage lance.")
            guidage, t_mouvement = True, 0.0

    if guidage: lien.stop()
    cap.release(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
