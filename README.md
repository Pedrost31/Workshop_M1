# Mission Sentinel-X — vision par caméra fixe pour le robot Yanshee

Système de vision du workshop EPSI : une webcam Logitech C270 cartographie la zone,
identifie les obstacles et guide le robot Yanshee. Un module séparé contrôle l'accès
à la data room par reconnaissance faciale.

## 1. Les fichiers

| Fichier | Où il tourne | Rôle |
|---|---|---|
| `test_camera.py` | PC | Liste les caméras et teste chaque numéro (index). |
| `test_live.py` | PC | Affiche l'image en direct de la C270 (caméra 1). |
| `marqueurs.py` / `marqueurs.png` | PC | Planche des marqueurs ArUco à imprimer (0, 1, 2, 3 = coins ; 10 = robot). |
| **`cartographie.py`** | PC | **Module 1** : calibration, carte des obstacles, trajet A*, guidage du robot. |
| **`identification.py`** | PC | **Module 2** : comme le Module 1, avec en plus YOLO, qui nomme les obstacles (personne, chaise, obstacle). |
| **`controle_acces.py`** | PC | **Module 3** : reconnaissance faciale à l'entrée. Totalement indépendant des autres. |
| `liaison_robot.py` | PC | Envoie les ordres au robot par Wi-Fi. Mode simulation par défaut. |
| `robot_serveur.py` | **Robot** (Raspberry Pi) | Reçoit les ordres et fait marcher le Yanshee. |
| `requirements.txt` | — | Liste des bibliothèques Python à installer. |
| `calib_coin.json`, `calib_dalle.json` | — | Créés par la touche `c` : calibration de chaque position de caméra. |
| `map.json` | — | Créé par la touche `g` : dernière grille et derniers waypoints calculés. |
| `autorises/` | — | Créé au 1er lancement du Module 3 : photos des personnes autorisées. |

Les paramètres de chaque fichier sont regroupés **en haut du fichier**, dans le bloc `PARAMÈTRES`.

## 2. Installation (une seule fois, sur le PC)

Dans VS Code, ouvrez un terminal (menu *Terminal > Nouveau terminal*) dans le dossier `robot_camera`, puis :

```powershell
python -m pip install -r requirements.txt
```

Cette commande installe YOLO et PyTorch (environ 200 Mo, version CPU). YOLO installe aussi une
deuxième version d'OpenCV qui peut entrer en conflit avec celle déjà présente. Vérifiez :

```powershell
python -c "import cv2; print(cv2.__version__); cv2.aruco.ArucoDetector; print('OK')"
```

Si la commande affiche une erreur, réparez OpenCV :

```powershell
python -m pip uninstall -y opencv-python opencv-contrib-python
python -m pip install opencv-contrib-python
```

Le Module 3 a une installation à part : voir la [section 6](#6-module-3--contrôle-daccès-reconnaissance-faciale).

## 3. Mise en place du matériel

### Les marqueurs au sol

- Imprimez `marqueurs.png` **à 100 %**, sans « ajuster à la page ». Chaque marqueur doit faire environ 9 cm.
- Posez les marqueurs 0, 1, 2 et 3 à plat, **dans le sens des aiguilles d'une montre vu d'au-dessus** :
  0 en haut à gauche, 1 en haut à droite, 2 en bas à droite, 3 en bas à gauche.
  Le sens dans lequel chaque marqueur de coin est tourné n'a pas d'importance.
- Mesurez la distance **entre les centres** des marqueurs : de 0 à 1 (largeur) et de 0 à 3 (longueur).
  Reportez ces valeurs dans `ROOM_W, ROOM_H` en haut de `cartographie.py`.
- Sur le robot, collez le marqueur 10 **à plat, sur le dessus** (la tête), le haut du marqueur vers l'avant du robot.
  Mesurez sa hauteur par rapport au sol et indiquez-la dans `MARQUEUR_HAUTEUR_CM`.
  Sans cette valeur, la position du robot est décalée de 10 à 20 cm.

### Où placer la caméra

La C270 a un champ de vision assez étroit (environ 49° × 29°). Toujours vérifier avec
`python test_live.py` que **les 4 marqueurs de coin sont bien dans l'image**.

- **Position A, « coin »** (vue en biais) : à 1,40–1,50 m de haut, un peu en retrait derrière le coin
  du marqueur 0 (environ 60 à 90 cm en arrière), orientée vers le milieu de la zone, légèrement vers le bas.
- **Position B, « dalle »** (vue du dessus) : la caméra voit une zone d'environ **0,9 × 0,5 fois sa hauteur**.
  À 1,45 m, elle couvre seulement environ 1,30 m × 0,72 m : **une zone de 120 × 150 cm ne rentre pas**.
  Il faut alors soit fixer la caméra vers 2,6 m de haut, avec le grand côté de la zone dans la largeur de l'image,
  soit utiliser une zone plus petite pour cette position (par exemple 110 × 60 cm) en adaptant `ROOM_W, ROOM_H`.

## 4. Module 1 — Cartographie (étape prioritaire)

### Lancer

```powershell
python test_camera.py                 # repérer le numéro de la C270 (normalement 1)
python cartographie.py coin           # caméra dans le coin
python cartographie.py dalle          # caméra sous la dalle
```

Trois fenêtres s'ouvrent :
- **camera** : l'image brute, avec les marqueurs repérés en vert ;
- **vue du dessus** : l'image redressée, avec les obstacles teintés de rouge ;
- **carte** : la carte de navigation.

Cliquez sur une de ces fenêtres avant d'utiliser le clavier.

| Touche | Action |
|---|---|
| `c` | **Calibrer** pendant 2 s : la caméra ne doit pas bouger et les 4 coins doivent être visibles. La calibration est enregistrée dans `calib_coin.json` ou `calib_dalle.json`. |
| `1` / `2` | Recharger la calibration **coin** / **dalle**. Elle est aussi rechargée automatiquement au lancement. |
| `r` | **Référence** : photographie la zone vide, de préférence sans le robot. À refaire à chaque séance, car la lumière change. |
| `g` | Calculer le trajet et **lancer le guidage**. |
| `s` | Stopper le guidage. |
| `+` / `-` (ou `p` / `m`) | Agrandir ou réduire la **marge de sécurité** autour des obstacles (25 cm au départ). |
| clic gauche sur la carte | Choisir une nouvelle destination. |
| `q` | Quitter. |

Sur la carte :
- **rouge** = obstacle ;
- **rose** = marge de sécurité ;
- **point bleu + flèche** = robot et son orientation ;
- **vert** = trajet ;
- **orange** = destination.

Si la caméra a bougé depuis le calibrage, « CAMERA DEPLACEE ? » s'affiche : appuyez sur `c`.

### Déroulé d'une séance

1. Fixez la caméra, puis lancez `python cartographie.py coin`. Il n'y a rien à faire si `calib_coin.json` existe et que la caméra n'a pas bougé. Sinon, appuyez sur `c`.
2. Videz la zone et appuyez sur `r`.
3. Posez le robot et les obstacles, cliquez la destination, appuyez sur `g`.
4. Pour passer à l'autre position : déplacez la caméra, appuyez sur `2` (ou relancez avec `dalle`), puis sur `r`.

### Tester sur le PC seul, sans robot

- La liaison robot est en **mode simulation** (`SIMULATION = True` dans `liaison_robot.py`).
  Après `g`, le terminal affiche les ordres que le robot recevrait, par exemple
  `[SIMULATION] ordre robot : tourner -60 deg, avancer 30 cm`.
- Posez les 4 marqueurs sur une table ou au sol.
- Le **marqueur 10 imprimé et déplacé à la main** joue le rôle du robot : la flèche bleue et les ordres doivent suivre.
- Des objets (boîte, chaise) servent d'obstacles : ils deviennent rouges, et le trajet vert les contourne.
- Pour tester la « vue en biais », filmez la table depuis un coin ; pour la « vue du dessus », filmez-la à la verticale.
  Ce sont deux fichiers de calibration différents.

### Réglages utiles (en haut de `cartographie.py`)

| Problème | Réglage |
|---|---|
| La flèche bleue part sur le côté ou à l'envers | `ROBOT_CAP_DECALAGE_DEG` = 90, -90 ou 180 |
| Des ombres ou des reflets apparaissent comme obstacles | Augmenter `DIFF_THRESH` (40 → 55), ou refaire `r` |
| Un objet clair n'est pas détecté | Baisser `DIFF_THRESH` (40 → 30) |
| Des « obstacles » apparaissent autour ou derrière le robot | Augmenter `masque_robot_cm` (dans `POSITIONS`) ou `ROBOT_LARGEUR_CM` |
| Un marqueur lointain n'est pas détecté | Mieux l'éclairer, l'imprimer plus grand, ou rapprocher la caméra |

## 5. Module 2 — Identification des obstacles (YOLO)

```powershell
python identification.py --test       # 1) vérifier YOLO seul (cadres sur l'image de la caméra)
python identification.py coin         # 2) cartographie + YOLO (mêmes touches que le Module 1)
```

- La 1re fois, le modèle `yolov8n.pt` (environ 6 Mo) se télécharge : il faut Internet.
- YOLO analyse l'**image brute**, car il reconnaît mal les objets vus du dessus. Comptez 2 à 5 analyses par seconde sur CPU ; la vitesse s'affiche en bas de la fenêtre camera.
- Sur la carte, chaque objet reconnu apparaît comme un cercle avec un libellé :
  - **personne** en violet ;
  - **chaise** en jaune ;
  - **obstacle** en gris pour tout autre objet reconnu par YOLO.
- La détection par différence d'images reste active. Un objet que YOLO ne connaît pas reste donc rouge sur la carte.
- Si le PC rame, mettez `TAILLE_YOLO = 416` ou `PAUSE_ENTRE_ANALYSES_S = 0.2` en haut de `identification.py`.

**Test sur le PC seul** : passez devant la caméra, ou posez une chaise dans la zone. Un cadre doit apparaître sur l'image et un cercle sur la carte.

## 6. Module 3 — Contrôle d'accès (reconnaissance faciale)

### Installation

La bibliothèque `face_recognition` dépend de `dlib`, difficile à compiler sous Windows.
On utilise donc une version de `dlib` déjà compilée :

```powershell
python -m pip install dlib-bin
python -m pip install face_recognition_models Pillow Click
python -m pip install --no-deps face_recognition
python -c "import face_recognition; print('face_recognition OK')"
```

- **Erreur `No module named 'pkg_resources'`** : lancez `python -m pip install "setuptools<81"`.
- **`dlib-bin` refuse de s'installer** : votre version de Python (3.13) est peut-être trop récente pour cette bibliothèque.
  Installez Python 3.12 en plus, puis utilisez `py -3.12 -m pip ...` et `py -3.12 controle_acces.py`.

### Utilisation

1. Lancez `python controle_acces.py` une 1re fois : le dossier `autorises/` est créé.
2. Mettez-y une photo nette, de face, par personne **qui a donné son accord** (par exemple `hajar.jpg`, `thomas_1.jpg`, `thomas_2.jpg`).
3. Placez la caméra à l'entrée, à hauteur de visage (1,5 à 1,7 m), face aux personnes qui arrivent.
4. Relancez `python controle_acces.py`. Un cadre **vert** « Autorisé : Hajar » ou **rouge** « Inconnu / Alerte » (avec un bip) s'affiche. Touche `q` pour quitter.

Réglage : `TOLERANCE` en haut du fichier. Baissez-la (par exemple 0.45) s'il y a de fausses autorisations, augmentez-la (0.55) si une personne autorisée n'est pas reconnue.

**RGPD** : voir la note en tête de `controle_acces.py`. Un visage est une donnée biométrique sensible. Les règles :
- usage réservé à la démo ;
- consentement écrit des personnes ;
- aucune image ni empreinte enregistrée par le script ;
- supprimer le dossier `autorises/` à la fin du workshop.

## 7. Liaison avec le robot

### Principe : boucle fermée

1. Le PC envoie au robot sa position vue par la caméra, ainsi que les waypoints.
2. Le robot fait **un seul petit tronçon** : il tourne, puis avance de 30 cm au maximum.
3. Le robot répond. Le PC le revoit à la caméra, recalcule le trajet et renvoie la suite.

Ce fonctionnement compense la dérive d'un humanoïde qui marche.

En plus, le robot consulte son **capteur à ultrasons** avant et pendant la marche. Si quelque chose est trop proche, il s'arrête et la carte affiche « ARRET DE SECURITE ».

### Étape A — Test sans réseau

Le mode simulation est activé par défaut. Lancez :

```powershell
python liaison_robot.py
```

Les ordres calculés s'affichent dans le terminal. Rien n'est envoyé.

### Étape B — Test du réseau sur le PC seul (deux terminaux)

Terminal 1 (le « faux robot ») :

```powershell
python robot_serveur.py --simulation
```

Dans `liaison_robot.py`, mettez `ROBOT_IP = "127.0.0.1"` et `SIMULATION = False`.

Terminal 2 :

```powershell
python liaison_robot.py
```

Puis lancez `python cartographie.py coin`. Le terminal 1 affiche les ordres reçus par le réseau.

### Étape C — Avec le vrai robot (quand le câblage sera réparé)

1. Branchez le PC et le Yanshee sur **le même Wi-Fi**. Notez l'IP du robot : dans l'appli Yanshee, ou avec la commande `hostname -I` sur le robot.
2. Copiez `robot_serveur.py` sur le robot. Utilisez les identifiants fournis avec le robot :

   ```powershell
   scp robot_serveur.py <utilisateur>@<IP_DU_ROBOT>:~/
   ```

3. Sur le robot, en SSH ou depuis son terminal :

   ```bash
   python3 robot_serveur.py
   ```

   Si le message `YanAPI indisponible -> MODE SIMULATION` apparaît, YanAPI n'a pas été trouvée.
4. Sur le PC, dans `liaison_robot.py`, mettez `ROBOT_IP = "<IP_DU_ROBOT>"` et `SIMULATION = False`.
5. Vérifiez la liaison avec `python liaison_robot.py`. Le ping doit répondre `{'ok': True, ...}`.
6. **Adaptez la partie marquée « À ADAPTER au SDK Yanshee »** dans `robot_serveur.py` (classe `CommandeYanshee`) :
   - vérifiez les noms des mouvements YanAPI (`walk`, `turn around`), la fonction d'arrêt et le format de réponse du capteur à ultrasons ;
   - mesurez `CM_PAR_PAS` : faites 10 pas, mesurez la distance, divisez par 10 ;
   - mesurez `DEG_PAR_ROTATION` : l'angle tourné par un mouvement « turn around » ;
   - si le robot tourne du mauvais côté, mettez `SENS_ROTATION = -1`.

Le port utilisé est **5005**, en TCP. Les messages sont en JSON ; leur format est décrit en tête de `robot_serveur.py`.

## 8. Ordre de lancement (récapitulatif)

1. `python test_camera.py`, puis `python test_live.py` : la caméra fonctionne et les 4 coins sont visibles.
2. `python cartographie.py coin`, puis `c` et `r`, puis un clic sur la carte et `g`. C'est le Module 1, en simulation.
3. Même chose avec `dalle`.
4. `python identification.py coin` : Module 2.
5. `python controle_acces.py` : Module 3, avec la caméra à l'entrée.
6. Robot : `python3 robot_serveur.py` sur le Yanshee, puis `SIMULATION = False` sur le PC, puis `python cartographie.py coin` (ou `identification.py`).
