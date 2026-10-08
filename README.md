# Workshop M1 — Yanshee : obstacles, gaz et température

Le robot Yanshee marche seul, évite les obstacles et surveille l'air :
s'il détecte du gaz ou une température trop élevée, il s'arrête, l'annonce
à voix haute et ne repart que quand l'alerte est terminée. Tout s'affiche en
direct dans une page web.

```
Arduino Uno (tête)  --USB série-->  capteurs_serveur.py  --HTTP :8080-->  interface web (navigateur)
 ultrason, MQ-2,                    (Raspberry Pi du robot)    \
 DHT11, vapeur, lumière                                         -->  marche_obstacle.py (fait marcher le robot)
```

## Fichiers

| Fichier | Rôle |
|---|---|
| `arduino/capteurs_dht11/capteurs_dht11.ino` | Programme de l'Arduino : envoie `c=28 g=-1 d=-1 gaz=64 vapeur=26 lum=50 temp=24 hum=55` à 115200 bauds |
| `arduino/sauvegarde_origine.hex` | Sauvegarde du programme Arduino d'origine |
| `capteurs_serveur.py` | Lit l'Arduino, calcule les alertes gaz / température, sert l'API et l'interface |
| `marche_obstacle.py` | Fait marcher le robot (obstacles + réaction aux alertes) |
| `interface.html` | Tableau de bord (servi par `capteurs_serveur.py`) |
| `cartographie.py` | Webcam au-dessus de la zone : carte (robot, obstacles) à partir des marqueurs ArUco |
| `ultrason_serveur.py` | Ancien serveur (ultrason seul), remplacé par `capteurs_serveur.py` |
| `tests/` | Tests sans robot : `python -m pytest tests -v` |

## Câblage (Arduino Uno)

| Capteur | Broche |
|---|---|
| HC-SR04 (torse) | Trig D9, Echo D10 |
| MQ-2 gaz | A0 |
| Thermistance NTC 10k (optionnelle) | A1 |
| Vapeur | A2 |
| Lumière (LDR) | A3 |
| DHT11 | DATA sur D2 |

Bibliothèque Arduino : « DHT sensor library » d'Adafruit.

## Lancer sur le robot

```sh
scp capteurs_serveur.py marche_obstacle.py interface.html pi@IP_ROBOT:~
ssh pi@IP_ROBOT
pkill -f ultrason_serveur.py ; pkill -f capteurs_serveur.py
nohup python3 capteurs_serveur.py > capteurs.log 2>&1 &
python3 marche_obstacle.py            # Ctrl + C pour arrêter
```

Puis ouvrir `http://IP_ROBOT:8080/` dans un navigateur sur le même Wi-Fi.

Options de `marche_obstacle.py` : une vitesse (`slow`, `fast`…), `--arret`
(s'arrête devant l'obstacle sans tourner), `--test` (décide sans bouger le robot).

## Démo en une commande (PC Windows)

```powershell
powershell -ExecutionPolicy Bypass -File demo.ps1           # le robot marche (lent, 5 min max)
powershell -ExecutionPolicy Bypass -File demo.ps1 -Test     # répétition sans bouger le robot
```

Le script vérifie le robot, lance le serveur, ouvre l'interface, attend la
calibration du gaz puis fait marcher le robot. Options : `-Vitesse normal`,
`-Duree 600`, `-Robot 10.124.7.2`. Prérequis : `YanAPI.py` (voir ci-dessous).

## Tester depuis le PC (sans rien changer sur le robot)

Le serveur déjà lancé sur le robot sert les mesures brutes ; le PC fait tourner
la nouvelle version (alertes, interface) et pilote le robot par le Wi-Fi.
PC et robot doivent être sur le même Wi-Fi.

1. Une seule fois : récupérer `YanAPI.py` depuis le robot (lecture seule,
   demande le mot de passe de `pi`) :

   ```powershell
   ssh pi@10.124.7.2 "python3 -c 'import YanAPI;print(YanAPI.__file__)'"
   # affiche le chemin, par exemple /usr/local/lib/python3.5/dist-packages/YanAPI.py
   scp pi@10.124.7.2:/usr/local/lib/python3.5/dist-packages/YanAPI.py .
   ```

   (mettre le chemin affiché ; sur notre robot : `/usr/lib/python3.5/YanAPI.py`).
   `YanAPI.py` est ignoré par git. Puis installer ses dépendances :

   ```powershell
   python -m pip install requests nest_asyncio
   ```

   (`lib_ukit`, qui n'existe que sur le robot, est remplacé automatiquement.)

2. Terminal 1 : serveur en mode distant

   ```powershell
   $env:SOURCE_HTTP="http://10.124.7.2:8080"; $env:ROBOT_IP="10.124.7.2"; python capteurs_serveur.py
   ```

   Interface : <http://localhost:8080/>. Attendre ~80 s la calibration du gaz.

3. Terminal 2 : d'abord sans bouger, puis pour de vrai (vitesse lente conseillée)

   ```powershell
   python marche_obstacle.py --test
   $env:ROBOT_IP="10.124.7.2"; python marche_obstacle.py slow
   ```

Par le Wi-Fi, une mesure met ~0,2 s (parfois 1 s) : le robot réagit un peu plus
tard qu'en local. Rester à côté, Ctrl + C arrête le robot.

## Alertes

- **Gaz (MQ-2)** : le capteur chauffe 60 s, puis mesure l'air normal pendant
  20 s (`gaz_ref`). Alerte à `gaz_ref + 100`, danger à `gaz_ref + 250`.
  Lancer le serveur dans un air propre pour que la référence soit juste.
- **Température (DHT11)** : alerte à 40 °C, danger à 48 °C.

Les seuils sont en haut de `capteurs_serveur.py`.

## Simulation (fuite de gaz, surchauffe)

Pour tester la réaction du robot sans gaz ni chaleur : boutons **Fuite de gaz**
et **Surchauffe** de l'interface, ou

```powershell
curl.exe -X POST "http://localhost:8080/simuler?type=gaz"           # gaz : jusqu'au niveau danger
curl.exe -X POST "http://localhost:8080/simuler?type=temperature"   # 24 -> 50 °C
curl.exe -X POST "http://localhost:8080/simuler?type=stop"
```

La valeur monte, reste au maximum, puis redescend (options : `pic`, `montee`,
`palier`, `descente` en secondes, ex. `?type=gaz&pic=300&palier=30`). Elle
remplace la vraie mesure et passe par la même détection : le robot s'arrête et
parle comme pour une vraie alerte. L'interface et les messages indiquent
« SIMULATION ». La surchauffe marche même sans DHT11 ; la fuite de gaz attend
la fin de la calibration du MQ-2.

## Carte du sol (webcam + marqueurs ArUco)

Une webcam fixée au-dessus de la zone (branchée sur le PC) filme le sol.
`cartographie.py` en tire une carte : position et orientation du robot, et
obstacles posés dans la zone. Avec `--carte`, le robot reste dans le rectangle
et contourne les obstacles de la carte (en plus de l'ultrason).

```
webcam (USB, PC) --> cartographie.py :8081 --> capteurs_serveur.py :8080/carte --> marche_obstacle.py --carte
                                                                           \--> interface (carte du sol)
```

**Mise en place**

1. Marqueurs ArUco **4x4_50** (feuille imprimée) : posés à plat aux 4 coins de la zone,
   dans le sens des aiguilles d'une montre vu d'en haut :

   ```
   0 ---- x ----> 1
   |              |      x : de 0 vers 1, y : de 0 vers 3
   y     zone     |      cap 0° = vers 1, cap 90° = vers 3
   v              |
   3 ------------ 2
   ```

2. Mesurer la zone **entre les centres** des marqueurs (largeur 0→1, hauteur 1→2).
3. Marqueur **ID 10** à plat sur la tête du robot, le haut du marqueur vers l'avant du robot.
4. Webcam au-dessus de la zone, la plus verticale possible, les 4 coins visibles.
   Marqueurs d'au moins 10 cm si la camera est haute.

**Lancer** (sur le PC de la webcam)

```powershell
python -m pip install opencv-python
$env:ZONE="200x150"; python cartographie.py
```

Deux fenêtres s'ouvrent : la caméra (marqueurs vus, contour de la zone) et la vue de
dessus. **Zone dégagée et robot dehors**, appuyer sur **R** (ou bouton *Photo du sol
vide* de l'interface) : tout ce qui apparaît ensuite dans la zone devient un obstacle.
La photo est gardée dans `sol_reference.png` (à refaire si la caméra ou la lumière change).

Puis le serveur et la marche comme d'habitude, avec `--carte` :

```powershell
python marche_obstacle.py slow --carte
powershell -ExecutionPolicy Bypass -File demo.ps1 -Carte -Zone 200x150    # ou tout en un
```

Si `capteurs_serveur.py` tourne **sur le robot**, lui donner l'adresse du PC :
`CAMERA_HTTP=http://IP_DU_PC:8081 python3 capteurs_serveur.py`.

**Comportement avec `--carte`** : 30 cm avant de passer à moins de 25 cm d'un bord, le
robot s'arrête, dit « Bord de la zone » et tourne vers le centre jusqu'à avoir la voie
libre. Même chose devant un obstacle de la carte (il tourne du côté opposé). Si la caméra
ne le voit plus pendant 1,5 s, il s'arrête et attend. Réglages en haut de
`marche_obstacle.py` (`MARGE_BORD_CM`, `ANTICIPATION_CM`…).

**Réglages** de `cartographie.py` (variables d'environnement) : `CAMERA=1` (autre
webcam), `CAMERA=photo.jpg` (test sans webcam), `RESOLUTION=1280x720`,
`CAP_DECALAGE_DEG` (marqueur collé de travers : robot face au marqueur 1, lire le cap,
mettre son opposé), `HAUTEUR_CAMERA_CM=250` (corrige le décalage dû à la hauteur de
la tête, 37 cm), `--sans-fenetre`.

## API (`capteurs_serveur.py`, port 8080)

| Route | Réponse |
|---|---|
| `GET /` | interface web |
| `GET /capteurs` | mesures brutes + `alertes`, `gaz_etat`, `gaz_ref`, `seuils`, `age`, `perime` |
| `GET /distance` | distances en mètres (`null` = capteur non câblé) |
| `GET /batterie` | batterie du robot |
| `GET /marche` | état publié par `marche_obstacle.py` |
| `POST /simuler?type=gaz\|temperature\|stop` | lance / arrête une simulation |
| `GET /carte` | carte de la webcam : `zone`, `robot` (`x`, `y` en cm, `cap`, `age`), `obstacles`, `grille` |
| `GET /carte.jpg` | vue de dessus annotée |
| `POST /reference` | photo du sol vide (robot hors de la zone) |
