"""
controle_acces.py — MODULE 3 : contrôle d'accès à la data room par reconnaissance faciale
========================================================================================

Script SÉPARÉ de la cartographie. La C270 est placée à l'entrée, à hauteur de visage
(1,5 à 1,7 m), face aux personnes qui arrivent.

Fonctionnement :
  - au démarrage, on lit les photos du dossier "autorises/" (une personne autorisée
    par photo) et on calcule l'"empreinte" de chaque visage, en mémoire seulement ;
  - pendant la démo, 1 image sur N est analysée (la reconnaissance est lente sur CPU) ;
  - chaque visage vu est comparé aux empreintes : cadre VERT "Autorisé : <nom>"
    ou cadre ROUGE "Inconnu / Alerte" (+ bip).

Préparer les photos :
    autorises/
        hajar.jpg              -> nom affiché : "Hajar"
        thomas_1.jpg           -> "Thomas" (plusieurs photos d'une même personne :
        thomas_2.jpg               suffixe _1, _2... ou un sous-dossier autorises/thomas/)
  Une photo nette, de face, bien éclairée, un seul visage par photo.

Lancement :
    python controle_acces.py
Touche : q = quitter.

---------------------------------------------------------------------------------------
NOTE RGPD — À LIRE AVANT TOUTE UTILISATION
  * Un visage utilisé pour identifier une personne est une DONNÉE BIOMÉTRIQUE, donc une
    donnée "sensible" (article 9 du RGPD) : son traitement est interdit par principe,
    sauf exceptions, dont le CONSENTEMENT EXPLICITE de la personne.
  * Usage limité à la DÉMONSTRATION du workshop EPSI. Pas de mise en production.
  * Consentement : chaque personne dont la photo est dans "autorises/" doit avoir donné
    son accord écrit, et peut le retirer (on supprime alors sa photo).
  * Les personnes filmées doivent être informées (affichette "zone filmée - démo").
  * Pas de conservation : ce script n'enregistre AUCUNE image, aucune empreinte de
    visage et aucun historique sur le disque. Tout est en mémoire et disparaît à la
    fermeture. Supprimer le dossier "autorises/" à la fin du workshop.
  * Ne pas faire tourner ce script sur un PC partagé sans protéger ce dossier.
---------------------------------------------------------------------------------------
"""
import os
import sys
import threading
import time

import cv2
import numpy as np

try:
    import face_recognition
except ImportError:
    raise SystemExit("face_recognition n'est pas installe : voir la section Module 3 du README.md")

# =============================== PARAMÈTRES ===============================
CAMERA_SOURCE = 1              # C270 (0 = webcam intégrée du PC)
CAMERA_W, CAMERA_H = 1280, 720
DOSSIER_AUTORISES = "autorises"
TOLERANCE = 0.5                # plus petit = plus strict (0.6 = valeur par défaut de la bibliothèque)
ANALYSER_1_IMAGE_SUR = 5       # on n'analyse qu'1 image sur 5 (CPU)
REDUCTION = 0.5                # l'image est réduite de moitié avant analyse (plus rapide)
MODELE_DETECTION = "hog"       # "hog" = rapide sur CPU ("cnn" = précis mais très lent sans carte graphique)
CONFIRMATIONS_ALERTE = 2       # alerte seulement si "inconnu" sur 2 analyses de suite (évite les fausses alertes)
BIP_ALERTE = True
EXTENSIONS = (".jpg", ".jpeg", ".png")
POLICE = r"C:\Windows\Fonts\arial.ttf"   # pour afficher les accents (é)
# ==========================================================================

VERT, ROUGE = (0, 190, 0), (0, 0, 230)


def nom_depuis_fichier(chemin):
    """'thomas_2.jpg' -> 'Thomas'"""
    base = os.path.splitext(os.path.basename(chemin))[0]
    morceaux = base.split("_")
    if len(morceaux) > 1 and morceaux[-1].isdigit():
        base = "_".join(morceaux[:-1])
    return base.replace("_", " ").title()


def charger_autorises(dossier):
    """Calcule l'empreinte de chaque visage de référence (en mémoire, rien n'est enregistré)."""
    if not os.path.isdir(dossier):
        os.makedirs(dossier)
        raise SystemExit(f"Dossier '{dossier}' cree : mettez-y les photos des personnes autorisees, puis relancez.")
    fichiers = []
    for racine, _, noms in os.walk(dossier):
        for n in noms:
            if n.lower().endswith(EXTENSIONS):
                chemin = os.path.join(racine, n)
                # photo dans un sous-dossier -> le nom du sous-dossier est le nom de la personne
                nom = os.path.basename(racine).title() if racine != dossier else nom_depuis_fichier(n)
                fichiers.append((nom, chemin))
    empreintes, noms = [], []
    for nom, chemin in sorted(fichiers):
        image = face_recognition.load_image_file(chemin)
        trouves = face_recognition.face_encodings(image)
        if len(trouves) != 1:
            print(f"  ! {chemin} : {len(trouves)} visage(s) trouve(s) -> photo ignoree (il en faut exactement 1)")
            continue
        empreintes.append(trouves[0]); noms.append(nom)
        print(f"  + {nom}  ({chemin})")
    if not empreintes:
        raise SystemExit(f"Aucun visage utilisable dans '{dossier}'.")
    print(f"{len(set(noms))} personne(s) autorisee(s), {len(noms)} photo(s).")
    return empreintes, noms


def analyser(frame, empreintes, noms):
    """Renvoie une liste de (haut, droite, bas, gauche, nom_ou_None, distance) en pixels de l'image complète."""
    petite = cv2.resize(frame, None, fx=REDUCTION, fy=REDUCTION)
    rgb = np.ascontiguousarray(petite[:, :, ::-1])          # OpenCV = BGR, face_recognition = RGB
    lieux = face_recognition.face_locations(rgb, model=MODELE_DETECTION)
    resultats = []
    for lieu, emp in zip(lieux, face_recognition.face_encodings(rgb, lieux)):
        dist = face_recognition.face_distance(empreintes, emp)
        i = int(np.argmin(dist))
        nom = noms[i] if dist[i] <= TOLERANCE else None
        h, d, b, g = (int(v / REDUCTION) for v in lieu)
        resultats.append((h, d, b, g, nom, float(dist[i])))
    return resultats


# --- Écriture de texte avec accents (OpenCV seul ne sait pas afficher "é") ---
try:
    from PIL import Image, ImageDraw, ImageFont
    _police = ImageFont.truetype(POLICE, 28)
    _police_grande = ImageFont.truetype(POLICE, 44)
except Exception:
    _police = None

def ecrire(img, textes):
    """textes = liste de (texte, (x, y), couleur BGR, grand?)."""
    if _police is None:
        for t, (x, y), c, grand in textes:
            t = t.replace("é", "e").replace("É", "E")
            cv2.putText(img, t, (x, y + 25), cv2.FONT_HERSHEY_SIMPLEX, 1.2 if grand else 0.8, c, 2)
        return img
    pil = Image.fromarray(img[:, :, ::-1])
    dessin = ImageDraw.Draw(pil)
    for t, (x, y), c, grand in textes:
        dessin.text((x, y), t, font=_police_grande if grand else _police, fill=(c[2], c[1], c[0]),
                    stroke_width=2, stroke_fill=(255, 255, 255))
    return np.ascontiguousarray(np.array(pil)[:, :, ::-1])


def bip():
    if not BIP_ALERTE: return
    try:
        import winsound
        threading.Thread(target=winsound.Beep, args=(1200, 400), daemon=True).start()
    except Exception:
        pass


def main():
    print("Chargement des personnes autorisees...")
    empreintes, noms = charger_autorises(DOSSIER_AUTORISES)

    cap = cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_H)
    if not cap.isOpened():
        raise SystemExit("Camera introuvable : changez CAMERA_SOURCE (0, 1, 2).")

    resultats, n_image, inconnus_de_suite, dernier_bip = [], 0, 0, 0.0
    t_analyse, ips = time.time(), 0.0
    print("Controle d'acces en cours. q = quitter")
    while True:
        ok, frame = cap.read()
        if not ok:
            cv2.waitKey(10); continue
        n_image += 1
        if n_image % ANALYSER_1_IMAGE_SUR == 0:
            resultats = analyser(frame, empreintes, noms)
            maintenant = time.time()
            ips = 0.7 * ips + 0.3 / max(maintenant - t_analyse, 1e-3); t_analyse = maintenant
            inconnus_de_suite = inconnus_de_suite + 1 if any(r[4] is None for r in resultats) else 0
            if inconnus_de_suite >= CONFIRMATIONS_ALERTE and maintenant - dernier_bip > 3:
                bip(); dernier_bip = maintenant

        textes = []
        for h, d, b, g, nom, dist in resultats:
            couleur = VERT if nom else ROUGE
            cv2.rectangle(frame, (g, h), (d, b), couleur, 3)
            etiquette = f"Autorisé : {nom}" if nom else "Inconnu / Alerte"
            # "écart" = distance entre les deux visages (0 = identiques ; au-dessus de TOLERANCE = inconnu)
            textes.append((f"{etiquette}  (écart {dist:.2f})", (g, max(b + 5, 0)), couleur, False))
        if inconnus_de_suite >= CONFIRMATIONS_ALERTE:
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 70), ROUGE, -1)
            textes.append(("ALERTE : personne non autorisée", (20, 10), (255, 255, 255), True))
        elif resultats and all(r[4] for r in resultats):
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 70), VERT, -1)
            textes.append(("Accès autorisé", (20, 10), (255, 255, 255), True))
        cv2.putText(frame, f"{ips:.1f} analyses/s", (10, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("controle d'acces", ecrire(frame, textes))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
