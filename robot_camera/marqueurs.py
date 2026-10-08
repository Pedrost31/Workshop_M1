import cv2, numpy as np

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
page = np.full((3508, 2480), 255, np.uint8)   # feuille A4 blanche
S = 1063                                       # taille d'un marqueur (environ 9 cm)

for n, i in enumerate([0, 1, 2, 3, 10]):
    m = cv2.aruco.generateImageMarker(d, i, S)
    r, c = divmod(n, 2)
    x, y = 100 + c * (S + 150), 100 + r * (S + 100)
    page[y:y + S, x:x + S] = m
    cv2.putText(page, f"ID {i}", (x, y + S + 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)

cv2.imwrite("marqueurs.png", page)
print("Fichier marqueurs.png créé")