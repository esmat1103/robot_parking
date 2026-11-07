"""
lecture_plaque.py : lit une plaque tunisienne dans une image (vision V2, sans ROS).

    image caméra --> 1) trouver la plaque --> 2) la redresser --> 3) lire les chiffres
                                                                     |
                                                     "176 TUN 5530"  <--

Disposition d'une plaque (lue de droite à gauche) :
    +--------------------------------------------+
    |  5530        تونس         176             |
    |  numéro      (milieu)     série            |
    +--------------------------------------------+
On lit seulement la zone de gauche (numéro) et celle de droite (série).

Fonction principale :
    lire_plaque_image(image_bgr) -> (texte ou None, image_de_controle)
"""
import re

import cv2
import numpy as np
import pytesseract

LARG, HAUT = 600, 150            # taille de la plaque une fois redressée (pixels)

# Zones de la plaque redressée (en fraction de la largeur / hauteur)
ZONE_NUMERO = (0.03, 0.42)       # à gauche
ZONE_SERIE = (0.66, 0.97)        # à droite
ZONE_HAUTEUR = (0.15, 0.85)      # on enlève le cadre blanc en haut et en bas

CONFIG_OCR = '--psm 7 -c tessedit_char_whitelist=0123456789'   # 1 ligne, chiffres seulement


# ---------------------------------------------------------------- 1) trouver la plaque
def trouver_plaque(image):
    """Renvoie les 4 coins de la plaque (ou None).
    Plaque = zone SOMBRE, rectangulaire, environ 4 fois plus large que haute,
    contenant des chiffres BLANCS."""
    gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, sombre = cv2.threshold(gris, 60, 255, cv2.THRESH_BINARY_INV)   # sombre -> blanc
    contours, _ = cv2.findContours(sombre, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    aire_image = image.shape[0] * image.shape[1]
    meilleur, meilleur_score = None, 0.0
    for c in contours:
        (cx, cy), (w, h), angle = cv2.minAreaRect(c)
        if w < h:
            w, h = h, w
        if h < 5:
            continue
        aire = w * h
        rapport = w / h
        if aire < 0.002 * aire_image or not (2.5 < rapport < 6.5):
            continue                                     # trop petit ou mauvaise forme
        if cv2.contourArea(c) < 0.6 * aire:
            continue                                     # pas assez rectangulaire
        coins = cv2.boxPoints(cv2.minAreaRect(c))
        # la vraie plaque contient des chiffres blancs (5 à 40 % de pixels clairs)
        redressee = redresser(gris, coins)
        clairs = np.mean(redressee[int(0.15 * HAUT):int(0.85 * HAUT)] > 150)
        if not (0.05 < clairs < 0.40):
            continue
        score = aire * np.exp(-abs(rapport - 4.0))       # grande, et proche de 4:1
        if score > meilleur_score:
            meilleur, meilleur_score = coins, score
    return meilleur


# ---------------------------------------------------------------- 2) la redresser
def ordonner_coins(coins):
    """Range les 4 coins : haut-gauche, haut-droit, bas-droit, bas-gauche."""
    coins = np.array(coins, dtype=np.float32)
    somme = coins.sum(axis=1)
    diff = np.diff(coins, axis=1).ravel()
    return np.array([coins[np.argmin(somme)], coins[np.argmin(diff)],
                     coins[np.argmax(somme)], coins[np.argmax(diff)]], dtype=np.float32)


def redresser(image, coins):
    """Découpe la plaque et la met bien à plat en LARG x HAUT pixels."""
    source = ordonner_coins(coins)
    cible = np.array([[0, 0], [LARG - 1, 0], [LARG - 1, HAUT - 1], [0, HAUT - 1]], dtype=np.float32)
    matrice = cv2.getPerspectiveTransform(source, cible)
    return cv2.warpPerspective(image, matrice, (LARG, HAUT))


# ---------------------------------------------------------------- 3) lire les chiffres
def lire_zone(plaque_grise, zone_x):
    """Lit les chiffres d'une zone de la plaque redressée."""
    x1, x2 = int(zone_x[0] * LARG), int(zone_x[1] * LARG)
    y1, y2 = int(ZONE_HAUTEUR[0] * HAUT), int(ZONE_HAUTEUR[1] * HAUT)
    zone = plaque_grise[y1:y2, x1:x2]
    zone = cv2.resize(zone, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    # Tesseract préfère du texte NOIR sur fond BLANC : on inverse
    _, nb = cv2.threshold(zone, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    nb = cv2.copyMakeBorder(nb, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
    texte = pytesseract.image_to_string(nb, config=CONFIG_OCR)
    return re.sub(r'\D', '', texte), nb


def lire_plaque_image(image):
    """Image caméra (BGR) -> ("176 TUN 5530" ou None, image de contrôle)."""
    coins = trouver_plaque(image)
    if coins is None:
        return None, image
    gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    plaque = redresser(gris, coins)
    numero, img_num = lire_zone(plaque, ZONE_NUMERO)
    serie, img_serie = lire_zone(plaque, ZONE_SERIE)

    # image de contrôle : la plaque redressée + ce que Tesseract a vu
    h = 120
    controle = np.vstack([
        cv2.resize(plaque, (LARG, HAUT)),
        np.hstack([cv2.resize(img_num, (LARG // 2, h)), cv2.resize(img_serie, (LARG // 2, h))]),
    ])
    if not numero or not serie:
        return None, controle
    return f'{serie} TUN {numero}', controle
