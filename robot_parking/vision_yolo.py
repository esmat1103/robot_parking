"""
vision_yolo.py : détection de voiture avec YOLOv8 pré-entraîné (vision V4, sans ROS).

YOLOv8 a été entraîné sur le jeu d'images COCO (80 classes). On l'utilise TEL QUEL,
sans entraînement : il connaît déjà "car", "truck", "bus".
    yolov8n = "nano"  (6 Mo,  le plus rapide)
    yolov8s = "small" (22 Mo, plus précis, ~3x plus lent)

Fonction principale :
    detecter_voiture(image_bgr) -> (meilleure_detection ou None, image_annotée)
    une détection = {'classe': 'car', 'confiance': 0.87, 'boite': (x1, y1, x2, y2)}

Les modèles sont téléchargés automatiquement la 1ʳᵉ fois dans ~/parking_ws/modeles/
"""
import os

from ultralytics import YOLO

DOSSIER_MODELES = os.path.expanduser('~/parking_ws/modeles')
MODELE_DEFAUT = 'yolov8s'     # test sur 10 images : yolov8n 7/10, yolov8s 9/10 (confiance bien plus haute)
CONFIANCE_DEFAUT = 0.30                        # en dessous, on ignore la détection
CLASSES_VEHICULE = {'car', 'truck', 'bus'}     # classes COCO qui nous intéressent

_modeles = {}


def charger_modele(nom=MODELE_DEFAUT):
    """Charge un modèle YOLO une seule fois (le chargement prend ~1 s)."""
    if nom not in _modeles:
        os.makedirs(DOSSIER_MODELES, exist_ok=True)
        _modeles[nom] = YOLO(os.path.join(DOSSIER_MODELES, f'{nom}.pt'))   # téléchargé si absent
    return _modeles[nom]


def detecter_voiture(image, seulement_devant=True, nom=MODELE_DEFAUT, confiance=CONFIANCE_DEFAUT):
    """Cherche les véhicules dans l'image et renvoie le PLUS GRAND.
    seulement_devant=True : on ignore les voitures AU LOIN ou SUR LE CÔTÉ :
    le rectangle doit faire au moins 20 % de la largeur de l'image et son centre
    doit être dans la moitié centrale de l'image."""
    resultat = charger_modele(nom)(image, imgsz=640, conf=confiance, verbose=False)[0]
    largeur_image = image.shape[1]
    meilleure, plus_grande_aire = None, 0
    for boite in resultat.boxes:
        classe = resultat.names[int(boite.cls)]
        if classe not in CLASSES_VEHICULE:
            continue
        x1, y1, x2, y2 = (int(v) for v in boite.xyxy[0])
        if seulement_devant:
            centre = (x1 + x2) / 2 / largeur_image
            if (x2 - x1) < 0.20 * largeur_image or not (0.25 < centre < 0.75):
                continue                       # trop petit ou trop sur le côté : pas devant
        aire = (x2 - x1) * (y2 - y1)
        if aire > plus_grande_aire:
            plus_grande_aire = aire
            meilleure = {'classe': classe, 'confiance': float(boite.conf),
                         'boite': (x1, y1, x2, y2)}
    return meilleure, resultat.plot()          # plot() = image avec TOUS les rectangles dessinés
