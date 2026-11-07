"""
tester_yolo.py : teste la détection de voiture YOLOv8n sur les images collectées (vision V4).

Pour chaque image (dataset/sim/C5.png, ...) : la voiture principale détectée
(classe, confiance, taille du rectangle) et le temps de calcul.
Les images annotées (rectangles dessinés) vont dans dataset/yolo/.

Utilisation :
    ros2 run robot_parking tester_yolo                    (yolov8n, confiance 0.30)
    ros2 run robot_parking tester_yolo yolov8n 0.15       (seuil plus bas)
    ros2 run robot_parking tester_yolo yolov8s 0.30       (modèle plus gros)
"""
import csv
import glob
import os
import sys
import time

import cv2

from robot_parking.vision_yolo import detecter_voiture, charger_modele

IMAGES = os.path.expanduser('~/parking_ws/dataset/sim')
SORTIE = os.path.expanduser('~/parking_ws/dataset/yolo')
VERITE = os.path.expanduser('~/parking_ws/src/robot_parking/worlds/voitures.csv')


def main():
    os.makedirs(SORTIE, exist_ok=True)
    with open(VERITE) as f:
        verite = {l['place']: f"{l['couleur']} {l['type']}" for l in csv.DictReader(f)}

    nom = sys.argv[1] if len(sys.argv) > 1 else 'yolov8n'
    confiance = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30
    print(f'Modèle {nom}, seuil de confiance {confiance} (téléchargement la 1re fois)...')
    charger_modele(nom)

    detectees, total = 0, 0
    print(f"{'place':<6}{'vraie voiture':<18}{'détection':<10}{'confiance':>10}{'largeur':>9}{'temps':>9}")
    for chemin in sorted(glob.glob(os.path.join(IMAGES, '*.png'))):
        place = os.path.splitext(os.path.basename(chemin))[0]
        image = cv2.imread(chemin)
        t0 = time.perf_counter()
        det, annotee = detecter_voiture(image, nom=nom, confiance=confiance)
        duree = (time.perf_counter() - t0) * 1000
        cv2.imwrite(os.path.join(SORTIE, f'{place}.png'), annotee)

        total += 1
        if det:
            detectees += 1
            x1, _, x2, _ = det['boite']
            print(f"{place:<6}{verite.get(place, '?'):<18}{det['classe']:<10}"
                  f"{det['confiance']:>10.2f}{x2 - x1:>7} px{duree:>7.0f} ms  ✅")
        else:
            print(f"{place:<6}{verite.get(place, '?'):<18}{'rien':<10}{'':>10}{'':>9}{duree:>7.0f} ms  ❌")

    print(f'\nVoiture détectée sur {detectees}/{total} images')
    print(f'Images annotées : {SORTIE}')


if __name__ == '__main__':
    main()
