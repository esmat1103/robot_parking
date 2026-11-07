"""
tester_lecture.py : teste la lecture de plaque sur les images collectées (vision V2).

Compare ce qui est lu sur chaque image (dataset/sim/C5.png, ...) avec la vérité
(worlds/voitures.csv) et affiche un score. Les images de contrôle (plaque redressée
+ ce que Tesseract a vu) vont dans dataset/controle/ pour comprendre les erreurs.

Utilisation :
    ros2 run robot_parking tester_lecture
"""
import csv
import os
import time

import cv2

from robot_parking.lecture_plaque import lire_plaque_image

IMAGES = os.path.expanduser('~/parking_ws/dataset/sim')
CONTROLE = os.path.expanduser('~/parking_ws/dataset/controle')
VERITE = os.path.expanduser('~/parking_ws/src/robot_parking/worlds/voitures.csv')


def main():
    with open(VERITE) as f:
        verite = {ligne['place']: ligne['plaque'] for ligne in csv.DictReader(f)}
    os.makedirs(CONTROLE, exist_ok=True)

    justes, total = 0, 0
    print(f"{'place':<6}{'vraie plaque':<16}{'lu':<16}{'temps':>8}")
    for place in sorted(verite):
        chemin = os.path.join(IMAGES, f'{place}.png')
        if not os.path.exists(chemin):
            print(f'{place:<6}{verite[place]:<16}(image absente)')
            continue
        image = cv2.imread(chemin)
        t0 = time.perf_counter()
        lu, controle = lire_plaque_image(image)
        duree = (time.perf_counter() - t0) * 1000
        cv2.imwrite(os.path.join(CONTROLE, f'{place}.png'), controle)

        total += 1
        ok = lu == verite[place]
        justes += ok
        print(f"{place:<6}{verite[place]:<16}{str(lu):<16}{duree:>6.0f} ms  {'✅' if ok else '❌'}")

    print(f'\nScore : {justes}/{total} plaques lues correctement')
    print(f'Images de contrôle : {CONTROLE}')


if __name__ == '__main__':
    main()
