"""
generer_carte.py : transforme la grille (carte.py) en carte ROS.

Une carte ROS = 2 fichiers :
    parking.pgm  -> une image en niveaux de gris (noir = obstacle, blanc = libre)
    parking.yaml -> la "légende" : taille d'un pixel, position de l'image, seuils

Utilisation :
    ros2 run robot_parking generer_carte
"""
import os

import numpy as np

from robot_parking.carte import creer_parking, TAILLE_CASE, OBSTACLE

DOSSIER = os.path.expanduser("~/parking_ws/src/robot_parking/maps")


def main():
    grille, _ = creer_parking()

    # 1) Image PGM : 0 = noir (obstacle), 254 = blanc (libre)
    image = np.where(grille == OBSTACLE, 0, 254).astype(np.uint8)
    hauteur, largeur = image.shape
    os.makedirs(DOSSIER, exist_ok=True)
    chemin_pgm = os.path.join(DOSSIER, "parking.pgm")
    with open(chemin_pgm, "wb") as f:
        f.write(f"P5\n{largeur} {hauteur}\n255\n".encode())   # en-tête du format PGM
        f.write(image.tobytes())                              # les pixels

    # 2) Fichier YAML : la légende de l'image
    chemin_yaml = os.path.join(DOSSIER, "parking.yaml")
    with open(chemin_yaml, "w") as f:
        f.write("image: parking.pgm\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {TAILLE_CASE}\n")     # 1 pixel = 2 cm
        f.write("origin: [0.0, 0.0, 0.0]\n")        # coin bas-gauche = (0, 0)
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")

    print(f"Carte écrite : {chemin_pgm} ({largeur} x {hauteur} pixels)")
    print(f"Légende      : {chemin_yaml}")


if __name__ == "__main__":
    main()
