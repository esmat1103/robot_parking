"""
generer_monde.py : construit le parking 3D pour Gazebo à partir de carte.py.

Il lit les MÊMES dimensions que la carte (murs, places, voitures garées),
donc le monde Gazebo et la carte RViz correspondent exactement.

Résultat : worlds/parking.world

Utilisation :
    ros2 run robot_parking generer_monde
"""
import os

from robot_parking.carte import (LARGEUR, HAUTEUR, EPAIS_MUR, LARG_PLACE, PROF_PLACE,
                                 LARG_VOITURE, LONG_VOITURE, PLACES_OCCUPEES,
                                 calculer_places)

DOSSIER = os.path.expanduser("~/parking_ws/src/robot_parking/worlds")

HAUT_MUR = 0.10        # hauteur des murs (10 cm)
HAUT_VOITURE = 0.06    # hauteur d'une miniature de voiture


def boite(nom, x, y, sx, sy, sz, couleur, statique=True):
    """Un objet 'boîte' Gazebo : position (x, y), taille (sx, sy, sz)."""
    return f"""
    <model name="{nom}">
      <static>{'true' if statique else 'false'}</static>
      <pose>{x:.3f} {y:.3f} {sz / 2:.3f} 0 0 0</pose>
      <link name="corps">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name="visuel">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{couleur}</ambient><diffuse>{couleur}</diffuse></material>
        </visual>
      </link>
    </model>"""


def ligne_sol(nom, x, y, sx, sy):
    """Marquage blanc au sol (visuel seulement : le robot roule dessus)."""
    return f"""
    <model name="{nom}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.0005 0 0 0</pose>
      <link name="corps">
        <visual name="visuel">
          <geometry><box><size>{sx:.3f} {sy:.3f} 0.001</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def main():
    objets = []

    # 1) Les 4 murs (gris)
    gris = "0.5 0.5 0.5 1"
    objets.append(boite("mur_bas", LARGEUR / 2, EPAIS_MUR / 2, LARGEUR, EPAIS_MUR, HAUT_MUR, gris))
    objets.append(boite("mur_haut", LARGEUR / 2, HAUTEUR - EPAIS_MUR / 2, LARGEUR, EPAIS_MUR, HAUT_MUR, gris))
    objets.append(boite("mur_gauche", EPAIS_MUR / 2, HAUTEUR / 2, EPAIS_MUR, HAUTEUR, HAUT_MUR, gris))
    objets.append(boite("mur_droit", LARGEUR - EPAIS_MUR / 2, HAUTEUR / 2, EPAIS_MUR, HAUTEUR, HAUT_MUR, gris))

    # 2) Les marquages des places (traits blancs entre les places)
    places = calculer_places()
    for nom, (x, y) in places.items():
        g = x - LARG_PLACE / 2                       # trait de gauche de la place
        objets.append(ligne_sol(f"trait_{nom}", g, y, 0.005, PROF_PLACE))
        if nom.endswith("6"):                        # dernier trait à droite
            objets.append(ligne_sol(f"trait_{nom}_d", x + LARG_PLACE / 2, y, 0.005, PROF_PLACE))

    # 3) Les voitures garées (boîtes colorées)
    couleurs = ["0.8 0.1 0.1 1", "0.1 0.3 0.8 1", "0.9 0.8 0.1 1", "0.1 0.6 0.2 1", "0.9 0.9 0.9 1"]
    for i, nom in enumerate(PLACES_OCCUPEES):
        x, y = places[nom]
        objets.append(boite(f"voiture_{nom}", x, y, LARG_VOITURE, LONG_VOITURE, HAUT_VOITURE,
                            couleurs[i % len(couleurs)]))

    monde = f"""<?xml version="1.0"?>
<!-- Fichier généré automatiquement par generer_monde.py : ne pas modifier à la main -->
<sdf version="1.6">
  <world name="parking">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- Caméra de la fenêtre Gazebo : vue d'ensemble de la maquette -->
    <gui>
      <camera name="vue">
        <pose>{LARGEUR / 2:.2f} -1.2 2.4 0 0.85 1.5708</pose>
      </camera>
    </gui>
{''.join(objets)}
  </world>
</sdf>
"""
    os.makedirs(DOSSIER, exist_ok=True)
    chemin = os.path.join(DOSSIER, "parking.world")
    with open(chemin, "w") as f:
        f.write(monde)
    print(f"Monde écrit : {chemin}")
    print(f"  4 murs, {len(places)} places, {len(PLACES_OCCUPEES)} voitures")


if __name__ == "__main__":
    main()
