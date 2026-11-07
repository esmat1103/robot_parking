"""
generer_monde.py : construit le parking 3D pour Gazebo à partir de carte.py.

Il lit les MÊMES dimensions que la carte (murs, places, voitures garées),
donc le monde Gazebo et la carte RViz correspondent exactement.

Par défaut : voitures RÉALISTES (maillages 3D OSRF, 7 modèles couleur/type),
chacune avec sa plaque tunisienne à l'avant et à l'arrière. Les plaques sont
celles du tableau PLAQUES de carte.py -> la simulation et la mission sont cohérentes.

Résultats :
    worlds/parking.world         le monde Gazebo
    worlds/voitures.csv          la "vérité" (place, couleur, type, plaque) pour tester la vision
    models/plaques/materials/    les images des plaques (générées)

Utilisation :
    ros2 run robot_parking generer_monde            (voitures réalistes)
    ros2 run robot_parking generer_monde --boites   (anciennes boîtes colorées, plus léger)
    ros2 run robot_parking generer_monde --miroir   (si les plaques apparaissent à l'envers)
"""
import csv
import glob
import math
import os
import sys

from robot_parking.carte import (LARGEUR, HAUTEUR, EPAIS_MUR, LARG_PLACE, PROF_PLACE,
                                 LARG_VOITURE, LONG_VOITURE, PLACES_OCCUPEES, PLAQUES,
                                 calculer_places)

RACINE = os.path.expanduser("~/parking_ws/src/robot_parking")
DOSSIER = os.path.join(RACINE, "worlds")
MODELES = os.path.join(RACINE, "models")

HAUT_MUR = 0.10        # hauteur des murs (10 cm)
HAUT_VOITURE = 0.06    # hauteur d'une boîte-voiture (option --boites)

# Les 7 voitures réalistes : (dossier du modèle, fichier du maillage, couleur, type)
CATALOGUE = [
    ("hatchback_red",   "hatchback.obj", "rouge", "citadine"),
    ("hatchback_blue",  "hatchback.obj", "bleu",  "citadine"),
    ("hatchback_jaune", "hatchback.obj", "jaune", "citadine"),
    ("suv",             "suv.obj",       "noir",  "SUV"),
    ("hatchback_vert",  "hatchback.obj", "vert",  "citadine"),
    ("prius_hybrid",    "Hybrid.obj",    "blanc", "berline"),
    ("hatchback",       "hatchback.obj", "gris",  "citadine"),
]
PLAQUE_L, PLAQUE_H, PLAQUE_Z = 0.06, 0.015, 0.022   # plaque 6 x 1,5 cm, centre à 2,2 cm du sol


# ---------------------------------------------------------------- objets simples
def boite(nom, x, y, sx, sy, sz, couleur):
    """Un objet 'boîte' Gazebo statique : position (x, y), taille (sx, sy, sz)."""
    return f"""
    <model name="{nom}">
      <static>true</static>
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


# ---------------------------------------------------------------- voitures réalistes
def etendue_maillage(dossier, fichier):
    """(xmin, xmax, ymin, ymax, zmin, zmax) du maillage OBJ, dans ses propres unités.
    Sert à calculer l'échelle pour que chaque voiture fasse 19 x 8 cm."""
    xs, ys, zs = [], [], []
    with open(os.path.join(MODELES, dossier, "meshes", fichier), errors="ignore") as f:
        for ligne in f:
            if ligne.startswith("v "):
                x, y, z = map(float, ligne.split()[1:4])
                xs.append(x); ys.append(y); zs.append(z)
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


def image_plaque(serie, numero, miroir=False):
    """Plaque tunisienne : fond noir, chiffres blancs, « تونس » au milieu (600 x 150 px).
    Lue de droite à gauche : serie تونس numero  (ex : 176 تونس 5530)."""
    from PIL import Image, ImageDraw, ImageFont
    img = Image.new("RGB", (600, 150), (15, 15, 15))
    d = ImageDraw.Draw(img)
    d.rectangle([6, 6, 593, 143], outline=(235, 235, 235), width=6)
    try:
        police = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 66)
    except OSError:
        police = ImageFont.load_default()
    d.text((25, 75), numero, font=police, fill=(240, 240, 240), anchor="lm")
    d.text((575, 75), serie, font=police, fill=(240, 240, 240), anchor="rm")
    mot = Image.open(os.path.join(MODELES, "plaques", "tounes.png"))   # « تونس » déjà mis en forme
    mot = mot.resize((int(mot.width * 0.7), int(mot.height * 0.7)))
    img.paste(mot, (300 - mot.width // 2, 75 - mot.height // 2), mot)
    if miroir:
        img = img.transpose(Image.FLIP_LEFT_RIGHT)
    return img


def voiture_realiste(nom, x, y, modele, ext, echelle, plaque):
    """Une voiture : maillage 3D + boîte de collision + 2 plaques (avant et arrière).
    La longueur de la voiture est le long de y, comme les places."""
    dossier, fichier = modele[0], modele[1]
    xmin, xmax, ymin, ymax, zmin, zmax = ext
    cx, cy = (xmin + xmax) / 2 * echelle, (ymin + ymax) / 2 * echelle
    lx, ly, lz = (xmax - xmin) * echelle, (ymax - ymin) * echelle, (zmax - zmin) * echelle
    mat = f"""<material><script><uri>model://plaques/materials/scripts</uri>
            <uri>model://plaques/materials/textures</uri><name>Plaque/{plaque}</name></script></material>"""
    return f"""
    <model name="{nom}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="corps">
        <collision name="collision">
          <pose>0 0 {lz / 2:.3f} 0 0 0</pose>
          <geometry><box><size>{lx:.3f} {ly:.3f} {lz:.3f}</size></box></geometry>
        </collision>
        <visual name="carrosserie">
          <pose>{-cx:.4f} {-cy:.4f} {-zmin * echelle:.4f} 0 0 0</pose>
          <geometry><mesh><uri>model://{dossier}/meshes/{fichier}</uri>
            <scale>{echelle:.6f} {echelle:.6f} {echelle:.6f}</scale></mesh></geometry>
        </visual>
        <visual name="plaque_haut">
          <pose>0 {ly / 2 + 0.0015:.4f} {PLAQUE_Z} 0 0 0</pose>
          <geometry><box><size>{PLAQUE_L} 0.002 {PLAQUE_H}</size></box></geometry>
          {mat}
        </visual>
        <visual name="plaque_bas">
          <pose>0 {-ly / 2 - 0.0015:.4f} {PLAQUE_Z} 0 0 3.1416</pose>
          <geometry><box><size>{PLAQUE_L} 0.002 {PLAQUE_H}</size></box></geometry>
          {mat}
        </visual>
      </link>
    </model>"""


def materiau_plaque(plaque):
    """Script OGRE : dit à Gazebo d'afficher l'image plaque_<nom>.png sur la plaque."""
    return f"""material Plaque/{plaque}
{{
  technique {{ pass {{
    lighting off
    texture_unit {{ texture plaque_{plaque}.png
                   filtering trilinear }}
  }} }}
}}
"""


# ---------------------------------------------------------------- monde complet
def main():
    boites = "--boites" in sys.argv
    miroir = "--miroir" in sys.argv
    objets = []

    # 1) Les 4 murs (gris)
    gris = "0.5 0.5 0.5 1"
    objets.append(boite("mur_bas", LARGEUR / 2, EPAIS_MUR / 2, LARGEUR, EPAIS_MUR, HAUT_MUR, gris))
    objets.append(boite("mur_haut", LARGEUR / 2, HAUTEUR - EPAIS_MUR / 2, LARGEUR, EPAIS_MUR, HAUT_MUR, gris))
    objets.append(boite("mur_gauche", EPAIS_MUR / 2, HAUTEUR / 2, EPAIS_MUR, HAUTEUR, HAUT_MUR, gris))
    objets.append(boite("mur_droit", LARGEUR - EPAIS_MUR / 2, HAUTEUR / 2, EPAIS_MUR, HAUTEUR, HAUT_MUR, gris))

    # 2) Les marquages des places
    places = calculer_places()
    for nom, (x, y) in places.items():
        objets.append(ligne_sol(f"trait_{nom}", x - LARG_PLACE / 2, y, 0.005, PROF_PLACE))
        if nom.endswith("6"):
            objets.append(ligne_sol(f"trait_{nom}_d", x + LARG_PLACE / 2, y, 0.005, PROF_PLACE))

    # 3) Les voitures garées
    verite = []          # (place, couleur, type, plaque)
    if boites:
        couleurs = ["0.8 0.1 0.1 1", "0.1 0.3 0.8 1", "0.9 0.8 0.1 1", "0.1 0.6 0.2 1", "0.9 0.9 0.9 1"]
        for i, nom in enumerate(PLACES_OCCUPEES):
            x, y = places[nom]
            objets.append(boite(f"voiture_{nom}", x, y, LARG_VOITURE, LONG_VOITURE, HAUT_VOITURE,
                                couleurs[i % len(couleurs)]))
            verite.append((nom, "boîte", "boîte", PLAQUES.get(nom, "")))
    else:
        dossier_tex = os.path.join(MODELES, "plaques", "materials", "textures")
        dossier_scr = os.path.join(MODELES, "plaques", "materials", "scripts")
        os.makedirs(dossier_tex, exist_ok=True)
        os.makedirs(dossier_scr, exist_ok=True)
        for ancien in glob.glob(os.path.join(dossier_tex, "plaque_*.png")):
            os.remove(ancien)                            # on efface les anciennes plaques

        etendues = {m[0]: etendue_maillage(m[0], m[1]) for m in CATALOGUE}
        materiaux = []
        for i, nom in enumerate(PLACES_OCCUPEES):
            x, y = places[nom]
            modele = CATALOGUE[i % len(CATALOGUE)]
            ext = etendues[modele[0]]
            # même emprise que la carte (19 x 8 cm) : la voiture ne la dépasse jamais
            echelle = min(LONG_VOITURE / (ext[3] - ext[2]), LARG_VOITURE / (ext[1] - ext[0]))
            serie, _, numero = PLAQUES[nom].split()      # "176 TUN 5530" -> 176, 5530
            image_plaque(serie, numero, miroir).save(os.path.join(dossier_tex, f"plaque_{nom}.png"))
            materiaux.append(materiau_plaque(nom))
            objets.append(voiture_realiste(f"voiture_{nom}", x, y, modele, ext, echelle, nom))
            verite.append((nom, modele[2], modele[3], PLAQUES[nom]))
        with open(os.path.join(dossier_scr, "plaques.material"), "w") as f:
            f.write("\n".join(materiaux))

    monde = f"""<?xml version="1.0"?>
<!-- Fichier généré automatiquement par generer_monde.py : ne pas modifier à la main -->
<sdf version="1.6">
  <world name="parking">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

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
    with open(os.path.join(DOSSIER, "voitures.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["place", "couleur", "type", "plaque"])
        w.writerows(verite)

    print(f"Monde écrit : {chemin}")
    print(f"  4 murs, {len(places)} places, {len(verite)} voitures "
          f"({'boîtes' if boites else 'réalistes'}{', plaques en miroir' if miroir else ''}) :")
    for place, couleur, type_v, plaque in verite:
        print(f"    {place} : {couleur:<6} {type_v:<9} {plaque}")
    print(f"Vérité écrite : {os.path.join(DOSSIER, 'voitures.csv')}")


if __name__ == "__main__":
    main()
