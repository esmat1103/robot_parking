"""
carte.py : la maquette du parking décrite sous forme de grille.

Repère (le même que ROS) :
    x vers la droite, y vers le haut, en mètres.
    Origine (0, 0) = coin en bas à gauche de la maquette.

La grille découpe la maquette en petites cases de 2 cm :
    0 = libre (le robot peut passer)
    1 = obstacle (mur ou voiture garée)
"""
import numpy as np

# ---------- Réglages de la maquette (en mètres) ----------
TAILLE_CASE = 0.02        # 1 case = 2 cm

LARG_PLACE = 0.12         # largeur d'une place
PROF_PLACE = 0.22         # profondeur d'une place
LARG_ALLEE = 0.50         # largeur des allées
EPAIS_MUR = 0.04          # épaisseur du mur extérieur
PLACES_PAR_RANGEE = 6
N_BLOCS = 2               # 1 bloc = 2 rangées de places dos à dos

LARG_VOITURE = 0.08       # miniature de voiture
LONG_VOITURE = 0.19

# Taille totale de la maquette
LARGEUR = 2 * EPAIS_MUR + 2 * LARG_ALLEE + PLACES_PAR_RANGEE * LARG_PLACE
HAUTEUR = 2 * EPAIS_MUR + (N_BLOCS + 1) * LARG_ALLEE + N_BLOCS * 2 * PROF_PLACE

# Point de départ du robot : l'entrée, en bas à gauche, au milieu de l'allée
ENTREE = (EPAIS_MUR + LARG_ALLEE / 2, EPAIS_MUR + LARG_ALLEE / 2)

# Places où une voiture est garée (choix fixe, le même dans Gazebo)
PLACES_OCCUPEES = ["A1", "A3", "A4", "B2", "B6", "C1", "C5", "D2", "D3", "D6"]

# Plaques des voitures garées (SIMULATION de la vision : plus tard, la caméra
# + YOLO liront la vraie plaque ; ce tableau sera alors remplacé)
PLAQUES = {
    "A1": "112 TUN 3452", "A3": "205 TUN 1187", "A4": "098 TUN 7741",
    "B2": "187 TUN 2290", "B6": "231 TUN 0456",
    "C1": "150 TUN 9012", "C5": "176 TUN 5530",
    "D2": "214 TUN 6678", "D3": "121 TUN 3009", "D6": "199 TUN 8125",
}

# Distance entre le bord de la place et le point où le robot s'arrête (dans l'allée)
RECUL_APPROCHE = 0.17

LIBRE, OBSTACLE = 0, 1


def calculer_places():
    """Renvoie {nom: (x, y)} : le centre de chaque place.
    Rangées A, B, C, D de bas en haut ; places 1 à 6 de gauche à droite."""
    places = {}
    lettres = "ABCDEFGH"
    for b in range(N_BLOCS):
        y_bas = EPAIS_MUR + LARG_ALLEE + b * (2 * PROF_PLACE + LARG_ALLEE)
        for r in range(2):                       # 2 rangées dos à dos
            lettre = lettres[2 * b + r]
            y = y_bas + PROF_PLACE / 2 + r * PROF_PLACE
            for p in range(PLACES_PAR_RANGEE):
                x = EPAIS_MUR + LARG_ALLEE + (p + 0.5) * LARG_PLACE
                places[f"{lettre}{p + 1}"] = (x, y)
    return places


def point_approche(nom):
    """Point dans l'allée, juste devant l'entrée de la place 'nom'.
    Rangées A et C : entrée par le bas. Rangées B et D : entrée par le haut."""
    x, y = calculer_places()[nom]
    if nom[0] in "AC":
        return (x, y - PROF_PLACE / 2 - RECUL_APPROCHE)
    return (x, y + PROF_PLACE / 2 + RECUL_APPROCHE)


def remplir_rectangle(grille, xmin, ymin, xmax, ymax):
    """Met à OBSTACLE toutes les cases du rectangle (coordonnées en m)."""
    n_lignes = grille.shape[0]
    c1 = int(round(xmin / TAILLE_CASE))
    c2 = int(round(xmax / TAILLE_CASE))
    # la ligne 0 de la grille est en HAUT (y max), comme une image
    l1 = n_lignes - int(round(ymax / TAILLE_CASE))
    l2 = n_lignes - int(round(ymin / TAILLE_CASE))
    grille[max(l1, 0):l2, max(c1, 0):c2] = OBSTACLE


def creer_parking():
    """Construit la grille du parking. Renvoie (grille, places)."""
    n_col = int(round(LARGEUR / TAILLE_CASE))
    n_lig = int(round(HAUTEUR / TAILLE_CASE))
    grille = np.zeros((n_lig, n_col), dtype=np.uint8)

    # 1) les 4 murs
    remplir_rectangle(grille, 0, 0, LARGEUR, EPAIS_MUR)                    # bas
    remplir_rectangle(grille, 0, HAUTEUR - EPAIS_MUR, LARGEUR, HAUTEUR)    # haut
    remplir_rectangle(grille, 0, 0, EPAIS_MUR, HAUTEUR)                    # gauche
    remplir_rectangle(grille, LARGEUR - EPAIS_MUR, 0, LARGEUR, HAUTEUR)    # droite

    # 2) les voitures garées
    places = calculer_places()
    for nom in PLACES_OCCUPEES:
        x, y = places[nom]
        remplir_rectangle(grille,
                          x - LARG_VOITURE / 2, y - LONG_VOITURE / 2,
                          x + LARG_VOITURE / 2, y + LONG_VOITURE / 2)
    return grille, places


if __name__ == "__main__":
    g, places = creer_parking()
    print(f"Maquette : {LARGEUR:.2f} m x {HAUTEUR:.2f} m")
    print(f"Grille   : {g.shape[1]} colonnes x {g.shape[0]} lignes (cases de 2 cm)")
    print(f"Places   : {len(places)} dont {len(PLACES_OCCUPEES)} occupées")
