"""
astar.py : calcul du chemin le plus court sur la grille (sans ROS, testable seul).

Deux fonctions principales :
    gonfler(obstacles, rayon)  -> agrandit les obstacles de la taille du robot
    astar(bloque, depart, but) -> liste des cases du chemin, ou None

Pourquoi "gonfler" ? A* traite le robot comme un POINT. Si on agrandit chaque
obstacle de la demi-taille du robot (+ une marge), un chemin pour un point
devient un chemin sûr pour le vrai robot.

Une case = (ligne, colonne). La ligne 0 est en BAS de la carte (convention ROS).
"""
import heapq
import math
from collections import deque

import numpy as np

# Les 8 voisins d'une case : (déplacement ligne, déplacement colonne, coût)
VOISINS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
           (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
           (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def gonfler(obstacles, rayon_cases):
    """Renvoie une nouvelle grille où chaque obstacle est agrandi d'un disque
    de 'rayon_cases' cases. obstacles = tableau de booléens (True = obstacle)."""
    h, w = obstacles.shape
    resultat = obstacles.copy()
    r = rayon_cases
    for dl in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dl * dl + dc * dc > r * r:
                continue                      # hors du disque
            # on décale toute la grille d'obstacles de (dl, dc) et on l'ajoute
            source = obstacles[max(0, -dl):h - max(0, dl), max(0, -dc):w - max(0, dc)]
            resultat[max(0, dl):h - max(0, -dl), max(0, dc):w - max(0, -dc)] |= source
    return resultat


def heuristique(a, b):
    """Distance 'octile' : la vraie distance si on peut aller en 8 directions
    et qu'il n'y a pas d'obstacle. Elle ne surestime jamais -> A* reste optimal."""
    dl, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dl + dc) + (math.sqrt(2) - 2) * min(dl, dc)


def astar(bloque, depart, but):
    """Chemin le plus court de 'depart' à 'but' en évitant les cases bloquées.
    A* explore en priorité la case qui minimise  f = g + h :
        g = distance déjà parcourue depuis le départ
        h = estimation de la distance restante (heuristique)"""
    h, w = bloque.shape
    a_explorer = [(heuristique(depart, but), 0.0, depart)]   # file de priorité
    parent = {depart: None}
    cout = {depart: 0.0}

    while a_explorer:
        _, g, case = heapq.heappop(a_explorer)
        if case == but:                       # arrivé : on remonte les parents
            chemin = []
            while case is not None:
                chemin.append(case)
                case = parent[case]
            return chemin[::-1]
        if g > cout[case]:
            continue                          # ancienne entrée, déjà améliorée

        for dl, dc, pas in VOISINS:
            l, c = case[0] + dl, case[1] + dc
            if not (0 <= l < h and 0 <= c < w) or bloque[l, c]:
                continue
            # en diagonale, interdit de "couper le coin" d'un obstacle
            if dl and dc and (bloque[case[0] + dl, case[1]] or bloque[case[0], case[1] + dc]):
                continue
            nouveau = g + pas
            if nouveau < cout.get((l, c), math.inf):
                cout[(l, c)] = nouveau
                parent[(l, c)] = case
                heapq.heappush(a_explorer, (nouveau + heuristique((l, c), but), nouveau, (l, c)))
    return None                               # aucun chemin possible


def case_libre_proche(bloque, case, distance_max=30):
    """Si 'case' est bloquée (ex : on a cliqué sur une voiture), renvoie la case
    libre la plus proche. Recherche en largeur (cercles de plus en plus grands)."""
    h, w = bloque.shape
    if not bloque[case]:
        return case
    vus = {case}
    file = deque([case])
    while file:
        l, c = file.popleft()
        if abs(l - case[0]) > distance_max or abs(c - case[1]) > distance_max:
            continue
        for dl, dc, _ in VOISINS:
            v = (l + dl, c + dc)
            if 0 <= v[0] < h and 0 <= v[1] < w and v not in vus:
                if not bloque[v]:
                    return v
                vus.add(v)
                file.append(v)
    return None


def ligne_libre(bloque, a, b):
    """Vrai si le segment droit entre les cases a et b ne traverse aucune case bloquée.
    On teste des points tous les 1/2 case le long du segment."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    for i in range(n + 1):
        t = i / n
        l = int(round(a[0] + t * (b[0] - a[0])))
        c = int(round(a[1] + t * (b[1] - a[1])))
        if bloque[l, c]:
            return False
    return True


def simplifier(bloque, chemin):
    """Lissage : on remplace les 'escaliers' par des lignes droites.
    Depuis un point, on saute directement au point le plus lointain
    qu'on peut atteindre en ligne droite sans toucher la zone interdite."""
    if len(chemin) <= 2:
        return list(chemin)
    resultat = [chemin[0]]
    i = 0
    while i < len(chemin) - 1:
        j = len(chemin) - 1
        while j > i + 1 and not ligne_libre(bloque, chemin[i], chemin[j]):
            j -= 1
        resultat.append(chemin[j])
        i = j
    return resultat


def longueur(chemin, taille_case):
    """Longueur du chemin en mètres."""
    total = 0.0
    for (l1, c1), (l2, c2) in zip(chemin, chemin[1:]):
        total += math.hypot(l2 - l1, c2 - c1)
    return total * taille_case


if __name__ == "__main__":
    # Petit test sans ROS : du coin bas-gauche au coin haut-droit d'une grille vide
    g = np.zeros((20, 20), dtype=bool)
    g[5:15, 10] = True                        # un mur au milieu
    ch = astar(gonfler(g, 1), (1, 1), (18, 18))
    print("Chemin trouvé :", len(ch), "cases")
