# Journal de bord — Robot Parking (partie navigation)

Une entrée par étape : **objectif → ce qui a été fait → résultat → ce que j'ai appris.**
Ce journal sert au groupe, au rapport, et au portfolio / LinkedIn.

---

## Étape 1 — Package ROS 2 et Git · ✅

**Objectif :** une base de projet propre et partageable pour les 4 membres.

**Fait :**
- workspace séparé `~/parking_ws` (le TP SLAM reste dans `~/ros2_ws`)
- package Python `robot_parking` (`ament_python`) avec dossiers `launch/ params/ maps/ worlds/`
- `package.xml` (identité, licence MIT, dépendances) et `setup.py` (installation des fichiers)
- dépôt Git + GitHub

**Résultat :** `colcon build` → `1 package finished` ; `ros2 pkg list` voit `robot_parking`.

**Appris :** workspace / package / `colcon build` / `source` ; rôle de `package.xml` (étiquette)
vs `setup.py` (notice d'installation) ; Git (`add`, `commit`, `push`), token GitHub.

---

## Étape 2 — Carte de la maquette dans RViz · ✅

**Objectif :** donner au robot le plan du parking (indispensable pour planifier un chemin).

**Fait :**
- `carte.py` : maquette 1,80 m × 2,46 m, grille de cases de 2 cm, 24 places (A1→D6), 10 voitures garées
- `generer_carte.py` : grille → carte ROS `parking.pgm` (noir = obstacle) + `parking.yaml` (1 px = 2 cm)
- `carte.launch.py` : `map_server` publie `/map`, `lifecycle_manager` l'active, RViz l'affiche

**Choix :** carte **dessinée** à partir des mesures (maquette connue et fixe) plutôt que par SLAM.
Le SLAM produit le même format de fichier → remplaçable plus tard sans changer le reste.

**Résultat :**

| Carte annotée | Carte dans RViz |
|---|---|
| ![carte](images/etape2_carte_annotee.png) | ![rviz](images/etape2_rviz.png) |

**Appris :** format de carte ROS (PGM + YAML, résolution, origine), repère ROS (x droite, y haut),
nœuds lifecycle de Nav2, QoS *Transient Local* du topic `/map`.

---

## Étape 3 — Robot simulé dans Gazebo · ✅

**Objectif :** un « faux parking réel » pour tester sans risque.

**Fait :**
- `generer_monde.py` : le monde Gazebo construit à partir du **même** `carte.py` (cohérence carte ↔ monde)
- `robot.urdf` : châssis 20 × 14 cm, 2 roues motrices Ø 65 mm, capteur de distance avant (70°, 1,2 m)
- `simulation.launch.py` : Gazebo + robot + carte + RViz en une commande

- test : pilotage au clavier (`teleop_twist_keyboard`) → le robot bouge dans Gazebo **et** dans RViz

**Résultat :** le robot apparaît au même endroit dans les deux fenêtres ; les points du capteur
tombent sur les murs de la carte → **le monde Gazebo et la carte correspondent**.

| Gazebo (parking 3D + rayons du capteur) | RViz (carte + robot + points du capteur) |
|---|---|
| ![gazebo](images/etape3_gazebo.png) | ![rviz](images/etape3_rviz.png) |

**Chaîne validée :** clavier → `/cmd_vel` → roues (Gazebo) → `/odom` + `/scan` → RViz.
Les étapes suivantes remplacent le clavier par du code (A*, suivi de chemin, sécurité).

**Appris :** URDF (links, joints, plugins Gazebo), `robot_state_publisher`, arbre TF
`map → odom → base_footprint → base_link`, `use_sim_time`, plugin `diff_drive`, capteur `ray` → `/scan`.

---

## Étape 4 — Planification A* dans RViz · ✅

**Objectif :** calculer automatiquement le plus court chemin sûr vers une destination (« décider où aller »).

**Fait :**
- `astar.py` (sans ROS, testable seul) :
  - `gonfler()` : agrandit les obstacles de 15 cm (demi-diagonale du robot 12 cm + marge 3 cm)
    → A* peut traiter le robot comme un **point**
  - `astar()` : grille 8-voisins, heuristique octile, interdiction de couper les coins
  - `case_libre_proche()` : si on clique sur un obstacle, on vise la case libre la plus proche
- `planificateur.py` (nœud ROS 2) : écoute `/map`, `/odom`, `/goal_pose` → publie `/plan` et `/carte_securite`
- lancé automatiquement par `simulation.launch.py`

**Résultat :** clic « 2D Goal Pose » dans RViz → chemin orange en quelques millisecondes.

| A* dans RViz (zone de sécurité en bleu clair) | Test hors ROS (3 destinations) |
|---|---|
| ![astar](images/etape4_astar.png) | ![test](images/etape4_test_hors_ros.png) |

**Limite observée :** le chemin longe le bord de la zone de sécurité et fait de petits « escaliers »
(grille 8 directions) → sera lissé à l'étape 5 avant le suivi.

**Appris :** A* (f = g + h, heuristique admissible), gonflement d'obstacles / espace des configurations,
OccupancyGrid (ligne 0 = bas, valeurs 0/100/-1), nav_msgs/Path, QoS Transient Local, paramètres ROS.

---

## Étape 5 — Suivi autonome du chemin · ✅

**Objectif :** remplacer le clavier par un programme (« conduire »).

**Fait :**
- lissage (`simplifier()` dans `astar.py`) : depuis chaque point, saut au point le plus lointain
  atteignable en ligne droite sans toucher la zone de sécurité → les « escaliers » disparaissent
- `suiveur.py` : 20 fois par seconde, vise le prochain point ; pivote sur place si l'écart d'angle
  dépasse 20°, sinon avance (ralentit près du point) ; point suivant à moins de 4 cm ; arrêt au dernier
- `/plan_brut` (gris) et `/plan` lissé (orange) affichés dans RViz

**Résultat :** chemin A* de 2,56 m (112 cases, 26 ms) → lissé en **9 points / 2,43 m**,
parcouru en autonomie en **≈ 20 s** (vitesse max 0,15 m/s).

![suivi](images/etape5_suivi.gif)

**Appris :** commande d'un robot différentiel (v, ω), quaternion → angle de cap, normalisation d'angle,
boucle de commande à fréquence fixe (timer), compromis vitesse / précision.

---

## Étape 6 — Arrêt de sécurité · ✅

**Objectif :** ne jamais heurter un obstacle **absent de la carte** (personne, carton, voiture mal garée).

**Fait :**
- `securite.py` : « garde du corps » entre le suiveur et les moteurs
  (`suiveur → /cmd_vel_brut → securite → /cmd_vel → moteurs`), grâce à un **remapping** dans le launch
  (le suiveur n'a pas été modifié)
- règles : obstacle < 12 cm → marche avant bloquée ; 12–30 cm → ralentissement ;
  capteur muet > 0,5 s → arrêt ; rotation et recul toujours permis

**Test :** cube ajouté dans Gazebo au milieu de l'allée (inconnu de la carte).

| Temps | Événement |
|---|---|
| +5 s | cube détecté à 30 cm → RALENTI |
| +13 s | cube à 12 cm → **ARRÊT, sans contact** |
| +67 s | cube retiré → OK, le robot repart |
| +73 s | arrivé à destination |

![securite](images/etape6_securite.gif)

**Limite :** le robot attend que la voie se libère ; il ne contourne pas encore l'obstacle
(→ bonus : re-planification).

**Appris :** séparation des responsabilités entre nœuds, remapping de topics, QoS capteur (Best Effort),
principe de « fail-safe » (arrêt si le capteur se tait).
