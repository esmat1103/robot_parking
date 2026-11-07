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
