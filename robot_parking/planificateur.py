"""
planificateur.py : nœud ROS 2 qui calcule le chemin avec A* (+ re-planification, étape D2).

Écoute :
    /map        (OccupancyGrid) : la carte du parking (obstacles FIXES connus)
    /odom       (Odometry)      : la position actuelle du robot
    /goal_pose  (PoseStamped)   : la destination (RViz "2D Goal Pose" ou nœud mission)
    /scan       (LaserScan)     : le capteur de distance (obstacles IMPRÉVUS)       [D2]
    /arrivee    (Bool)          : le suiveur est arrivé -> plus rien à surveiller   [D2]
Publie :
    /plan_brut       (Path)          : le chemin A* brut (case par case)
    /plan            (Path)          : le chemin lissé, à suivre
    /carte_securite  (OccupancyGrid) : les obstacles fixes "gonflés" (zone interdite)
    /obstacles_vus   (OccupancyGrid) : les obstacles imprévus vus par le capteur    [D2]

Re-planification (D2) + priorité piéton (D2b) :
    1. chaque point vu par le capteur, qui n'est PAS un obstacle connu de la carte,
       est noté comme « obstacle imprévu » (et oublié s'il n'est plus vu depuis 3 s :
       un piéton qui passe ne bloque pas la case pour toujours)
    2. deux fois par seconde : si le chemin RESTANT traverse un obstacle imprévu (gonflé),
       A* recalcule un chemin qui le contourne
    3. si aucun contournement n'existe, on garde le chemin : la sécurité fait attendre le robot
    D2b : un point vu près du PIÉTON (/pieton) est « mobile » : on ne contourne pas,
    on ATTEND qu'il passe (au plus 8 s), comme un conducteur dans un parking.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray

from robot_parking.astar import gonfler, astar, case_libre_proche, longueur, simplifier

POSITION_LASER = 0.09      # m : le capteur est 9 cm devant le centre du robot (robot.urdf)
PORTEE_DETECTION = 0.60    # m : on ne tient compte que des obstacles proches
MEMOIRE = 3.0              # s : un obstacle IMMOBILE non revu depuis 3 s est oublié
MEMOIRE_MOBILE = 0.5       # s : le PIÉTON est oublié vite (on connaît sa position en continu)
DELAI_REPLANIF = 1.5       # s : au plus une re-planification toutes les 1,5 s
RAYON_PIETON = 0.12        # m : un point vu à moins de 12 cm du piéton = le piéton (D2b)
ATTENTE_MAX = 8.0          # s : si un piéton bloque plus longtemps, on contourne quand même (D2b)


class Planificateur(Node):

    def __init__(self):
        super().__init__('planificateur')

        # Réglage : demi-diagonale du robot (12 cm) + marge (3 cm)
        self.declare_parameter('rayon_securite', 0.15)
        self.declare_parameter('replanification', True)    # False = comportement de l'étape 6

        # QoS "Transient Local" : on reçoit la carte même si elle a été publiée avant
        qos_carte = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, '/map', self.recevoir_carte, qos_carte)
        self.create_subscription(Odometry, '/odom', self.recevoir_odom, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.recevoir_but, 10)
        self.create_subscription(LaserScan, '/scan', self.recevoir_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/arrivee', self.recevoir_arrivee, 10)
        # D2b : où est le piéton ? (en simulation : nœud pieton ; en vrai : la vision, classe « person »)
        self.create_subscription(Float32MultiArray, '/pieton', self.recevoir_pieton, 10)

        self.pub_plan = self.create_publisher(Path, '/plan', qos_carte)            # chemin lissé
        self.pub_brut = self.create_publisher(Path, '/plan_brut', qos_carte)       # chemin A* brut
        self.pub_securite = self.create_publisher(OccupancyGrid, '/carte_securite', qos_carte)
        self.pub_vus = self.create_publisher(OccupancyGrid, '/obstacles_vus', qos_carte)

        self.carte = None          # message /map reçu
        self.bloque = None         # grille True/False des cases interdites (obstacles fixes)
        self.connu = None          # obstacles fixes + 2 cases : ce que le capteur « reconnaît »
        self.rayon_cases = 8
        self.position = None       # (x, y) du robot en mètres
        self.theta = 0.0           # orientation du robot (rad)

        self.but = None            # destination en cours (PoseStamped), None = rien à faire
        self.chemin = None         # chemin A* brut en cours (liste de cases)
        self.vus = {}              # obstacles imprévus IMMOBILES : {case: heure de la dernière détection}
        self.vus_mobiles = {}      # obstacles imprévus MOBILES (piéton) : pareil              [D2b]
        self.pieton = None         # (x, y) du piéton s'il est présent, sinon None            [D2b]
        self.debut_attente = None  # heure à laquelle on a commencé à attendre le piéton     [D2b]
        self.detour_pieton = False # True = le chemin actuel est un détour à cause d'un piéton [D2b]
        self.derniere_replanif = 0.0

        self.create_timer(0.5, self.surveiller)
        self.get_logger().info('Planificateur prêt : clique sur "2D Goal Pose" dans RViz.')

    # ---------- Conversions mètres <-> cases ----------
    def vers_case(self, x, y):
        info = self.carte.info
        c = int((x - info.origin.position.x) / info.resolution)
        l = int((y - info.origin.position.y) / info.resolution)
        return (l, c)

    def vers_metres(self, case):
        info = self.carte.info
        x = info.origin.position.x + (case[1] + 0.5) * info.resolution
        y = info.origin.position.y + (case[0] + 0.5) * info.resolution
        return x, y

    def dans_carte(self, case):
        return 0 <= case[0] < self.bloque.shape[0] and 0 <= case[1] < self.bloque.shape[1]

    # ---------- Réception des messages ----------
    def recevoir_carte(self, msg):
        self.carte = msg
        h, w = msg.info.height, msg.info.width
        valeurs = np.array(msg.data, dtype=np.int16).reshape(h, w)   # ligne 0 = bas
        obstacles = (valeurs >= 50) | (valeurs < 0)                  # occupé ou inconnu

        rayon = self.get_parameter('rayon_securite').value
        self.rayon_cases = math.ceil(rayon / msg.info.resolution)
        self.bloque = gonfler(obstacles, self.rayon_cases)
        self.connu = gonfler(obstacles, 2)    # un point du capteur sur un mur connu n'est pas « imprévu »

        # On publie la zone interdite pour la voir dans RViz
        securite = OccupancyGrid()
        securite.header = msg.header
        securite.info = msg.info
        grille = np.where(self.bloque, 99, 0).astype(np.int8)
        grille[obstacles] = 100
        securite.data = grille.flatten().tolist()
        self.pub_securite.publish(securite)

        self.get_logger().info(
            f'Carte reçue : {w} x {h} cases de {msg.info.resolution * 100:.0f} cm, '
            f'obstacles gonflés de {self.rayon_cases} cases '
            f'({self.rayon_cases * msg.info.resolution * 100:.0f} cm)')

    def recevoir_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.position = (p.x, p.y)
        self.theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def recevoir_but(self, msg):
        self.but = msg
        self.debut_attente = None
        self.detour_pieton = False
        self.planifier('nouvelle destination')

    def recevoir_arrivee(self, msg):
        if msg.data:
            self.but = None          # mission accomplie : plus de surveillance
            self.chemin = None

    def recevoir_pieton(self, msg):
        present, x, y = msg.data[0], msg.data[1], msg.data[2]
        self.pieton = (x, y) if present > 0.5 else None

    def recevoir_scan(self, msg):
        """Note chaque point vu par le capteur qui n'est pas un obstacle connu."""
        if self.carte is None or self.position is None:
            return
        maintenant = time.monotonic()
        x, y = self.position
        lx = x + POSITION_LASER * math.cos(self.theta)               # position du capteur
        ly = y + POSITION_LASER * math.sin(self.theta)
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= min(msg.range_max, PORTEE_DETECTION)):
                continue
            a = self.theta + msg.angle_min + i * msg.angle_increment
            px, py = lx + r * math.cos(a), ly + r * math.sin(a)
            case = self.vers_case(px, py)
            if self.dans_carte(case) and not self.connu[case]:
                # D2b : ce point est-il le piéton (mobile) ou un objet immobile ?
                if self.pieton is not None and math.dist((px, py), self.pieton) < RAYON_PIETON:
                    self.vus_mobiles[case] = maintenant
                else:
                    self.vus[case] = maintenant

    # ---------- Re-planification (D2) ----------
    def obstacles_imprevus(self, inclure_mobiles=False):
        """Grille True/False des obstacles imprévus encore « frais » (vus < 3 s).
        Par défaut : seulement les IMMOBILES (on attend les piétons au lieu de les contourner)."""
        maintenant = time.monotonic()
        self.vus = {c: t for c, t in self.vus.items() if maintenant - t < MEMOIRE}
        self.vus_mobiles = {c: t for c, t in self.vus_mobiles.items() if maintenant - t < MEMOIRE_MOBILE}
        grille = np.zeros_like(self.bloque)
        for case in list(self.vus) + (list(self.vus_mobiles) if inclure_mobiles else []):
            grille[case] = True
        return grille

    def chemin_coupe(self, obstacles_gonfles):
        """Le chemin RESTANT (à partir du point le plus proche du robot) est-il coupé ?"""
        ici = self.vers_case(*self.position)
        k = min(range(len(self.chemin)),
                key=lambda i: (self.chemin[i][0] - ici[0]) ** 2 + (self.chemin[i][1] - ici[1]) ** 2)
        return any(obstacles_gonfles[c] for c in self.chemin[k:])

    def surveiller(self):
        """2 fois par seconde :
        - obstacle IMMOBILE sur le chemin -> on contourne (re-planification)
        - PIÉTON sur le chemin -> on attend qu'il passe (la sécurité arrête le robot) ;
          s'il bloque plus de 8 s, on contourne quand même"""
        if self.carte is None:
            return
        immobiles = self.obstacles_imprevus()
        tous = self.obstacles_imprevus(inclure_mobiles=True)
        tous_gonfles = gonfler(tous, self.rayon_cases) if tous.any() else tous
        self.publier_obstacles_vus(tous, tous_gonfles)

        if (not self.get_parameter('replanification').value or self.but is None
                or self.chemin is None or self.position is None):
            return
        maintenant = time.monotonic()

        # 1) obstacle immobile : contourner
        if immobiles.any():
            if (self.chemin_coupe(gonfler(immobiles, self.rayon_cases))
                    and maintenant - self.derniere_replanif > DELAI_REPLANIF):
                self.planifier('obstacle imprévu sur le chemin')
                return

        # 2) piéton : attendre (au plus ATTENTE_MAX secondes)
        mobiles = tous & ~immobiles
        if mobiles.any() and self.chemin_coupe(gonfler(mobiles, self.rayon_cases)):
            if self.debut_attente is None:
                self.debut_attente = maintenant
                self.get_logger().info('Piéton sur le chemin : PRIORITÉ PIÉTON, j\'attends qu\'il passe')
            elif (maintenant - self.debut_attente > ATTENTE_MAX
                  and maintenant - self.derniere_replanif > DELAI_REPLANIF
                  and self.pieton is not None):              # il est TOUJOURS là
                self.get_logger().info(f'Le piéton bloque depuis plus de {ATTENTE_MAX:.0f} s : je contourne')
                self.planifier('piéton immobile trop longtemps', inclure_mobiles=True)
                self.detour_pieton = True
                self.debut_attente = None                  # on n'attend plus : on contourne
        elif self.debut_attente is not None:
            self.get_logger().info(f'Passage libre après {maintenant - self.debut_attente:.1f} s '
                                   f'd\'attente : je continue')
            self.debut_attente = None
        elif self.detour_pieton and self.pieton is None:
            # on avait contourné un piéton, et il est PARTI : on reprend le chemin le plus court
            self.detour_pieton = False
            self.planifier('piéton parti, retour au chemin le plus court')

    def planifier(self, raison, inclure_mobiles=False):
        """Calcule (ou recalcule) le chemin vers self.but, en évitant les obstacles
        fixes ET les obstacles imprévus vus par le capteur (piétons seulement si demandé)."""
        if self.carte is None or self.position is None or self.but is None:
            self.get_logger().warn('Pas encore de carte ou de position du robot.')
            return
        self.derniere_replanif = time.monotonic()

        imprevus = self.obstacles_imprevus(inclure_mobiles)
        bloque = self.bloque | gonfler(imprevus, self.rayon_cases) if imprevus.any() else self.bloque

        depart = self.vers_case(*self.position)
        but = self.vers_case(self.but.pose.position.x, self.but.pose.position.y)
        if not self.dans_carte(depart) or not self.dans_carte(but):
            self.get_logger().warn('Départ ou destination hors de la carte.')
            return

        # le robot peut se trouver DANS la zone gonflée d'un obstacle qu'il vient de voir :
        # on part de la case libre la plus proche
        depart = case_libre_proche(bloque, depart)
        but = case_libre_proche(bloque, but)
        if depart is None or but is None:
            self.get_logger().warn('Aucune case libre près du départ ou de la destination.')
            return

        t0 = time.perf_counter()
        chemin = astar(bloque, depart, but)
        duree_ms = (time.perf_counter() - t0) * 1000

        if chemin is None:
            if raison == 'nouvelle destination':
                self.get_logger().warn('Aucun chemin trouvé !')
            else:
                self.get_logger().warn('Pas de contournement possible : on garde le chemin '
                                       '(la sécurité fera attendre le robot)')
            return

        self.chemin = chemin
        lisse = simplifier(bloque, chemin)
        # l'orientation demandée (flèche de RViz, ou "face à la place" pour la mission)
        # est recopiée sur le DERNIER point : le suiveur s'alignera dessus à l'arrivée
        self.publier_chemin(self.pub_brut, chemin, self.but.pose.orientation)
        self.publier_chemin(self.pub_plan, lisse, self.but.pose.orientation)
        res = self.carte.info.resolution
        entete = 'Chemin trouvé' if raison == 'nouvelle destination' else f'RE-PLANIFICATION ({raison})'
        self.get_logger().info(
            f'{entete} : {len(chemin)} cases, {longueur(chemin, res):.2f} m, '
            f'calculé en {duree_ms:.1f} ms -> lissé : {len(lisse)} points, {longueur(lisse, res):.2f} m')

    # ---------- Publication ----------
    def publier_chemin(self, publieur, chemin, orientation_finale):
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = self.get_clock().now().to_msg()
        for case in chemin:
            pose = PoseStamped()
            pose.header = plan.header
            pose.pose.position.x, pose.pose.position.y = self.vers_metres(case)
            pose.pose.orientation.w = 1.0
            plan.poses.append(pose)
        plan.poses[-1].pose.orientation = orientation_finale
        publieur.publish(plan)

    def publier_obstacles_vus(self, imprevus, imprevus_gonfles):
        """Obstacles imprévus pour RViz : 100 = vu par le capteur, 99 = zone gonflée."""
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.carte.info
        grille = np.where(imprevus_gonfles, 99, 0).astype(np.int8)
        grille[imprevus] = 100
        msg.data = grille.flatten().tolist()
        self.pub_vus.publish(msg)


def main():
    rclpy.init()
    noeud = Planificateur()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        # Avec Ctrl+C, ROS 2 s'éteint parfois tout seul AVANT nous :
        # on ne fait le ménage que s'il est encore allumé (évite les erreurs rouges)
        if rclpy.ok():
            noeud.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
