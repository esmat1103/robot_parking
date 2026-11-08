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

Re-planification (D2) :
    1. chaque point vu par le capteur, qui n'est PAS un obstacle connu de la carte,
       est noté comme « obstacle imprévu » (et oublié s'il n'est plus vu depuis 3 s :
       un piéton qui passe ne bloque pas la case pour toujours)
    2. deux fois par seconde : si le chemin RESTANT traverse un obstacle imprévu (gonflé),
       A* recalcule un chemin qui le contourne
    3. si aucun contournement n'existe, on garde le chemin : la sécurité fait attendre le robot
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
from std_msgs.msg import Bool

from robot_parking.astar import gonfler, astar, case_libre_proche, longueur, simplifier

POSITION_LASER = 0.09      # m : le capteur est 9 cm devant le centre du robot (robot.urdf)
PORTEE_DETECTION = 0.60    # m : on ne tient compte que des obstacles proches
MEMOIRE = 3.0              # s : un obstacle non revu depuis 3 s est oublié
DELAI_REPLANIF = 1.5       # s : au plus une re-planification toutes les 1,5 s


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
        self.vus = {}              # obstacles imprévus : {case: heure de la dernière détection}
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
        self.planifier('nouvelle destination')

    def recevoir_arrivee(self, msg):
        if msg.data:
            self.but = None          # mission accomplie : plus de surveillance
            self.chemin = None

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
            case = self.vers_case(lx + r * math.cos(a), ly + r * math.sin(a))
            if self.dans_carte(case) and not self.connu[case]:
                self.vus[case] = maintenant

    # ---------- Re-planification (D2) ----------
    def obstacles_imprevus(self):
        """Grille True/False des obstacles imprévus encore « frais » (vus < 3 s)."""
        maintenant = time.monotonic()
        self.vus = {c: t for c, t in self.vus.items() if maintenant - t < MEMOIRE}
        grille = np.zeros_like(self.bloque)
        for case in self.vus:
            grille[case] = True
        return grille

    def surveiller(self):
        """2 fois par seconde : le chemin restant est-il coupé par un obstacle imprévu ?"""
        if self.carte is None:
            return
        imprevus = self.obstacles_imprevus()
        imprevus_gonfles = gonfler(imprevus, self.rayon_cases) if imprevus.any() else imprevus
        self.publier_obstacles_vus(imprevus, imprevus_gonfles)

        if (not self.get_parameter('replanification').value or self.but is None
                or self.chemin is None or self.position is None or not imprevus.any()):
            return
        # le chemin RESTANT = à partir du point du chemin le plus proche du robot
        ici = self.vers_case(*self.position)
        k = min(range(len(self.chemin)),
                key=lambda i: (self.chemin[i][0] - ici[0]) ** 2 + (self.chemin[i][1] - ici[1]) ** 2)
        coupe = any(imprevus_gonfles[c] for c in self.chemin[k:])
        if coupe and time.monotonic() - self.derniere_replanif > DELAI_REPLANIF:
            self.planifier('obstacle imprévu sur le chemin')

    def planifier(self, raison):
        """Calcule (ou recalcule) le chemin vers self.but, en évitant les obstacles
        fixes ET les obstacles imprévus vus par le capteur."""
        if self.carte is None or self.position is None or self.but is None:
            self.get_logger().warn('Pas encore de carte ou de position du robot.')
            return
        self.derniere_replanif = time.monotonic()

        imprevus = self.obstacles_imprevus()
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
