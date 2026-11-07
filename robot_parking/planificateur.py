"""
planificateur.py : nœud ROS 2 qui calcule le chemin avec A*.

Écoute :
    /map        (OccupancyGrid) : la carte du parking
    /odom       (Odometry)      : la position actuelle du robot
    /goal_pose  (PoseStamped)   : la destination (outil "2D Goal Pose" de RViz)
Publie :
    /plan_brut       (Path)          : le chemin A* brut (case par case)
    /plan            (Path)          : le chemin lissé, à suivre (dessiné dans RViz)
    /carte_securite  (OccupancyGrid) : les obstacles "gonflés" (zone interdite)

Utilisation (déjà inclus dans simulation.launch.py) :
    ros2 run robot_parking planificateur
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped

from robot_parking.astar import gonfler, astar, case_libre_proche, longueur, simplifier


class Planificateur(Node):

    def __init__(self):
        super().__init__('planificateur')

        # Réglage : demi-diagonale du robot (12 cm) + marge (3 cm)
        self.declare_parameter('rayon_securite', 0.15)

        # QoS "Transient Local" : on reçoit la carte même si elle a été publiée avant
        qos_carte = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, '/map', self.recevoir_carte, qos_carte)
        self.create_subscription(Odometry, '/odom', self.recevoir_odom, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.recevoir_but, 10)

        self.pub_plan = self.create_publisher(Path, '/plan', qos_carte)            # chemin lissé
        self.pub_brut = self.create_publisher(Path, '/plan_brut', qos_carte)       # chemin A* brut
        self.pub_securite = self.create_publisher(OccupancyGrid, '/carte_securite', qos_carte)

        self.carte = None        # message /map reçu
        self.bloque = None       # grille True/False des cases interdites
        self.position = None     # (x, y) du robot en mètres

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
        rayon_cases = math.ceil(rayon / msg.info.resolution)
        self.bloque = gonfler(obstacles, rayon_cases)

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
            f'obstacles gonflés de {rayon_cases} cases ({rayon_cases * msg.info.resolution * 100:.0f} cm)')

    def recevoir_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)

    def recevoir_but(self, msg):
        if self.carte is None or self.position is None:
            self.get_logger().warn('Pas encore de carte ou de position du robot.')
            return

        depart = self.vers_case(*self.position)
        but = self.vers_case(msg.pose.position.x, msg.pose.position.y)
        if not self.dans_carte(depart) or not self.dans_carte(but):
            self.get_logger().warn('Départ ou destination hors de la carte.')
            return

        # Si on clique dans une zone interdite, on prend la case libre la plus proche
        depart = case_libre_proche(self.bloque, depart)
        but = case_libre_proche(self.bloque, but)
        if depart is None or but is None:
            self.get_logger().warn('Aucune case libre près du départ ou de la destination.')
            return

        t0 = time.perf_counter()
        chemin = astar(self.bloque, depart, but)
        duree_ms = (time.perf_counter() - t0) * 1000

        if chemin is None:
            self.get_logger().warn('Aucun chemin trouvé !')
            return

        lisse = simplifier(self.bloque, chemin)
        # l'orientation demandée (flèche de RViz, ou "face à la place" pour la mission)
        # est recopiée sur le DERNIER point : le suiveur s'alignera dessus à l'arrivée
        self.publier_chemin(self.pub_brut, chemin, msg.pose.orientation)
        self.publier_chemin(self.pub_plan, lisse, msg.pose.orientation)
        res = self.carte.info.resolution
        self.get_logger().info(
            f'Chemin trouvé : {len(chemin)} cases, {longueur(chemin, res):.2f} m, '
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


def main():
    rclpy.init()
    noeud = Planificateur()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    noeud.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
