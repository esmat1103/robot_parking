"""
evaluer.py : évaluation AUTOMATIQUE d'un conducteur (étape D5).

À lancer pendant que la simulation tourne avec le conducteur à tester, SANS le nœud pieton :
    ros2 launch robot_parking simulation.launch.py controleur:=dwa vitesse:=0.20
    ros2 run robot_parking evaluer --ros-args -p controleur:=dwa -p use_sim_time:=true

Pour chaque scénario (destination x piéton oui/non) :
    1. téléporte le robot à l'entrée, cache le piéton, attend 1,5 s
    2. envoie la destination (point d'approche de la place, face à la place) sur /goal_pose
    3. scénario « piéton » : quand le robot arrive à 55 cm du passage piéton, le piéton
       traverse l'allée (8 cm/s) juste devant lui ; il est publié sur /pieton comme d'habitude
    4. mesure : réussite (/arrivee en moins de 60 s), temps, distance parcourue,
       collision voiture/mur (contour du robot dans un obstacle de la carte),
       collision piéton, distance minimale au piéton
    5. écrit une ligne dans ~/parking_ws/resultats/eval_<controleur>.csv
"""
import csv
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32MultiArray
from gazebo_msgs.srv import SetEntityState

from robot_parking.carte import ENTREE, point_approche, LARG_ALLEE

DESTINATIONS = ['A4', 'B2', 'B6', 'C1', 'C5', 'D6']
DELAI_MAX = 60.0                 # s : au-delà, l'essai est un échec
VITESSE_PIETON = 0.08            # m/s
DECLENCHEMENT = 0.55             # m : le piéton part quand le robot est à 55 cm du passage
DT = 0.05
DEMI_LONG, DEMI_LARG, RAYON_PIETON = 0.10, 0.07, 0.04
DOSSIER = os.path.expanduser('~/parking_ws/resultats')


class Evaluateur(Node):

    def __init__(self):
        super().__init__('evaluer')
        self.declare_parameter('controleur', 'inconnu')
        self.controleur = self.get_parameter('controleur').value

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.recevoir_carte, qos)
        self.create_subscription(Odometry, '/odom', self.recevoir_odom, 10)
        self.create_subscription(Bool, '/arrivee', self.recevoir_arrivee, 10)
        self.pub_but = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_pieton = self.create_publisher(Float32MultiArray, '/pieton', 10)
        self.client = self.create_client(SetEntityState, '/set_entity_state')

        self.scenarios = [(p, avec) for avec in (False, True) for p in DESTINATIONS]
        self.k = -1                      # numéro du scénario en cours
        self.etat = 'DEBUT'
        self.obstacles = None
        self.pose = None
        self.arrive = False
        self.resultats = []

        os.makedirs(DOSSIER, exist_ok=True)
        self.fichier = os.path.join(DOSSIER, f'eval_{self.controleur}.csv')
        self.create_timer(DT, self.boucle)
        self.get_logger().info(f'Évaluation du conducteur « {self.controleur} » : '
                               f'{len(self.scenarios)} scénarios -> {self.fichier}')

    # ---------- Réception ----------
    def recevoir_carte(self, msg):
        self.info = msg.info
        valeurs = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.obstacles = (valeurs >= 50) | (valeurs < 0)

    def recevoir_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.pose is not None and self.etat == 'EN_COURS':
            self.distance += math.dist(self.pose[:2], (p.x, p.y))
        self.pose = (p.x, p.y, th)

    def recevoir_arrivee(self, msg):
        if msg.data and self.etat == 'EN_COURS':
            self.arrive = True

    # ---------- Outils ----------
    def maintenant(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def teleporter(self, nom, x, y, z, yaw=0.0):
        req = SetEntityState.Request()
        req.state.name = nom
        req.state.reference_frame = 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
        req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if self.client.service_is_ready():
            self.client.call_async(req)

    def collision_fixe(self):
        """Un point du contour du robot est-il dans une voiture ou un mur (carte NON gonflée) ?"""
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        for lx in (-DEMI_LONG, 0.0, DEMI_LONG):
            for ly in (-DEMI_LARG, 0.0, DEMI_LARG):
                px, py = x + lx * c - ly * s, y + lx * s + ly * c
                col = int((px - self.info.origin.position.x) / self.info.resolution)
                lig = int((py - self.info.origin.position.y) / self.info.resolution)
                if not (0 <= lig < self.obstacles.shape[0] and 0 <= col < self.obstacles.shape[1]) \
                        or self.obstacles[lig, col]:
                    return True
        return False

    def distance_pieton(self, px, py):
        """Distance entre le BORD du robot (rectangle) et le centre du piéton."""
        x, y, th = self.pose
        dx, dy = px - x, py - y
        lx = dx * math.cos(th) + dy * math.sin(th)
        ly = -dx * math.sin(th) + dy * math.cos(th)
        qx = max(-DEMI_LONG, min(DEMI_LONG, lx))
        qy = max(-DEMI_LARG, min(DEMI_LARG, ly))
        return math.hypot(lx - qx, ly - qy)

    # ---------- Piéton ----------
    def preparer_pieton(self, place):
        """Le piéton traverse l'allée d'approche, à mi-chemin entre l'allée de gauche et la place."""
        xa, ya = point_approche(place)
        xc = (ENTREE[0] + xa) / 2
        self.passage = (xc, ya)
        self.pieton_a = (xc, ya - LARG_ALLEE / 2 + 0.03)
        self.pieton_b = (xc, ya + LARG_ALLEE / 2 - 0.03)
        self.pieton_t0 = None            # heure de départ (None = pas encore parti)

    def bouger_pieton(self, actif):
        present, x, y, vx, vy = False, -1.0, -1.0, 0.0, 0.0
        if actif:
            if self.pieton_t0 is None and math.dist(self.pose[:2], self.passage) < DECLENCHEMENT:
                self.pieton_t0 = self.maintenant()
            if self.pieton_t0 is not None:
                longueur = math.dist(self.pieton_a, self.pieton_b)
                d = (self.maintenant() - self.pieton_t0) * VITESSE_PIETON
                if d <= longueur:
                    f = d / longueur
                    x = self.pieton_a[0] + f * (self.pieton_b[0] - self.pieton_a[0])
                    y = self.pieton_a[1] + f * (self.pieton_b[1] - self.pieton_a[1])
                    vx = VITESSE_PIETON * (self.pieton_b[0] - self.pieton_a[0]) / longueur
                    vy = VITESSE_PIETON * (self.pieton_b[1] - self.pieton_a[1]) / longueur
                    present = True
        self.pub_pieton.publish(Float32MultiArray(data=[float(present), x, y, vx, vy]))
        self.teleporter('pieton', x if present else -1.0, y if present else -1.0,
                        0.05 if present else -1.0)
        return present, x, y

    # ---------- Déroulement ----------
    def boucle(self):
        if self.obstacles is None or self.pose is None or not self.client.service_is_ready():
            return
        t = self.maintenant()

        if self.etat in ('DEBUT', 'SUIVANT'):
            self.k += 1
            if self.k >= len(self.scenarios):
                self.finir()
                return
            place, avec = self.scenarios[self.k]
            self.get_logger().info(f'--- Scénario {self.k + 1}/{len(self.scenarios)} : {place}, '
                                   f'{"AVEC" if avec else "sans"} piéton ---')
            self.teleporter('robot', ENTREE[0], ENTREE[1], 0.005)
            self.preparer_pieton(place)
            self.bouger_pieton(False)
            self.etat, self.t_etat = 'PREPARATION', t
            return

        if self.etat == 'PREPARATION':
            self.bouger_pieton(False)
            if t - self.t_etat < 1.5:
                return
            place, _ = self.scenarios[self.k]
            xa, ya = point_approche(place)
            cap = math.pi / 2 if place[0] in 'AC' else -math.pi / 2
            but = PoseStamped()
            but.header.frame_id = 'map'
            but.header.stamp = self.get_clock().now().to_msg()
            but.pose.position.x, but.pose.position.y = xa, ya
            but.pose.orientation.z, but.pose.orientation.w = math.sin(cap / 2), math.cos(cap / 2)
            self.pub_but.publish(but)
            self.arrive, self.distance = False, 0.0
            self.coll_fixe = self.coll_pieton = False
            self.dist_min_pieton = math.inf
            self.etat, self.t_depart = 'EN_COURS', t
            return

        if self.etat == 'EN_COURS':
            place, avec = self.scenarios[self.k]
            present, px, py = self.bouger_pieton(avec)
            if self.collision_fixe():
                self.coll_fixe = True
            if present:
                d = self.distance_pieton(px, py)
                self.dist_min_pieton = min(self.dist_min_pieton, d)
                if d < RAYON_PIETON:
                    self.coll_pieton = True
            duree = t - self.t_depart
            if self.arrive or duree > DELAI_MAX:
                reussi = self.arrive and not self.coll_fixe and not self.coll_pieton
                ligne = {
                    'controleur': self.controleur, 'place': place, 'pieton': int(avec),
                    'reussi': int(reussi), 'arrive': int(self.arrive), 'temps_s': round(duree, 2),
                    'distance_m': round(self.distance, 3),
                    'collision_fixe': int(self.coll_fixe), 'collision_pieton': int(self.coll_pieton),
                    'piéton_vu': int(avec and self.pieton_t0 is not None),
                    'dist_min_pieton_m': round(self.dist_min_pieton, 3)
                    if self.dist_min_pieton < math.inf else '',
                }
                self.resultats.append(ligne)
                self.get_logger().info(
                    f"{place} {'avec' if avec else 'sans'} piéton : "
                    f"{'RÉUSSI' if reussi else 'ÉCHEC'} en {duree:.1f} s, {self.distance:.2f} m"
                    + (f", piéton au plus près à {self.dist_min_pieton * 100:.0f} cm"
                       if self.dist_min_pieton < math.inf else '')
                    + (' [COLLISION voiture/mur]' if self.coll_fixe else '')
                    + (' [COLLISION piéton]' if self.coll_pieton else ''))
                self.etat = 'SUIVANT'

    def finir(self):
        with open(self.fichier, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.resultats[0].keys()))
            w.writeheader()
            w.writerows(self.resultats)
        ok = sum(r['reussi'] for r in self.resultats)
        self.get_logger().info(f'=== FIN : {ok}/{len(self.resultats)} réussis -> {self.fichier} ===')
        self.etat = 'FINI'
        raise SystemExit


def main():
    rclpy.init()
    noeud = Evaluateur()
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            noeud.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
