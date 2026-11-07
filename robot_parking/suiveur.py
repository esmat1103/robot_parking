"""
suiveur.py : nœud ROS 2 qui fait rouler le robot le long du chemin.

Écoute :
    /plan  (Path)     : la liste des points à atteindre (chemin lissé)
    /odom  (Odometry) : la position (x, y) et l'orientation du robot
Publie :
    /cmd_vel (Twist)  : la vitesse d'avance v et la vitesse de rotation w

Méthode "tourner puis avancer", 20 fois par seconde :
    1. on vise le prochain point du chemin
    2. si le robot n'est pas tourné vers lui (> 20°) : il pivote sur place
    3. sinon il avance en corrigeant légèrement sa direction
    4. quand il est à moins de 4 cm du point : point suivant
    5. dernier point atteint : arrêt
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def angle_normalise(a):
    """Ramène un angle entre -pi et +pi (ex : 350° -> -10°)."""
    return math.atan2(math.sin(a), math.cos(a))


class Suiveur(Node):

    def __init__(self):
        super().__init__('suiveur')

        # Réglages (modifiables au lancement)
        self.declare_parameter('vitesse_max', 0.15)       # m/s
        self.declare_parameter('rotation_max', 1.5)       # rad/s
        self.declare_parameter('gain_rotation', 2.0)      # réactivité en rotation
        self.declare_parameter('angle_pivot', 0.35)       # rad (20°) : au-delà, on pivote
        self.declare_parameter('tolerance', 0.04)         # m : point intermédiaire "atteint"
        self.declare_parameter('tolerance_finale', 0.015) # m : DERNIER point (précision pour la caméra)

        qos_plan = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, '/plan', self.recevoir_plan, qos_plan)
        self.create_subscription(Odometry, '/odom', self.recevoir_odom, 10)
        self.pub_vitesse = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_arrivee = self.create_publisher(Bool, '/arrivee', 10)   # étape 7

        self.points = []          # [(x, y), ...] à atteindre
        self.indice = 0           # numéro du point visé
        self.pose = None          # (x, y, theta)
        self.cap_final = None     # orientation voulue à l'arrivée (rad)
        self.alignement = False   # True = dernier point atteint, on pivote vers cap_final

        self.create_timer(0.05, self.boucle)              # 20 Hz
        self.get_logger().info('Suiveur prêt : en attente d\'un chemin sur /plan.')

    # ---------- Réception ----------
    def recevoir_plan(self, msg):
        self.points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.indice = 1 if len(self.points) > 1 else 0    # le point 0 = position de départ
        # orientation voulue à l'arrivée (celle du dernier point)
        q = msg.poses[-1].pose.orientation if msg.poses else None
        self.cap_final = None if q is None else math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.alignement = False
        self.get_logger().info(f'Nouveau chemin : {len(self.points)} points à suivre.')

    def recevoir_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # orientation (quaternion) -> angle theta autour de l'axe vertical
        theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, theta)

    # ---------- Boucle de commande ----------
    def boucle(self):
        if self.pose is None or not self.points or self.indice >= len(self.points):
            return

        v_max = self.get_parameter('vitesse_max').value
        w_max = self.get_parameter('rotation_max').value
        k_w = self.get_parameter('gain_rotation').value
        angle_pivot = self.get_parameter('angle_pivot').value
        tolerance = self.get_parameter('tolerance').value

        x, y, theta = self.pose

        # Phase finale : pivoter sur place vers l'orientation demandée
        if self.alignement:
            ecart = angle_normalise(self.cap_final - theta)
            if abs(ecart) < 0.05:                              # ~3° : c'est bon
                self.terminer()
            else:
                commande = Twist()
                commande.angular.z = max(-w_max, min(w_max, k_w * ecart))
                if abs(commande.angular.z) < 0.3:             # éviter de tourner trop lentement
                    commande.angular.z = math.copysign(0.3, ecart)
                self.pub_vitesse.publish(commande)
            return

        cx, cy = self.points[self.indice]
        distance = math.hypot(cx - x, cy - y)
        if self.indice == len(self.points) - 1:
            tolerance = self.get_parameter('tolerance_finale').value

        # Point atteint ?
        if distance < tolerance:
            if self.indice == len(self.points) - 1:            # c'était le dernier
                if self.cap_final is not None:
                    self.alignement = True                     # -> on s'oriente
                    self.arreter()
                else:
                    self.terminer()
            else:
                self.indice += 1
            return

        # Erreur d'angle entre "où je regarde" et "où est le point"
        erreur = angle_normalise(math.atan2(cy - y, cx - x) - theta)

        commande = Twist()
        if abs(erreur) > angle_pivot:
            commande.linear.x = 0.0                            # 1) pivoter sur place
        else:
            commande.linear.x = min(v_max, 1.5 * distance)    # 2) avancer (ralentit près du point)
        commande.angular.z = max(-w_max, min(w_max, k_w * erreur))
        self.pub_vitesse.publish(commande)

    def terminer(self):
        self.arreter()
        self.indice = len(self.points)                         # plus rien à suivre
        self.alignement = False
        self.pub_arrivee.publish(Bool(data=True))              # prévient le nœud mission
        self.get_logger().info('Arrivé à destination !')

    def arreter(self):
        self.pub_vitesse.publish(Twist())                      # v = 0, w = 0


def main():
    rclpy.init()
    noeud = Suiveur()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        # Avec Ctrl+C, ROS 2 s'éteint parfois tout seul AVANT nous :
        # on ne fait le ménage que s'il est encore allumé (évite les erreurs rouges)
        if rclpy.ok():
            noeud.arreter()                 # dernier ordre : vitesse nulle
            noeud.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
