"""
pieton.py : nœud ROS 2 qui fait marcher un piéton dans le parking Gazebo (étape D1).

Le piéton TRAVERSE une fois de A à B puis disparaît (par défaut : il traverse l'allée du
milieu, en sortant d'entre les places vides B3/B4 vers C3/C4), à 8 cm/s.
Avec repeter:=true : allers-retours sans fin (test plus difficile).
  - il le déplace dans Gazebo  (service /set_entity_state)
  - il publie ce que « verrait » la vision sur /pieton : [présent, x, y, vx, vy]
    -> utile plus tard pour DWA (prédire où va le piéton)

Par défaut, il attend que le robot démarre (1er /cmd_vel non nul) + 'attente' secondes.

Utilisation (simulation lancée) :
    ros2 run robot_parking pieton
    ros2 run robot_parking pieton --ros-args -p attente:=2.0 -p vitesse:=0.06
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import SetEntityState

DT = 0.05            # 20 mises à jour par seconde
PAUSE = 2.0          # s : pause à chaque bout (le piéton « hésite »)


class Pieton(Node):

    def __init__(self):
        super().__init__('pieton')
        self.declare_parameter('a', [0.90, 1.00])        # départ (m)
        self.declare_parameter('b', [0.90, 1.46])        # arrivée (m)
        self.declare_parameter('vitesse', 0.08)          # m/s (≈ 2 m/s à l'échelle 1:24)
        self.declare_parameter('attente', 4.0)           # s après le départ du robot
        self.declare_parameter('attendre_robot', True)   # False = commence tout de suite
        self.declare_parameter('repeter', False)         # True = allers-retours sans fin
        self.declare_parameter('arret_milieu', 0.0)      # s : le piéton s'arrête au milieu de l'allée

        self.a = tuple(self.get_parameter('a').value)
        self.b = tuple(self.get_parameter('b').value)
        self.vitesse = self.get_parameter('vitesse').value
        self.longueur = math.dist(self.a, self.b)

        self.chrono = None                               # None = pas encore démarré
        if not self.get_parameter('attendre_robot').value:
            self.chrono = -self.get_parameter('attente').value

        self.pub = self.create_publisher(Float32MultiArray, '/pieton', 10)
        self.create_subscription(Twist, '/cmd_vel', self.recevoir_cmd, 10)
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /set_entity_state introuvable : as-tu régénéré '
                                    'le monde (generer_monde) et relancé la simulation ?')
        self.create_timer(DT, self.boucle)
        mode = 'allers-retours' if self.get_parameter('repeter').value else 'une traversée'
        self.get_logger().info(f'Piéton prêt ({mode}) : {self.a} -> {self.b} '
                               f'à {self.vitesse * 100:.0f} cm/s')

    def recevoir_cmd(self, msg):
        if self.chrono is None and (abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3):
            self.chrono = -self.get_parameter('attente').value
            self.get_logger().info('Le robot démarre : le piéton arrive bientôt')

    def etat(self):
        """Position et vitesse du piéton selon le chrono (allers-retours avec pauses)."""
        if self.chrono is None or self.chrono < 0:
            return False, self.a, (0.0, 0.0)
        duree_traversee = self.longueur / self.vitesse
        ux = (self.b[0] - self.a[0]) / self.longueur
        uy = (self.b[1] - self.a[1]) / self.longueur

        if not self.get_parameter('repeter').value:
            # une traversée, avec éventuellement un ARRÊT au milieu de l'allée
            arret = self.get_parameter('arret_milieu').value
            t_milieu = duree_traversee / 2
            t = self.chrono
            if t < t_milieu:                                     # 1re moitié
                d, sens = t * self.vitesse, 1
            elif t < t_milieu + arret:                           # immobile au milieu
                d, sens = self.longueur / 2, 0
            elif t < duree_traversee + arret:                    # 2e moitié
                d, sens = (t - arret) * self.vitesse, 1
            else:
                return False, self.b, (0.0, 0.0)                 # traversée finie : il est parti
            x, y = self.a[0] + d * ux, self.a[1] + d * uy
            return True, (x, y), (sens * self.vitesse * ux, sens * self.vitesse * uy)

        cycle = 2 * (duree_traversee + PAUSE)
        t = self.chrono % cycle
        if t < duree_traversee:                                  # aller A -> B
            d, sens = t * self.vitesse, 1
        elif t < duree_traversee + PAUSE:                        # pause en B
            d, sens = self.longueur, 0
        elif t < 2 * duree_traversee + PAUSE:                    # retour B -> A
            d, sens = self.longueur - (t - duree_traversee - PAUSE) * self.vitesse, -1
        else:                                                    # pause en A
            d, sens = 0.0, 0
        x, y = self.a[0] + d * ux, self.a[1] + d * uy
        return True, (x, y), (sens * self.vitesse * ux, sens * self.vitesse * uy)

    def boucle(self):
        if self.chrono is not None:
            self.chrono += DT
        avant = getattr(self, 'present', False)
        present, (x, y), (vx, vy) = self.etat()
        if present != avant:
            self.get_logger().info('Le piéton SURGIT' if present else 'Le piéton est parti')
        immobile = present and vx == 0.0 and vy == 0.0
        if immobile != getattr(self, 'immobile', False):
            self.get_logger().info("Le piéton s'ARRÊTE au milieu" if immobile else 'Le piéton REPART')
        self.immobile = immobile
        self.present = present
        self.pub.publish(Float32MultiArray(data=[float(present), x, y, vx, vy]))

        req = SetEntityState.Request()
        req.state.name = 'pieton'
        req.state.reference_frame = 'world'
        req.state.pose.position.x = x
        req.state.pose.position.y = y                            # même repère que la carte
        req.state.pose.position.z = 0.05 if present else -1.0    # caché sous le sol si absent
        req.state.pose.orientation.w = 1.0
        if self.client.service_is_ready():
            self.client.call_async(req)


def main():
    rclpy.init()
    noeud = Pieton()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            noeud.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
