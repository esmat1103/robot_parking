"""
securite.py : nœud "garde du corps" placé entre le suiveur et les moteurs.

    suiveur --/cmd_vel_brut--> [ securite ] --/cmd_vel--> robot
                                    ^
                                  /scan (capteur de distance)

Règles (vérifiées à chaque ordre reçu) :
    - obstacle devant à moins de 'distance_arret'  -> on bloque la marche avant
    - obstacle entre 'distance_arret' et 'distance_ralenti' -> on ralentit
    - pas de mesure du capteur depuis 0,5 s -> on s'arrête (capteur en panne ?)
    - tourner sur place et reculer restent permis (pour pouvoir se dégager)

Pourquoi ? La carte ne connaît que les obstacles FIXES. Un obstacle imprévu
(une personne, un carton, une voiture mal garée) n'est vu que par le capteur.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Securite(Node):

    def __init__(self):
        super().__init__('securite')
        self.declare_parameter('distance_arret', 0.12)     # m (mesurée depuis le capteur)
        self.declare_parameter('distance_ralenti', 0.30)   # m
        self.declare_parameter('delai_capteur', 0.5)       # s sans mesure -> arrêt

        self.create_subscription(LaserScan, '/scan', self.recevoir_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_brut', self.recevoir_ordre, 10)
        self.pub_vitesse = self.create_publisher(Twist, '/cmd_vel', 10)

        self.distance_devant = math.inf   # plus petite distance mesurée
        self.heure_scan = None            # heure de la dernière mesure
        self.etat = 'OK'

        self.get_logger().info('Sécurité active : /cmd_vel_brut -> /cmd_vel')

    def recevoir_scan(self, msg):
        valides = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        self.distance_devant = min(valides) if valides else math.inf
        self.heure_scan = self.get_clock().now()

    def recevoir_ordre(self, ordre):
        arret = self.get_parameter('distance_arret').value
        ralenti = self.get_parameter('distance_ralenti').value
        delai = self.get_parameter('delai_capteur').value

        sortie = Twist()
        sortie.angular.z = ordre.angular.z            # tourner reste toujours permis
        v = ordre.linear.x

        # Capteur muet ?
        maintenant = self.get_clock().now()
        if self.heure_scan is None or (maintenant - self.heure_scan).nanoseconds * 1e-9 > delai:
            nouvel_etat, v = 'CAPTEUR MUET', 0.0
        elif v > 0 and self.distance_devant < arret:
            nouvel_etat, v = 'ARRÊT', 0.0
        elif v > 0 and self.distance_devant < ralenti:
            facteur = (self.distance_devant - arret) / (ralenti - arret)   # 0 -> 1
            nouvel_etat, v = 'RALENTI', v * max(0.2, facteur)
        else:
            nouvel_etat = 'OK'

        if nouvel_etat != self.etat:                  # on n'affiche que les changements
            self.get_logger().info(
                f'Sécurité : {self.etat} -> {nouvel_etat} (obstacle à {self.distance_devant:.2f} m)')
            self.etat = nouvel_etat

        sortie.linear.x = v
        self.pub_vitesse.publish(sortie)


def main():
    rclpy.init()
    noeud = Securite()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        # Avec Ctrl+C, ROS 2 s'éteint parfois tout seul AVANT nous :
        # on ne fait le ménage que s'il est encore allumé (évite les erreurs rouges)
        if rclpy.ok():
            noeud.pub_vitesse.publish(Twist())   # dernier ordre : vitesse nulle
            noeud.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
