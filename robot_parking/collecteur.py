"""
collecteur.py : enregistre l'image de la caméra devant chaque voiture visitée (vision V1).

Écoute :
    /camera/image_raw (Image)  : la caméra du robot
    /mission_etat     (String) : les annonces de la mission ("Arrivé devant C5, ...")
Quand le robot annonce qu'il est arrivé devant une place, on attend 0,5 s
(image stable) puis on enregistre l'image : ~/parking_ws/dataset/sim/C5.png

Utilisation (pendant que la simulation tourne) :
    ros2 run robot_parking collecteur
    ros2 topic pub --once /mission std_msgs/msg/String "{data: '000 TUN 0000'}"
    (plaque inexistante -> le robot visite les 10 voitures -> 10 images)
"""
import os
import re

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

DOSSIER = os.path.expanduser('~/parking_ws/dataset/sim')


class Collecteur(Node):

    def __init__(self):
        super().__init__('collecteur')
        self.pont = CvBridge()              # convertit message ROS <-> image OpenCV
        self.derniere_image = None
        self.place_a_photographier = None
        self.minuteur = None

        self.create_subscription(Image, '/camera/image_raw', self.recevoir_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/mission_etat', self.recevoir_etat, 10)

        os.makedirs(DOSSIER, exist_ok=True)
        self.get_logger().info(f'Collecteur prêt : les images iront dans {DOSSIER}')

    def recevoir_image(self, msg):
        self.derniere_image = msg

    def recevoir_etat(self, msg):
        trouve = re.search(r'Arrivé devant (\w+)', msg.data)
        if trouve:
            self.place_a_photographier = trouve.group(1)
            if self.minuteur is None:
                self.minuteur = self.create_timer(0.5, self.photographier)

    def photographier(self):
        self.minuteur.cancel()
        self.destroy_timer(self.minuteur)
        self.minuteur = None
        if self.derniere_image is None:
            self.get_logger().warn('Aucune image reçue de la caméra !')
            return
        image = self.pont.imgmsg_to_cv2(self.derniere_image, desired_encoding='bgr8')
        chemin = os.path.join(DOSSIER, f'{self.place_a_photographier}.png')
        cv2.imwrite(chemin, image)
        self.get_logger().info(f'Image enregistrée : {chemin}')


def main():
    rclpy.init()
    noeud = Collecteur()
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
