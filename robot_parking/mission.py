"""
mission.py : le "chef d'orchestre" de la mission (étape 7).

On lui donne, sur le topic /mission (texte) :
    - un NOM DE PLACE, ex : "C5"            -> le robot va devant C5 et lit la plaque
    - une PLAQUE,      ex : "199 TUN 8125"  -> le robot visite les voitures une par une
                                              jusqu'à trouver cette plaque

Il utilise les nœuds déjà faits :
    /goal_pose  -> planificateur (A*) -> /plan -> suiveur -> sécurité -> moteurs
    /arrivee    <- suiveur (le robot est arrivé)
Il publie :
    /mission_etat (texte) : où en est la mission (pour l'affichage / les autres membres)

Envoyer une mission :
    ros2 topic pub --once /mission std_msgs/msg/String "{data: 'C5'}"
    ros2 topic pub --once /mission std_msgs/msg/String "{data: '199 TUN 8125'}"
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robot_parking.carte import PLAQUES, PLACES_OCCUPEES, calculer_places, point_approche
from robot_parking.lecture_plaque import lire_plaque_image
from robot_parking.vision_yolo import detecter_voiture, charger_modele

DUREE_LECTURE = 1.0   # s : on attend que l'image soit stable avant de lire
ESSAIS_MAX = 3        # nombre d'images essayées si la lecture échoue


def lire_plaque_simulee(place):
    """Ancienne lecture SIMULÉE (tableau PLAQUES) : sert maintenant de VÉRITÉ pour contrôler."""
    return PLAQUES.get(place)


class Mission(Node):

    def __init__(self):
        super().__init__('mission')
        self.create_subscription(String, '/mission', self.recevoir_mission, 10)
        self.create_subscription(Bool, '/arrivee', self.recevoir_arrivee, 10)
        self.create_subscription(Odometry, '/odom', self.recevoir_odom, 10)
        self.pub_but = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_etat = self.create_publisher(String, '/mission_etat', 10)

        # Vision V3 : 'camera' = vraie lecture ; 'simulee' = ancien tableau (pour comparer)
        self.declare_parameter('lecture', 'camera')
        self.create_subscription(Image, '/camera/image_raw', self.recevoir_image,
                                 qos_profile_sensor_data)
        self.pont = CvBridge()
        self.derniere_image = None
        self.essais = 0
        charger_modele()             # YOLOv8n chargé une fois au démarrage (~1 s)

        self.places = calculer_places()
        self.position = None
        self.etat = 'ATTENTE'        # ATTENTE -> EN_ROUTE -> LECTURE -> ... -> FINI
        self.a_visiter = []          # places restant à visiter
        self.place_actuelle = None
        self.plaque_cherchee = None  # None = mission "aller à une place"
        self.debut_lecture = None

        self.create_timer(0.1, self.boucle)
        self.get_logger().info('Mission prête. Exemple : '
                               'ros2 topic pub --once /mission std_msgs/msg/String "{data: \'C5\'}"')

    def annoncer(self, texte):
        self.get_logger().info(texte)
        self.pub_etat.publish(String(data=texte))

    # ---------- Réception ----------
    def recevoir_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)

    def recevoir_image(self, msg):
        self.derniere_image = msg                        # on garde seulement la plus récente

    def lire_plaque(self):
        """Lecture de la plaque devant le robot : par la CAMÉRA (vision V2)
        ou, si le paramètre lecture = 'simulee', par l'ancien tableau."""
        if self.get_parameter('lecture').value == 'simulee':
            return lire_plaque_simulee(self.place_actuelle)
        if self.derniere_image is None:
            self.get_logger().warn('Aucune image de la caméra !')
            return None
        image = self.pont.imgmsg_to_cv2(self.derniere_image, desired_encoding='bgr8')

        # V4 : y a-t-il un véhicule devant le robot ? (YOLOv8s)
        vehicule, _ = detecter_voiture(image)
        # V2 : lecture de la plaque
        texte, _ = lire_plaque_image(image)

        if vehicule is not None:
            self.get_logger().info(f"YOLO : {vehicule['classe']} détecté "
                                   f"(confiance {vehicule['confiance']:.2f})")
        elif texte is not None:
            # sécurité : YOLO n'a rien vu, mais une plaque est lisible devant le robot
            # -> il y a forcément une voiture (cas de B6 dans nos tests)
            self.get_logger().info('YOLO : rien vu, mais une plaque est lue -> place occupée')
        else:
            self.get_logger().info(f'YOLO : aucun véhicule devant {self.place_actuelle}')
            return 'VIDE'
        return texte

    def recevoir_mission(self, msg):
        demande = msg.data.strip().upper()
        if demande in self.places:                       # c'est un nom de place
            self.plaque_cherchee = None
            self.a_visiter = [demande]
            self.annoncer(f'Mission : aller à la place {demande}')
        else:                                            # c'est une plaque
            self.plaque_cherchee = demande
            self.a_visiter = list(PLACES_OCCUPEES)
            self.annoncer(f'Mission : trouver la voiture {demande} '
                          f'({len(self.a_visiter)} voitures à vérifier)')
        self.aller_place_suivante()

    def recevoir_arrivee(self, msg):
        if self.etat == 'EN_ROUTE' and msg.data:
            self.etat = 'LECTURE'
            self.debut_lecture = self.get_clock().now()
            self.essais = 0
            self.annoncer(f'Arrivé devant {self.place_actuelle}, lecture de la plaque...')

    # ---------- Déroulement ----------
    def aller_place_suivante(self):
        if not self.a_visiter:
            self.etat = 'FINI'
            self.annoncer(f'Échec : voiture {self.plaque_cherchee} introuvable dans le parking.')
            return
        # on choisit la place la plus proche du robot (tournée plus courte)
        if self.position is not None:
            self.a_visiter.sort(key=lambda n: math.dist(self.position, point_approche(n)))
        self.place_actuelle = self.a_visiter.pop(0)
        x, y = point_approche(self.place_actuelle)

        but = PoseStamped()
        but.header.frame_id = 'map'
        but.header.stamp = self.get_clock().now().to_msg()
        but.pose.position.x, but.pose.position.y = x, y
        # à l'arrivée, le robot se tourne FACE à la place (pour que la caméra voie la plaque)
        # rangées A et C : place au-dessus du robot (+90°) ; B et D : en dessous (-90°)
        cap = math.pi / 2 if self.place_actuelle[0] in "AC" else -math.pi / 2
        but.pose.orientation.z = math.sin(cap / 2)
        but.pose.orientation.w = math.cos(cap / 2)
        self.pub_but.publish(but)

        self.etat = 'EN_ROUTE'
        self.annoncer(f'En route vers la place {self.place_actuelle} ({x:.2f}, {y:.2f})')

    def boucle(self):
        if self.etat != 'LECTURE':
            return
        if (self.get_clock().now() - self.debut_lecture).nanoseconds * 1e-9 < DUREE_LECTURE:
            return                                       # on attend une image stable

        plaque = self.lire_plaque()
        if plaque is None and self.essais < ESSAIS_MAX - 1:
            self.essais += 1                             # lecture ratée : on réessaie
            self.debut_lecture = self.get_clock().now()  # sur une nouvelle image
            self.get_logger().info(f'Plaque illisible, nouvel essai ({self.essais + 1}/{ESSAIS_MAX})')
            return

        # contrôle : on compare avec la vérité (tableau PLAQUES ; pas d'entrée = place vide)
        verite = lire_plaque_simulee(self.place_actuelle) or 'VIDE'
        controle = '✅' if plaque == verite else f'❌ (vraie : {verite})'
        if plaque == 'VIDE':
            self.annoncer(f'Place {self.place_actuelle} : vide (aucun véhicule vu par YOLO)  {controle}')
        else:
            self.annoncer(f'Place {self.place_actuelle} : caméra lit {plaque or "rien"}  {controle}')

        if self.plaque_cherchee is None:                 # mission "aller à une place"
            self.etat = 'FINI'
            self.annoncer(f'Mission terminée devant {self.place_actuelle}.')
        elif plaque == self.plaque_cherchee:             # trouvée !
            self.etat = 'FINI'
            self.annoncer(f'VÉHICULE TROUVÉ : {plaque} en place {self.place_actuelle} !')
        else:                                            # pas la bonne : suivante
            self.aller_place_suivante()


def main():
    rclpy.init()
    noeud = Mission()
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
