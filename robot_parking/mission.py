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

from robot_parking.carte import PLAQUES, PLACES_OCCUPEES, calculer_places, point_approche

DUREE_LECTURE = 1.5   # s : temps pour "lire" la plaque une fois arrivé


def lire_plaque(place):
    """SIMULATION de la vision : renvoie la plaque de la voiture garée sur 'place'.
    -> À REMPLACER par le résultat de la caméra + YOLO (membre vision)."""
    return PLAQUES.get(place)


class Mission(Node):

    def __init__(self):
        super().__init__('mission')
        self.create_subscription(String, '/mission', self.recevoir_mission, 10)
        self.create_subscription(Bool, '/arrivee', self.recevoir_arrivee, 10)
        self.create_subscription(Odometry, '/odom', self.recevoir_odom, 10)
        self.pub_but = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_etat = self.create_publisher(String, '/mission_etat', 10)

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
            return                                       # on "lit" encore

        plaque = lire_plaque(self.place_actuelle)
        lu = plaque if plaque else 'place vide'
        self.annoncer(f'Place {self.place_actuelle} : {lu}')

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
    noeud.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
