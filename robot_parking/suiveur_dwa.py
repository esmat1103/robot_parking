"""
suiveur_dwa.py : conducteur DWA (Dynamic Window Approach), étape D3.

Même rôle et même interface que suiveur.py (on choisit l'un OU l'autre au lancement) :
Écoute :
    /plan_brut (Path)              : le chemin A* (case par case) et l'orientation finale
    /odom      (Odometry)          : position et orientation du robot
    /map       (OccupancyGrid)     : les obstacles fixes
    /pieton    (Float32MultiArray) : [présent, x, y, vx, vy] du piéton
Publie :
    /cmd_vel   (Twist) : (remappé en /cmd_vel_brut -> passe par la sécurité)
    /arrivee   (Bool)  : le robot est arrivé (pour la mission)

DWA, 10 fois par seconde :
    1. FENÊTRE : les vitesses (v, w) atteignables en 0,1 s (accélérations limitées)
    2. IMAGINER : pour 6 x 11 couples (v, w), la trajectoire des 1,5 s suivantes
    3. REJETER : celles qui touchent un obstacle fixe ou passent trop près de la position
       FUTURE du piéton (prédite avec sa vitesse ; marge qui grossit avec le temps)
    4. NOTER : proche de la cible + loin du piéton + rapide -> on garde la meilleure
Ajouts : mode précis « tourner puis avancer » à moins de 10 cm du but (lecture de plaque),
         et déblocage (pivot) si le robot n'avance plus pendant 3 s.

D'après navigation/dwa.py de l'ancien projet.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray

from robot_parking.astar import gonfler

DT = 0.1                 # s : 10 décisions par seconde

# Robot
V_MAX, V_RECUL, W_MAX = 0.15, 0.05, 1.5     # m/s, m/s, rad/s (comme suiveur.py)
A_MAX, ALPHA_MAX = 1.0, 6.0                  # accélérations max (m/s², rad/s²)

# DWA (valeurs de l'ancien projet)
HORIZON, PAS_SIM = 1.5, 0.1                  # on imagine 1,5 s d'avenir, par pas de 0,1 s
RAYON_ROBOT, RAYON_PIETON = 0.12, 0.04
DIST_SECU = RAYON_ROBOT + RAYON_PIETON + 0.04     # 20 cm entre les centres, au minimum
INCERTITUDE = 0.05                            # m/s : la bulle du piéton grossit avec le temps
P_BUT, P_DEGAGEMENT, P_VITESSE = 1.0, 0.4, 0.3    # poids des critères

# Suivi du chemin
VISEE = 0.25             # m : la cible de DWA = point du chemin 25 cm devant le robot
DIST_FINALE = 0.10       # m : à moins de 10 cm du but -> mode précis
TOLERANCE_FINALE = 0.015 # m
MARGE, MARGE_SECOURS = 0.15, 0.08            # gonflement des obstacles (normal / secours)


def angle_normalise(a):
    return math.atan2(math.sin(a), math.cos(a))


def imaginer(x, y, th, v, w):
    """Trajectoire future si on garde (v, w) constants pendant HORIZON secondes."""
    traj = []
    for _ in range(int(HORIZON / PAS_SIM)):
        x += v * math.cos(th) * PAS_SIM
        y += v * math.sin(th) * PAS_SIM
        th += w * PAS_SIM
        traj.append((x, y, th))
    return traj


class SuiveurDWA(Node):

    def __init__(self):
        super().__init__('suiveur_dwa')
        self.declare_parameter('vitesse_max', V_MAX)      # réglable (comparaison équitable, D5)
        self.v_max = self.get_parameter('vitesse_max').value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, '/plan_brut', self.recevoir_plan, qos)
        self.create_subscription(OccupancyGrid, '/map', self.recevoir_carte, qos)
        self.create_subscription(Odometry, '/odom', self.recevoir_odom, 10)
        self.create_subscription(Float32MultiArray, '/pieton', self.recevoir_pieton, 10)
        self.pub_vitesse = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_arrivee = self.create_publisher(Bool, '/arrivee', 10)

        self.info = None                # infos de la carte (origine, résolution)
        self.securite = None            # obstacles gonflés de 15 cm
        self.secours = None             # obstacles gonflés de 8 cm (si déjà dans la marge)
        self.pose = None                # (x, y, theta)
        self.pieton = None              # (x, y, vx, vy) ou None
        self.points = []                # chemin [(x, y), ...]
        self.cap_final = None
        self.actif = False
        self.mode_final = False
        self.alignement = False
        self.v, self.w = 0.0, 0.0       # dernière commande (= vitesse actuelle estimée)
        self.suivi_progres = (math.inf, 0.0)    # (meilleure distance au but, heure)
        self.deblocage_jusqua = 0.0
        self.t = 0.0                    # horloge interne (s)

        self.create_timer(DT, self.boucle)
        self.get_logger().info('Conducteur DWA prêt : en attente d\'un chemin.')

    # ---------- Réception ----------
    def recevoir_carte(self, msg):
        self.info = msg.info
        h, w = msg.info.height, msg.info.width
        valeurs = np.array(msg.data, dtype=np.int16).reshape(h, w)
        obstacles = (valeurs >= 50) | (valeurs < 0)
        r = msg.info.resolution
        self.securite = gonfler(obstacles, math.ceil(MARGE / r))
        self.secours = gonfler(obstacles, math.ceil(MARGE_SECOURS / r))

    def recevoir_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, theta)

    def recevoir_pieton(self, msg):
        d = msg.data
        self.pieton = (d[1], d[2], d[3], d[4]) if d[0] > 0.5 else None

    def recevoir_plan(self, msg):
        if not msg.poses:
            return
        self.points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        q = msg.poses[-1].pose.orientation
        self.cap_final = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.actif, self.mode_final, self.alignement = True, False, False
        self.suivi_progres = (math.inf, self.t)
        self.get_logger().info(f'Nouveau chemin : {len(self.points)} points (DWA).')

    # ---------- Outils ----------
    def bloque(self, grille, x, y):
        c = int((x - self.info.origin.position.x) / self.info.resolution)
        l = int((y - self.info.origin.position.y) / self.info.resolution)
        if not (0 <= l < grille.shape[0] and 0 <= c < grille.shape[1]):
            return True
        return bool(grille[l, c])

    def cible(self, x, y):
        """Point du chemin situé VISEE mètres devant le point le plus proche du robot."""
        k = min(range(len(self.points)), key=lambda i: math.dist((x, y), self.points[i]))
        parcouru = 0.0
        while k < len(self.points) - 1 and parcouru < VISEE:
            parcouru += math.dist(self.points[k], self.points[k + 1])
            k += 1
        return self.points[k]

    # ---------- DWA ----------
    def dwa(self, x, y, th, cible):
        grille = self.securite
        if self.bloque(self.securite, x, y):        # déjà dans la marge normale (après un évitement)
            grille = self.secours
        v_min, v_max = max(-V_RECUL, self.v - A_MAX * DT), min(self.v_max, self.v + A_MAX * DT)
        w_min, w_max = max(-W_MAX, self.w - ALPHA_MAX * DT), min(W_MAX, self.w + ALPHA_MAX * DT)

        meilleur, meilleur_score = None, -math.inf
        secours, secours_dist = (0.0, 0.0), -math.inf
        for iv in range(6):
            v = v_min + (v_max - v_min) * iv / 5
            for iw in range(11):
                w = w_min + (w_max - w_min) * iw / 10
                traj = imaginer(x, y, th, v, w)

                # 3. rejet des trajectoires dangereuses
                degagement, dist_min, danger = 1.0, math.inf, False
                for k, (tx, ty, _) in enumerate(traj):
                    if self.bloque(grille, tx, ty):           # obstacle fixe
                        danger = True
                        break
                    if self.pieton is not None:               # piéton : position PRÉDITE
                        px, py, pvx, pvy = self.pieton
                        t_futur = (k + 1) * PAS_SIM
                        d = math.hypot(px + pvx * t_futur - tx, py + pvy * t_futur - ty)
                        marge = DIST_SECU + INCERTITUDE * t_futur
                        dist_min = min(dist_min, d - marge)
                        if d < marge:
                            danger = True
                            break
                        degagement = min(degagement, d - marge)
                if danger:
                    # on retient quand même celle qui s'éloigne le plus du piéton
                    if not self.bloque(grille, traj[0][0], traj[0][1]) and dist_min > secours_dist:
                        secours, secours_dist = (v, w), dist_min
                    continue

                # 4. note
                xf, yf, _ = traj[-1]
                score = (-P_BUT * math.hypot(cible[0] - xf, cible[1] - yf)
                         + P_DEGAGEMENT * min(degagement, 0.3)
                         + P_VITESSE * v)
                if score > meilleur_score:
                    meilleur, meilleur_score = (v, w), score
        return meilleur if meilleur is not None else secours

    # ---------- Boucle ----------
    def boucle(self):
        self.t += DT
        if not self.actif or self.pose is None or self.securite is None or not self.points:
            return
        x, y, th = self.pose
        but = self.points[-1]
        dist_but = math.dist((x, y), but)

        if self.alignement:                                   # pivot final
            ecart = angle_normalise(self.cap_final - th)
            if abs(ecart) < 0.05:
                self.terminer()
            else:
                w = max(-W_MAX, min(W_MAX, 2.0 * ecart))
                self.commander(0.0, math.copysign(max(abs(w), 0.3), ecart))
            return

        if self.mode_final or dist_but < DIST_FINALE:         # mode précis
            self.mode_final = True
            if dist_but < TOLERANCE_FINALE:
                self.commander(0.0, 0.0)
                self.alignement = True
                return
            erreur = angle_normalise(math.atan2(but[1] - y, but[0] - x) - th)
            v = 0.0 if abs(erreur) > 0.35 else min(0.15, 1.5 * dist_but)
            self.commander(v, max(-W_MAX, min(W_MAX, 2.0 * erreur)))
            return

        cible = self.cible(x, y)

        # déblocage : si la distance au but ne baisse plus depuis 3 s, on pivote vers la cible 1 s
        meilleure, heure = self.suivi_progres
        if dist_but < meilleure - 0.02:
            self.suivi_progres = (dist_but, self.t)
        elif self.t - heure > 3.0 and self.t > self.deblocage_jusqua:
            self.get_logger().info('DWA : pas de progrès depuis 3 s -> déblocage (pivot)')
            self.deblocage_jusqua = self.t + 1.0
            self.suivi_progres = (dist_but, self.t + 1.0)
        if self.t < self.deblocage_jusqua:
            erreur = angle_normalise(math.atan2(cible[1] - y, cible[0] - x) - th)
            self.commander(0.0, math.copysign(0.8, erreur))
            return

        v, w = self.dwa(x, y, th, cible)
        self.commander(v, w)

    def commander(self, v, w):
        self.v, self.w = v, w
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub_vitesse.publish(msg)

    def terminer(self):
        self.commander(0.0, 0.0)
        self.actif, self.alignement, self.mode_final = False, False, False
        self.pub_arrivee.publish(Bool(data=True))
        self.get_logger().info('Arrivé à destination ! (DWA)')


def main():
    rclpy.init()
    noeud = SuiveurDWA()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            noeud.commander(0.0, 0.0)
            noeud.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
