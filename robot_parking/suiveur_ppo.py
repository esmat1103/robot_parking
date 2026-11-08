"""
suiveur_ppo.py : conducteur PPO (apprentissage par renforcement) + bouclier de sécurité, étape D4.

Même rôle et même interface que suiveur.py et suiveur_dwa.py (choix au lancement) :
Écoute :
    /plan    (Path)              : les points de passage (chemin A* lissé) + orientation finale
    /odom    (Odometry)          : position et orientation du robot
    /map     (OccupancyGrid)     : les obstacles fixes
    /pieton  (Float32MultiArray) : [présent, x, y, vx, vy] du piéton
Publie :
    /cmd_vel (Twist) : (remappé en /cmd_vel_brut -> passe par la sécurité)
    /arrivee (Bool)  : le robot est arrivé (pour la mission)

Le réseau PPO a été ENTRAÎNÉ dans l'ancien projet (navigation/env_parking.py, 78/90 scénarios).
Ce nœud lui donne EXACTEMENT les mêmes 19 observations qu'à l'entraînement, toutes relatives
au robot (donc indépendantes de la carte) :
    prochain point de passage (distance, cos, sin)                           (3)
    vitesses actuelles v, w                                                  (2)
    9 « rayons » vers les obstacles gonflés de 15 cm, de -90° à +90°, 50 cm  (9)
    piéton : présent ?, position et vitesse vues par le robot                (5)
et applique ses 7 actions (avancer, tourner, pivoter, ralentir, s'arrêter), 20 fois par seconde.

BOUCLIER (« safe RL ») : avant d'exécuter l'action de PPO, on imagine 1,5 s d'avenir (comme DWA).
Dangereuse ? -> on prend l'action sûre que PPO préfère, ou une action d'urgence (reculer).

Le modèle doit être copié dans ~/parking_ws/modeles/ppo_parking.zip
"""
import math
import os

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import io
import zipfile

from gymnasium import spaces
from stable_baselines3.common.policies import ActorCriticPolicy

from robot_parking.astar import gonfler

CHEMIN_MODELE = os.path.expanduser('~/parking_ws/modeles/ppo_parking.zip')
DT = 0.05                               # 20 décisions par seconde (comme à l'entraînement)

# ---- Valeurs de l'ENTRAÎNEMENT (ne pas modifier : le réseau a appris avec elles) ----
V_MAX, W_MAX = 0.20, 2.0
PORTEE_RAYONS = 0.5
ANGLES_RAYONS = np.radians(np.linspace(-90, 90, 9))
MARGE = 0.15                            # obstacles gonflés de 15 cm
ACTIONS = {
    0: (V_MAX, 0.0),                    # avancer
    1: (0.12, +W_MAX / 2),              # avancer en tournant à gauche
    2: (0.12, -W_MAX / 2),              # avancer en tournant à droite
    3: (0.0, +W_MAX),                   # pivoter sur place à gauche
    4: (0.0, -W_MAX),                   # pivoter sur place à droite
    5: (0.07, 0.0),                     # ralentir
    6: (0.0, 0.0),                      # s'arrêter
}
ACTIONS_URGENCE = {                     # seul le bouclier les utilise
    7: (-0.10, 0.0),
    8: (-0.08, +W_MAX / 2),
    9: (-0.08, -W_MAX / 2),
}
TOUTES = {**ACTIONS, **ACTIONS_URGENCE}
NOMS = ['avancer', 'avancer+gauche', 'avancer+droite', 'pivot gauche', 'pivot droite',
        'ralentir', 'stop', 'reculer', 'reculer+gauche', 'reculer+droite']
SEUIL_POINT = 0.05                      # point de passage atteint à 5 cm

# ---- Bouclier (comme l'ancien bouclier.py / dwa.py) ----
HORIZON, PAS_SIM, HORIZON_FIXE = 1.5, 0.1, 0.3
DIST_SECU = 0.12 + 0.04 + 0.04          # robot + piéton + marge
INCERTITUDE = 0.05
MARGE_CHOC = 0.08                       # « toucher une voiture » ≈ centre à moins de 8 cm du bord

# ---- Fin de parcours (pour la lecture de plaque) ----
DIST_FINALE, TOLERANCE_FINALE = 0.10, 0.015


def charger_politique():
    """Charge le réseau PPO entraîné SANS PPO.load().
    Le .zip a été sauvegardé avec numpy 2 (ici numpy 1, pour cv_bridge) : on ne lit que
    policy.pth = les POIDS du réseau (format PyTorch, sans numpy), et on reconstruit
    le même réseau qu'à l'entraînement (MlpPolicy : 2 couches de 64 neurones)."""
    with zipfile.ZipFile(CHEMIN_MODELE) as z:
        poids = torch.load(io.BytesIO(z.read('policy.pth')), map_location='cpu')
    politique = ActorCriticPolicy(spaces.Box(-1.0, 1.0, shape=(19,), dtype=np.float32),
                                  spaces.Discrete(7), lr_schedule=lambda _: 0.0)
    politique.load_state_dict(poids)
    politique.eval()
    return politique


def angle_normalise(a):
    return math.atan2(math.sin(a), math.cos(a))


def imaginer(x, y, th, v, w):
    traj = []
    for _ in range(int(HORIZON / PAS_SIM)):
        x += v * math.cos(th) * PAS_SIM
        y += v * math.sin(th) * PAS_SIM
        th += w * PAS_SIM
        traj.append((x, y, th))
    return traj


class SuiveurPPO(Node):

    def __init__(self):
        super().__init__('suiveur_ppo')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, '/plan', self.recevoir_plan, qos)
        self.create_subscription(OccupancyGrid, '/map', self.recevoir_carte, qos)
        self.create_subscription(Odometry, '/odom', self.recevoir_odom, 10)
        self.create_subscription(Float32MultiArray, '/pieton', self.recevoir_pieton, 10)
        self.pub_vitesse = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_arrivee = self.create_publisher(Bool, '/arrivee', 10)

        self.politique = charger_politique()       # le réseau PPO entraîné
        self.info = self.securite = self.choc = None
        self.pose = None
        self.pieton = None
        self.points, self.i, self.cap_final = [], 1, None
        self.actif = self.mode_final = self.alignement = False
        self.v = self.w = 0.0
        self.corrections = 0            # nombre de fois où le bouclier a corrigé PPO
        self.decisions = 0

        self.create_timer(DT, self.boucle)
        self.get_logger().info(f'Conducteur PPO prêt (modèle : {CHEMIN_MODELE}).')

    # ---------- Réception ----------
    def recevoir_carte(self, msg):
        self.info = msg.info
        h, w = msg.info.height, msg.info.width
        valeurs = np.array(msg.data, dtype=np.int16).reshape(h, w)
        obstacles = (valeurs >= 50) | (valeurs < 0)
        r = msg.info.resolution
        self.securite = gonfler(obstacles, math.ceil(MARGE / r))     # pour les rayons
        self.choc = gonfler(obstacles, math.ceil(MARGE_CHOC / r))   # pour le bouclier

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
        self.i = 1 if len(self.points) > 1 else 0                  # le point 0 = le départ
        self.actif, self.mode_final, self.alignement = True, False, False
        self.corrections = self.decisions = 0
        self.get_logger().info(f'Nouveau chemin : {len(self.points)} points de passage (PPO).')

    # ---------- Outils ----------
    def bloque(self, grille, x, y):
        c = int((x - self.info.origin.position.x) / self.info.resolution)
        l = int((y - self.info.origin.position.y) / self.info.resolution)
        if not (0 <= l < grille.shape[0] and 0 <= c < grille.shape[1]):
            return True
        return bool(grille[l, c])

    def vers_repere_robot(self, dx, dy):
        th = self.pose[2]
        c, s = math.cos(-th), math.sin(-th)
        return dx * c - dy * s, dx * s + dy * c

    def rayon(self, angle):
        x, y, th = self.pose
        a = th + angle
        pas = self.info.resolution
        for k in range(1, int(PORTEE_RAYONS / pas) + 1):
            d = k * pas
            if self.bloque(self.securite, x + d * math.cos(a), y + d * math.sin(a)):
                return d
        return PORTEE_RAYONS

    def observation(self):
        """Les 19 nombres, calculés EXACTEMENT comme dans env_parking.py."""
        x, y, _ = self.pose
        px, py = self.points[min(self.i, len(self.points) - 1)]
        lx, ly = self.vers_repere_robot(px - x, py - y)
        d, ang = math.hypot(lx, ly), math.atan2(ly, lx)
        obs = [min(d, 2.0) / 2.0, math.cos(ang), math.sin(ang), self.v / V_MAX, self.w / W_MAX]
        obs += [2 * self.rayon(a) / PORTEE_RAYONS - 1 for a in ANGLES_RAYONS]
        if self.pieton is not None:
            ppx, ppy, pvx, pvy = self.pieton
            rx, ry = self.vers_repere_robot(ppx - x, ppy - y)
            vx, vy = self.vers_repere_robot(pvx, pvy)
            obs += [1.0, np.clip(rx, -1, 1), np.clip(ry, -1, 1), vx / 0.1, vy / 0.1]
        else:
            obs += [0.0] * 5
        return np.clip(np.array(obs, dtype=np.float32), -1, 1)

    # ---------- Bouclier ----------
    def evaluer(self, v, w):
        """(dangereux ?, marge minimale au piéton) si on garde (v, w) pendant 1,5 s."""
        x, y, th = self.pose
        marge_min = math.inf
        for k, (tx, ty, _) in enumerate(imaginer(x, y, th, v, w)):
            t = (k + 1) * PAS_SIM
            if t <= HORIZON_FIXE and self.bloque(self.choc, tx, ty):
                return True, -math.inf                         # va toucher une voiture / un mur
            if self.pieton is not None:
                px, py, pvx, pvy = self.pieton
                d = math.hypot(px + pvx * t - tx, py + pvy * t - ty)
                marge = DIST_SECU + INCERTITUDE * t
                marge_min = min(marge_min, d - marge)
                if d < marge:
                    return True, marge_min                     # trop près du piéton
        return False, marge_min

    def proteger(self, obs, action):
        if not self.evaluer(*ACTIONS[action])[0]:
            return action, False
        with torch.no_grad():
            probas = self.politique.get_distribution(
                torch.as_tensor(obs).unsqueeze(0)).distribution.probs[0].numpy()
        candidates = sorted(ACTIONS, key=lambda a: -probas[a]) + list(ACTIONS_URGENCE)
        meilleure, meilleure_marge = 6, -math.inf
        for a in candidates:
            dangereux, marge = self.evaluer(*TOUTES[a])
            if not dangereux:
                return a, True
            if marge > meilleure_marge:
                meilleure, meilleure_marge = a, marge
        return meilleure, True

    # ---------- Boucle ----------
    def boucle(self):
        if not self.actif or self.pose is None or self.securite is None or not self.points:
            return
        x, y, th = self.pose
        but = self.points[-1]
        dist_but = math.dist((x, y), but)

        if self.alignement:                                     # pivot final
            ecart = angle_normalise(self.cap_final - th)
            if abs(ecart) < 0.05:
                self.terminer()
            else:
                self.commander(0.0, math.copysign(max(min(abs(2.0 * ecart), W_MAX), 0.3), ecart))
            return

        if self.mode_final or (self.i >= len(self.points) - 1 and dist_but < DIST_FINALE):
            self.mode_final = True                              # mode précis (lecture de plaque)
            if dist_but < TOLERANCE_FINALE:
                self.commander(0.0, 0.0)
                self.alignement = True
                return
            erreur = angle_normalise(math.atan2(but[1] - y, but[0] - x) - th)
            v = 0.0 if abs(erreur) > 0.35 else min(0.15, 1.5 * dist_but)
            self.commander(v, max(-1.5, min(1.5, 2.0 * erreur)))
            return

        # point de passage atteint ? (comme à l'entraînement : 5 cm)
        if self.i < len(self.points) - 1 and \
                math.dist((x, y), self.points[self.i]) < SEUIL_POINT:
            self.i += 1

        obs = self.observation()
        action, _ = self.politique.predict(obs, deterministic=True)
        action, corrige = self.proteger(obs, int(action))
        self.decisions += 1
        if corrige:
            self.corrections += 1
            self.get_logger().info(f'Bouclier : action corrigée -> {NOMS[action]}',
                                   throttle_duration_sec=1.0)
        self.commander(*TOUTES[action])

    def commander(self, v, w):
        self.v, self.w = v, w
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub_vitesse.publish(msg)

    def terminer(self):
        self.commander(0.0, 0.0)
        self.actif = self.alignement = self.mode_final = False
        self.pub_arrivee.publish(Bool(data=True))
        part = 100 * self.corrections / max(1, self.decisions)
        self.get_logger().info(f'Arrivé à destination ! (PPO, bouclier utilisé sur '
                               f'{self.corrections}/{self.decisions} décisions = {part:.0f} %)')


def main():
    rclpy.init()
    noeud = SuiveurPPO()
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
