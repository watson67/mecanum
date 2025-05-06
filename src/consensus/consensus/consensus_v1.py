#!/usr/bin/env python3

# Ce script correspond a une prmeiere implementation de la loi de consensus
# Contrôle leurs mouvements en formation en fonction des commandes de vitesse 
# du barycentre et des positions des robots.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

# Noms des robots dans l'essaim
ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]

# Sujets ROS pour les poses des robots
POSE_TOPICS = [f"/vrpn_mocap/{name}/pose" for name in ROBOT_NAMES]

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')

        # Configuration QoS pour la compatibilité avec VRPN
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Initialisation des poses des robots (None signifie non initialisé)
        self.poses = {name: None for name in ROBOT_NAMES}

        # Création des publishers pour les commandes de vitesse des robots
        self.cmd_publishers = {
            name: self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            for name in ROBOT_NAMES
        }

        # Paramètres du contrôleur PID
        self.Kp = 0.5  # Gain proportionnel
        self.Ki = 0.05  # Gain intégral
        self.Kd = 0.1  # Gain dérivé
        self.dt = 0.1  # Période de contrôle en secondes

        # Mémoire pour les calculs PID de chaque robot
        self.pid_state = {
            name: {
                "integral_x": 0.0,
                "integral_y": 0.0,
                "prev_error_x": 0.0,
                "prev_error_y": 0.0
            }
            for name in ROBOT_NAMES
        }

        # Commande de vitesse globale pour le barycentre de l'essaim
        self.swarm_cmd_vel = np.zeros(2)

        # Abonnement au sujet de commande de vitesse du barycentre
        self.create_subscription(
            Twist,
            '/Swarm/cmd_vel',
            self.cmd_vel_callback,
            qos_profile=self.qos_profile
        )

        # Abonnements aux sujets des poses des robots
        for name, topic in zip(ROBOT_NAMES, POSE_TOPICS):
            self.create_subscription(
                PoseStamped,
                topic,
                self.make_pose_callback(name),
                qos_profile=self.qos_profile
            )

        # Création d'un timer pour exécuter la boucle de contrôle périodiquement
        self.timer = self.create_timer(self.dt, self.control_loop)

    # Génère un callback pour mettre à jour la pose d'un robot donné
    def make_pose_callback(self, name):
        def callback(msg):
            self.poses[name] = np.array([msg.pose.position.x, msg.pose.position.y])
        return callback

    # Callback pour mettre à jour la commande de vitesse du barycentre
    def cmd_vel_callback(self, msg):
        self.swarm_cmd_vel[0] = msg.linear.x
        self.swarm_cmd_vel[1] = msg.linear.y

    # Calcul PID pour ajuster les vitesses des robots
    def compute_pid(self, error_x, error_y, integral_x, integral_y, prev_error_x, prev_error_y):
        # Calcul des termes proportionnel, intégral et dérivé
        up_x = self.Kp * error_x
        up_y = self.Kp * error_y

        ud_x = self.Kd * (error_x - prev_error_x) / self.dt
        ud_y = self.Kd * (error_y - prev_error_y) / self.dt

        integral_x += self.Ki * error_x * self.dt
        integral_y += self.Ki * error_y * self.dt

        # Calcul des vitesses en x et y
        vx = -(up_x + ud_x + integral_x)
        vy = -(up_y + ud_y + integral_y)

        return vx, vy, integral_x, integral_y, error_x, error_y

    # Boucle principale de contrôle
    def control_loop(self):
        # Vérifie si toutes les poses des robots sont initialisées
        if not all(pose is not None for pose in self.poses.values()):
            return

        # Calcul du barycentre de l'essaim
        barycenter = np.mean([p for p in self.poses.values()], axis=0)
        # Anticipation de la position cible du barycentre en fonction de la commande de vitesse globale
        target_barycenter = barycenter + self.swarm_cmd_vel * self.dt

        # Contrôle de chaque robot
        for name in ROBOT_NAMES:
            # Position actuelle du robot
            pi = self.poses[name]
            # Calcul de l'erreur entre la position actuelle du robot et la position cible du barycentre
            error = pi - target_barycenter

            # Application de la loi de consensus
            # Le terme de consensus est calculé en fonction des positions des voisins
            consensus_term = np.zeros(2)
            for neighbor_name in ROBOT_NAMES:
                if neighbor_name != name:  # Exclut le robot lui-même
                    # Contribution de chaque voisin : différence entre sa position et celle du robot actuel
                    consensus_term += self.poses[neighbor_name] - pi
            # Moyenne des contributions des voisins
            consensus_term /= len(ROBOT_NAMES) - 1

            # Ajout du terme de consensus à l'erreur
            # Le facteur 0.1 contrôle l'influence du consensus sur le comportement du robot
            error -= 0.1 * consensus_term

            # Récupération de l'état PID du robot
            state = self.pid_state[name]
            # Calcul des vitesses en x et y à l'aide du contrôleur PID
            vx_pid, vy_pid, ix, iy, ex, ey = self.compute_pid(
                error[0], error[1],
                state["integral_x"], state["integral_y"],
                state["prev_error_x"], state["prev_error_y"]
            )

            # Mise à jour de l'état PID
            state.update({
                "integral_x": ix,  # Mise à jour de l'intégrale en x
                "integral_y": iy,  # Mise à jour de l'intégrale en y
                "prev_error_x": ex,  # Mise à jour de l'erreur précédente en x
                "prev_error_y": ey   # Mise à jour de l'erreur précédente en y
            })

            # Ajout de la commande globale du barycentre aux vitesses calculées par le PID
            vx_total = vx_pid + self.swarm_cmd_vel[0]
            vy_total = vy_pid + self.swarm_cmd_vel[1]

            # Conversion des vitesses en message Twist
            # Calcul de la norme de la vitesse (magnitude)
            v = np.hypot(vx_total, vy_total)
            # Calcul de l'angle cible en fonction des vitesses x et y
            target_theta = np.arctan2(vy_total, vx_total)
            # Orientation actuelle supposée nulle (si une IMU est disponible, elle peut être utilisée ici)
            theta = 0.0
            # Calcul de la vitesse angulaire pour atteindre l'angle cible
            w = 2.0 * np.arctan2(np.sin(target_theta - theta), np.cos(target_theta - theta))

            # Création du message Twist pour commander le robot
            twist = Twist()
            twist.linear.x = v  # Vitesse linéaire
            twist.angular.z = w  # Vitesse angulaire
            # Publication de la commande de vitesse pour le robot
            self.cmd_publishers[name].publish(twist)

# Point d'entrée principal du script
def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    rclpy.spin(node)  # Exécution du nœud ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
