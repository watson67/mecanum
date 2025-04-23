#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Follower(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Déclarer les paramètres pour le leader et le follower
        self.declare_parameter('leader', 'leader')
        self.declare_parameter('follower', 'follower')

        # Récupérer les valeurs des paramètres
        leader = self.get_parameter('leader').get_parameter_value().string_value
        follower = self.get_parameter('follower').get_parameter_value().string_value

        # Construire dynamiquement les noms des topics
        self.leader_pose_topic = f'/vrpn_mocap/{leader}/pose'
        self.follower_pose_topic = f'/vrpn_mocap/{follower}/pose'
        self.follower_cmd_topic = f'/{follower}/cmd_vel'

        # QoS compatible avec VRPN
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Paramètres PID
        self.Kp = 1  # Gain proportionnel
        self.Ki = 0.02  # Gain intégral
        self.Kd = 0.3  # Gain dérivé
        self.dt = 0.1  # Intervalle de temps
        self.target_dist = 0.5  # Distance cible

        # Variables pour le calcul PID
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.pose_leader = None
        self.pose_follower = None

        # Subscriber pour la position du leader
        self.sub_leader = self.create_subscription(
            PoseStamped,
            self.leader_pose_topic,
            self.leader_callback,
            qos_profile
        )

        # Subscriber pour la position du follower
        self.sub_follower = self.create_subscription(
            PoseStamped,
            self.follower_pose_topic,
            self.follower_callback,
            qos_profile
        )

        # Publisher pour envoyer les commandes au follower
        self.publisher_cmd = self.create_publisher(
            Twist,
            self.follower_cmd_topic,
            10
        )

        # Timer pour exécuter la logique périodiquement
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def leader_callback(self, msg):
        # Callback pour mettre à jour la position du leader
        self.pose_leader = msg

    def follower_callback(self, msg):
        # Callback pour mettre à jour la position du follower
        self.pose_follower = msg

    def timer_callback(self):
        # Vérifier si les positions du leader et du follower sont disponibles
        if self.pose_leader is None or self.pose_follower is None:
            return

        # Calculer la distance entre le leader et le follower
        x1 = self.pose_leader.pose.position.x
        y1 = self.pose_leader.pose.position.y
        x2 = self.pose_follower.pose.position.x
        y2 = self.pose_follower.pose.position.y

        dx = x1 - x2
        dy = y1 - y2
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return

        # Calculer la direction et l'erreur
        dir_x = dx / dist
        dir_y = dy / dist
        error = self.target_dist - dist  # Inverser le calcul de l'erreur
        error_x = dir_x * error
        error_y = dir_y * error

        # Logs des erreurs
        self.get_logger().info(f"Erreur X: {error_x}, Erreur Y: {error_y}")

        # Calculs de la commande avec le PID
        # Constantes
        beta = 1.0  # Supposons que beta est égal à 1 pour simplifier
        N = 10.0  # Coefficient de filtre
        hact = self.dt  # Temps d'échantillonnage

        # Terme proportionnel
        up_x = self.Kp * error_x
        up_y = self.Kp * error_y

        # Terme dérivé
        ud_x = (self.Kd * N * (error_x - self.prev_error_x) - self.Kd * self.prev_error_x) / (N * hact + self.Kd)
        ud_y = (self.Kd * N * (error_y - self.prev_error_y) - self.Kd * self.prev_error_y) / (N * hact + self.Kd)

        # Terme intégral
        self.integral_x += self.Ki * hact * error_x
        self.integral_y += self.Ki * hact * error_y

        # Commande
        vx = -(up_x + self.integral_x + ud_x)
        vy = -(up_y + self.integral_y + ud_y)

        # Logs des vitesses
        self.get_logger().info(f"Vx: {vx}, Vy: {vy}")

        # Mettre à jour les erreurs précédentes
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # Limiter la vitesse maximale
        max_speed = 0.5
        vx = max(min(vx, max_speed), -max_speed)
        vy = max(min(vy, max_speed), -max_speed)

        # Publier la commande Twist
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = 0.0
        self.publisher_cmd.publish(twist)

def main(args=None):
    # Initialiser le nœud
    rclpy.init(args=args)
    node = Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
