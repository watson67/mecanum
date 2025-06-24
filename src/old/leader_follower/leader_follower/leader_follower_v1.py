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
        self.K = 1.0         # Gain principal
        self.Ti = 50.0       # Temps intégral
        self.Td = 0.3        # Temps dérivatif
        self.beta = 1.0      # Pondération consigne
        self.N = 10.0        # Coeff. filtre dérivé
        self.dt = 0.1        # Période
        self.gamma = 1.0     # Pondération intégrale
        
        self.target_dist = 0.5  # Distance à maintenir
        self.get_logger().info(
            f'K : {self.K}, Ti={self.Ti}, Td={self.Td}'
        )

        # Variables pour le calcul PID
        self.ui_x = 0.0
        self.ui_y = 0.0
        self.ud_x = 0.0
        self.ud_y = 0.0
        self.y_old_x = 0.0
        self.y_old_y = 0.0

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

        if dist < 1e-3:
            return

        dir_x = dx / dist
        dir_y = dy / dist
        y = dist
        ysp = self.target_dist
        e = ysp - y

        y_output_x = y * dir_x
        y_output_y = y * dir_y
        ysp_output_x = ysp * dir_x
        ysp_output_y = ysp * dir_y
        e_x = ysp_output_x - y_output_x
        e_y = ysp_output_y - y_output_y

        # Proportionnel (avec beta)
        up_x = self.K * (self.beta * ysp_output_x - y_output_x)
        up_y = self.K * (self.beta * ysp_output_y - y_output_y)

        # Dérivé (filtré)
        self.ud_x = (self.Td / (self.N * self.dt + self.Td)) * self.ud_x \
                    - self.K * self.Td * self.N / (self.N * self.dt + self.Td) * (y_output_x - self.y_old_x)
        self.ud_y = (self.Td / (self.N * self.dt + self.Td)) * self.ud_y \
                    - self.K * self.Td * self.N / (self.N * self.dt + self.Td) * (y_output_y - self.y_old_y)

        # Intégrale
        self.ui_x += self.K / self.Ti * self.gamma * self.dt * e_x
        self.ui_y += self.K / self.Ti * self.gamma * self.dt * e_y

        # Commande PID
        vx = -(up_x + self.ui_x + self.ud_x)
        vy = -(up_y + self.ui_y + self.ud_y)

        self.get_logger().info(f"Vx: {vx:.3f}, Vy: {vy:.3f}")

        # Mise à jour des états
        self.y_old_x = y_output_x
        self.y_old_y = y_output_y

        # Limite de vitesse
        max_speed = 0.5
        vx = max(min(vx, max_speed), -max_speed)
        vy = max(min(vy, max_speed), -max_speed)

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
