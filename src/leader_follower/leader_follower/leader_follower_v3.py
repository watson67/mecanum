#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Follower(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Déclarer les paramètres pour le leader et les deux followers
        self.declare_parameter('leader', 'leader')
        self.declare_parameter('follower1', 'follower1')
        self.declare_parameter('follower2', 'follower2')

        # Retrieve parameter values
        leader = self.get_parameter('leader').get_parameter_value().string_value
        follower1 = self.get_parameter('follower1').get_parameter_value().string_value
        follower2 = self.get_parameter('follower2').get_parameter_value().string_value

        # Construire dynamiquement les noms des topics
        self.leader_pose_topic = f'/vrpn_mocap/{leader}/pose'
        self.follower1_pose_topic = f'/vrpn_mocap/{follower1}/pose'
        self.follower2_pose_topic = f'/vrpn_mocap/{follower2}/pose'
        self.follower1_cmd_topic = f'/{follower1}/cmd_vel'
        self.follower2_cmd_topic = f'/{follower2}/cmd_vel'

        # QoS compatible avec VRPN
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Paramètres PID
        self.Kp = 1
        self.Ki = 0.02
        self.Kd = 0.3
        self.dt = 0.1
        self.target_dist = 0.5  # Target distance to leader
        self.target_dist_between_followers = 0.5  # Target distance between followers

        # Angle
        self.target_angle_follower1 = math.radians(30)  
        self.target_angle_follower2 = math.radians(-30)  

        # Variables pour le calcul PID
        self.integral_x1 = 0.0
        self.integral_y1 = 0.0
        self.prev_error_x1 = 0.0
        self.prev_error_y1 = 0.0

        self.integral_x2 = 0.0
        self.integral_y2 = 0.0
        self.prev_error_x2 = 0.0
        self.prev_error_y2 = 0.0

        # PID variables for angle control
        self.integral_angle1 = 0.0
        self.prev_error_angle1 = 0.0

        self.integral_angle2 = 0.0
        self.prev_error_angle2 = 0.0

        self.pose_leader = None
        self.pose_follower1 = None
        self.pose_follower2 = None

        # Subscribers pour les positions du leader et des followers
        self.sub_leader = self.create_subscription(
            PoseStamped,
            self.leader_pose_topic,
            self.leader_callback,
            qos_profile
        )

        self.sub_follower1 = self.create_subscription(
            PoseStamped,
            self.follower1_pose_topic,
            self.follower1_callback,
            qos_profile
        )

        self.sub_follower2 = self.create_subscription(
            PoseStamped,
            self.follower2_pose_topic,
            self.follower2_callback,
            qos_profile
        )

        # Publishers pour envoyer les commandes aux followers
        self.publisher_cmd1 = self.create_publisher(
            Twist,
            self.follower1_cmd_topic,
            10
        )

        self.publisher_cmd2 = self.create_publisher(
            Twist,
            self.follower2_cmd_topic,
            10
        )

        # Timer pour exécuter la logique périodiquement
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def leader_callback(self, msg):
        # Callback pour mettre à jour la position du leader
        self.pose_leader = msg

    def follower1_callback(self, msg):
        # Callback pour mettre à jour la position du follower1
        self.pose_follower1 = msg

    def follower2_callback(self, msg):
        # Callback pour mettre à jour la position du follower2
        self.pose_follower2 = msg

    def compute_pid(self, error_x, error_y, integral_x, integral_y, prev_error_x, prev_error_y):
        # Calcul PID pour générer les commandes de vitesse
        up_x = self.Kp * error_x
        up_y = self.Kp * error_y

        ud_x = self.Kd * (error_x - prev_error_x) / self.dt
        ud_y = self.Kd * (error_y - prev_error_y) / self.dt

        integral_x += self.Ki * error_x * self.dt
        integral_y += self.Ki * error_y * self.dt

        vx = -(up_x + ud_x + integral_x)
        vy = -(up_y + ud_y + integral_y)

        return vx, vy, integral_x, integral_y, error_x, error_y

    def compute_pid_angle(self, error_angle, integral_angle, prev_error_angle):
        # PID control for angle
        up = self.Kp * error_angle
        ud = self.Kd * (error_angle - prev_error_angle) / self.dt
        integral_angle += self.Ki * error_angle * self.dt

        angular_z = up + ud + integral_angle
        return angular_z, integral_angle, error_angle

    def timer_callback(self):
        # Vérifier si les positions du leader et des followers sont disponibles
        if self.pose_leader is None or self.pose_follower1 is None or self.pose_follower2 is None:
            return

        # Leader position
        x_leader = self.pose_leader.pose.position.x
        y_leader = self.pose_leader.pose.position.y

        # Follower1 position
        x_follower1 = self.pose_follower1.pose.position.x
        y_follower1 = self.pose_follower1.pose.position.y

        # Follower2 position
        x_follower2 = self.pose_follower2.pose.position.x
        y_follower2 = self.pose_follower2.pose.position.y

        # Calculer les commandes pour le follower1
        # Distance avec le leader
        error_x1_leader = x_leader - x_follower1
        error_y1_leader = y_leader - y_follower1
        dist1_leader = math.hypot(error_x1_leader, error_y1_leader)
        if dist1_leader > 1e-6:
            error_x1_leader = (error_x1_leader / dist1_leader) * (self.target_dist - dist1_leader)
            error_y1_leader = (error_y1_leader / dist1_leader) * (self.target_dist - dist1_leader)

        # Distance avec le follower2
        error_x1_follower2 = x_follower2 - x_follower1
        error_y1_follower2 = y_follower2 - y_follower1
        dist1_follower2 = math.hypot(error_x1_follower2, error_y1_follower2)
        if dist1_follower2 > 1e-6:
            error_x1_follower2 = (error_x1_follower2 / dist1_follower2) * (self.target_dist_between_followers - dist1_follower2)
            error_y1_follower2 = (error_y1_follower2 / dist1_follower2) * (self.target_dist_between_followers - dist1_follower2)

        # Combiner les erreurs pour follower1
        error_x1 = error_x1_leader + error_x1_follower2
        error_y1 = error_y1_leader + error_y1_follower2

        vx1, vy1, self.integral_x1, self.integral_y1, self.prev_error_x1, self.prev_error_y1 = self.compute_pid(
            error_x1, error_y1, self.integral_x1, self.integral_y1, self.prev_error_x1, self.prev_error_y1
        )

        # Calculer l'angle du segment leader-follower1 par rapport à l'origine
        angle_leader_follower1 = math.atan2(y_leader - y_follower1, x_leader - x_follower1)
        error_angle1 = self.target_angle_follower1 - angle_leader_follower1

        # Normalize angle error to [-pi, pi]
        error_angle1 = (error_angle1 + math.pi) % (2 * math.pi) - math.pi

        # Calculer les commandes pour le follower2
        # Distance avec le leader
        error_x2_leader = x_leader - x_follower2
        error_y2_leader = y_leader - y_follower2
        dist2_leader = math.hypot(error_x2_leader, error_y2_leader)
        if dist2_leader > 1e-6:
            error_x2_leader = (error_x2_leader / dist2_leader) * (self.target_dist - dist2_leader)
            error_y2_leader = (error_y2_leader / dist2_leader) * (self.target_dist - dist2_leader)

        # Distance avec le follower1
        error_x2_follower1 = x_follower1 - x_follower2
        error_y2_follower1 = y_follower1 - y_follower2
        dist2_follower1 = math.hypot(error_x2_follower1, error_y2_follower1)
        if dist2_follower1 > 1e-6:
            error_x2_follower1 = (error_x2_follower1 / dist2_follower1) * (self.target_dist_between_followers - dist2_follower1)
            error_y2_follower1 = (error_y2_follower1 / dist2_follower1) * (self.target_dist_between_followers - dist2_follower1)

        # Combiner les erreurs pour follower2
        error_x2 = error_x2_leader + error_x2_follower1
        error_y2 = error_y2_leader + error_y2_follower1

        vx2, vy2, self.integral_x2, self.integral_y2, self.prev_error_x2, self.prev_error_y2 = self.compute_pid(
            error_x2, error_y2, self.integral_x2, self.integral_y2, self.prev_error_x2, self.prev_error_y2
        )

        # Calculer l'angle du segment leader-follower2 par rapport à l'origine
        angle_leader_follower2 = math.atan2(y_leader - y_follower2, x_leader - x_follower2)
        error_angle2 = self.target_angle_follower2 - angle_leader_follower2

        # Normalize angle error to [-pi, pi]
        error_angle2 = (error_angle2 + math.pi) % (2 * math.pi) - math.pi

        # Publier les commandes pour les deux followers
        twist1 = Twist()
        twist1.linear.x = max(min(vx1, 0.5), -0.5)
        twist1.linear.y = max(min(vy1, 0.5), -0.5)
        twist1.angular.z = 0.0  
        self.publisher_cmd1.publish(twist1)

        twist2 = Twist()
        twist2.linear.x = max(min(vx2, 0.5), -0.5)
        twist2.linear.y = max(min(vy2, 0.5), -0.5)
        twist2.angular.z = 0.0  
        self.publisher_cmd2.publish(twist2)

def main(args=None):
    # Initialiser le nœud
    rclpy.init(args=args)
    node = Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
