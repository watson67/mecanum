#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Point
import math
from mecanum_swarm.config import ALL_ROBOT_NAMES

class EightTrajectory(Node):
    def __init__(self):
        super().__init__('eight_trajectory')

        # Paramètres du "8"
        self.center_x = 0.0
        self.center_y = 0.0
        self.radius_x = 0.86  # Pour aller de -0.64 à 1.08, mais on veut -1 à 1, donc 1.0
        self.radius_y = 0.23  # Faible largeur selon y
        self.num_points = 16  # Nombre de points pour le "8"
        self.current_point = 0

        # Calculer les points du "8"
        self.points = self.compute_eight_points()

        # Publisher pour les nouveaux points cibles
        self.goal_publisher = self.create_publisher(
            Point, '/goal_point', 10
        )

        # Subscriber pour savoir quand le point est atteint
        self.create_subscription(
            Int32, '/target_reached', self.target_reached_callback, 10
        )

        # Nom du robot (ici on prend le premier de la liste)
        self.robot_name = ALL_ROBOT_NAMES[0]

        # Contribution à la détection d'atteinte de cible
        self.target_status_publisher = self.create_publisher(
            Int32, f"/{self.robot_name}/target_status", 10
        )

        # Subscribers pour chaque robot sur /target_status/{robot_name}
        self.target_status = {name: 0 for name in ALL_ROBOT_NAMES}
        for name in ALL_ROBOT_NAMES:
            self.create_subscription(
                Int32,
                f"/target_status/{name}",
                lambda msg, robot=name: self.target_status_callback(msg, robot),
                10
            )

        # Publisher pour signaler la fin du parcours
        self.travel_finished_publisher = self.create_publisher(
            Int32, '/travel_finished', 10
        )

        # Publisher pour le type de trajectoire
        self.trajectory_type_pub = self.create_publisher(
            String, '/trajectory_type', 10
        )
        # Publier le type de trajectoire au démarrage
        msg = String()
        msg.data = "eight"
        self.trajectory_type_pub.publish(msg)

        self.get_logger().info('Eight trajectory node initialized')

        # Subscriber pour /master
        self.create_subscription(
            Int32, '/master', self.master_callback, 10
        )

    def compute_eight_points(self):
        """Génère les points d'une trajectoire en 8 passant par (0,0) et débutant/finissant à (-0.64, -0.77)"""
        points = []
        start_x, start_y = -0.64, -0.77
        points.append((start_x, start_y))  # Point de départ

        # Paramètres du "8"
        # Utilisation de la courbe de Lissajous pour un vrai 8 centré sur (0,0)
        # x = a * sin(t), y = b * sin(2t)
        a = 1.0  # amplitude x (-1 à 1)
        b = 0.23  # faible largeur y
        num_points = self.num_points

        for i in range(num_points - 1):
            t = 2 * math.pi * i / num_points
            x = a * math.sin(t)
            y = b * math.sin(2 * t)
            points.append((x, y))

        points.append((start_x, start_y))  # Retour au point de départ
        return points

    def target_reached_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info('TPoint cible atteint, envoi du prochain point')
            self.publish_target_status(1)
            self.publish_next_point()

    def publish_target_status(self, status):
        status_msg = Int32()
        status_msg.data = status
        self.target_status_publisher.publish(status_msg)

    def target_status_callback(self, msg, robot_name):
        self.target_status[robot_name] = msg.data
        self.get_logger().info(f"Robot {robot_name} status: {msg.data}")
        if all(status == 1 for status in self.target_status.values()):
            self.get_logger().info("Tous les robots ont atteint leur cible.")
            self.publish_next_point()

    def publish_next_point(self):
        if self.current_point >= len(self.points):
            self.get_logger().info("Trajectoire terminée.")
            # Publier 1 sur /travel_finished
            finished_msg = Int32()
            finished_msg.data = 1
            self.travel_finished_publisher.publish(finished_msg)
            return
        x, y = self.points[self.current_point]
        point_msg = Point()
        point_msg.x = float(x)
        point_msg.y = float(y)
        point_msg.z = 0.0
        self.goal_publisher.publish(point_msg)
        self.get_logger().info(f'Nouveau point: x={x:.4f}, y={y:.4f}')
        self.current_point += 1

    def master_callback(self, msg):
        if msg.data == 1:
            # Republier le type de trajectoire à chaque activation
            msg_type = String()
            msg_type.data = "eight"
            self.trajectory_type_pub.publish(msg_type)
            self.get_logger().info(f"Republished trajectory type on /master: eight")
            # Publier le premier point cible lors de l'activation
            self.current_point = 0
            self.publish_next_point()

def main(args=None):
    rclpy.init(args=args)
    node = EightTrajectory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
