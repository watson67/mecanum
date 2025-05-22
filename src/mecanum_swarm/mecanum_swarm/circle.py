#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Point
import math
ALL_ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]  # Liste de tous les robots possibles

class CircleTrajectory(Node):
    def __init__(self):
        super().__init__('circle_trajectory')
        
        # Paramètres du cercle
        self.radius = 0.5  # Augmentation du rayon à 25cm au lieu de 5cm
        self.center_x = 0.0
        self.center_y = 0.0
        self.angle = 0.0
        self.angle_step = 45.0  # Augmentation de l'incrément angulaire à 45 degrés au lieu de 15
        
        # Publisher pour les nouveaux points cibles
        self.goal_publisher = self.create_publisher(
            Point, '/goal_point', 10
        )
        
        # Subscriber pour savoir quand le point est atteint
        self.create_subscription(
            Int32, '/target_reached', self.target_reached_callback, 10
        )
        
        # Nom du robot (à adapter selon le contexte, ici on prend le premier de la liste)
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
                f"/{name}/target_status",
                lambda msg, robot=name: self.target_status_callback(msg, robot),
                10
            )

        # Publisher pour le type de trajectoire
        self.trajectory_type_pub = self.create_publisher(
            String, '/trajectory_type', 10
        )
        # Publier le type de trajectoire au démarrage
        msg = String()
        msg.data = "circle"
        self.trajectory_type_pub.publish(msg)

        self.get_logger().info('Circle trajectory node initialized')
        
        # Publier le premier point dès le démarrage
        self.publish_next_point()
        
    def target_reached_callback(self, msg):
        """Callback appelé quand un message est reçu sur le topic /target_reached"""
        if msg.data == 1:
            self.get_logger().info('Target reached, moving to next point')
            # Publier le statut sur le topic dédié à ce robot
            self.publish_target_status(1)
            # Le point est atteint, passer au suivant
            self.publish_next_point()
    
    def publish_target_status(self, status):
        """Publie le statut d'atteinte de la cible sur le topic dédié au robot."""
        status_msg = Int32()
        status_msg.data = status
        self.target_status_publisher.publish(status_msg)

    def target_status_callback(self, msg, robot_name):
        """Callback pour la réception du statut d'atteinte de cible de chaque robot."""
        self.target_status[robot_name] = msg.data
        self.get_logger().info(f"Robot {robot_name} status: {msg.data}")
        # Exemple : vérifier si tous les robots ont atteint leur cible
        if all(status == 1 for status in self.target_status.values()):
            self.get_logger().info("Tous les robots ont atteint leur cible.")
            # Le point est atteint, passer au suivant
            self.publish_next_point()

    def publish_next_point(self):
        """Calcule et publie le prochain point sur le cercle"""
        # Convertir l'angle en radians
        angle_rad = math.radians(self.angle)
        
        # Calculer les coordonnées du nouveau point
        x = self.center_x + self.radius * math.cos(angle_rad)
        y = self.center_y + self.radius * math.sin(angle_rad)
        
        # Créer et publier le message
        point_msg = Point()
        point_msg.x = float(x)
        point_msg.y = float(y)
        point_msg.z = 0.0
        
        self.goal_publisher.publish(point_msg)
        self.get_logger().info(f'Published new goal point: x={x:.4f}, y={y:.4f}')
        
        # Mettre à jour l'angle pour le prochain point
        self.angle = (self.angle + self.angle_step) % 360.0

def main(args=None):
    rclpy.init(args=args)
    node = CircleTrajectory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
