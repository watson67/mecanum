#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
import math

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
        
        self.get_logger().info('Circle trajectory node initialized')
        
        # Publier le premier point dès le démarrage
        self.publish_next_point()
        
    def target_reached_callback(self, msg):
        """Callback appelé quand un message est reçu sur le topic /target_reached"""
        if msg.data == 1:
            self.get_logger().info('Target reached, moving to next point')
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
