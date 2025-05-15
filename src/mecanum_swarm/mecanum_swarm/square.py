#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

class SquareTrajectory(Node):
    def __init__(self):
        super().__init__('square_trajectory')
        
        # Définir manuellement les coordonnées des sommets du carré
        # Format: Liste de tuples (x, y) pour chaque sommet
        # Les sommets seront visités dans l'ordre où ils sont définis
        self.vertices = [
            (0.5, 0.5),    # Premier sommet
            (-0.5, 0.5),   # Deuxième sommet
            (-0.5, -0.5),  # Troisième sommet
            (0.5, -0.5),   # Quatrième sommet
        ]
        
        # Index du sommet actuel
        self.current_vertex = 0
        
        # Publisher pour les nouveaux points cibles
        self.goal_publisher = self.create_publisher(
            Point, '/goal_point', 10
        )
        
        # Subscriber pour savoir quand le point est atteint
        self.create_subscription(
            Int32, '/target_reached', self.target_reached_callback, 10
        )
        
        self.get_logger().info('Square trajectory node initialized')
        self.get_logger().info(f'Vertices defined: {self.vertices}')
        
        # Publier le premier point dès le démarrage
        self.publish_next_point()
        
    def target_reached_callback(self, msg):
        """Callback appelé quand un message est reçu sur le topic /target_reached"""
        if msg.data == 1:
            self.get_logger().info('Target reached, moving to next point')
            # Le point est atteint, passer au suivant
            self.publish_next_point()
    
    def publish_next_point(self):
        """Publie le prochain sommet du carré"""
        # Obtenir les coordonnées du sommet actuel
        x, y = self.vertices[self.current_vertex]
        
        # Créer et publier le message
        point_msg = Point()
        point_msg.x = float(x)
        point_msg.y = float(y)
        point_msg.z = 0.0
        
        self.goal_publisher.publish(point_msg)
        self.get_logger().info(f'Published new goal point: vertex {self.current_vertex}: x={x:.4f}, y={y:.4f}')
        
        # Passer au sommet suivant
        self.current_vertex = (self.current_vertex + 1) % len(self.vertices)

def main(args=None):
    rclpy.init(args=args)
    node = SquareTrajectory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
