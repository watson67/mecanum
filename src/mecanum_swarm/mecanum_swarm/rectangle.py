#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Point

class SquareTrajectory(Node):
    def __init__(self):
        super().__init__('rectangle_trajectory')
        
        # Définir manuellement les coordonnées des sommets du carré
        # Format: Liste de tuples (x, y) pour chaque sommet
        # Les sommets seront visités dans l'ordre où ils sont définis
        self.vertices = [
            (-0.64, -0.77),    # Premier sommet
            (0.64, -0.77),     # Deuxième sommet
            (0.64, 0.4),       # Troisième sommet
            (-0.64, 0.4),      # Quatrième sommet
            (-0.64, -0.77),    # Premier sommet pour boucler
        ]
        
        # Index du sommet actuel
        self.current_vertex = 0
        self.last_point_sent = False  # Ajout d'un flag pour la logique de fin
        
        # Publisher pour les nouveaux points cibles
        self.goal_publisher = self.create_publisher(
            Point, '/goal_point', 10
        )
        
        # Publisher pour indiquer que le carré est terminé
        self.square_finished_pub = self.create_publisher(Int32, '/travel_finished', 10)
        
        # Publisher pour le type de trajectoire
        self.trajectory_type_pub = self.create_publisher(
            String, '/trajectory_type', 10
        )
        self.trajectory_type = "rectangle"
        # Publier le type de trajectoire au démarrage
        msg = String()
        msg.data = self.trajectory_type
        self.trajectory_type_pub.publish(msg)

        # Republier le type après un court délai pour garantir la réception
        self.create_timer(0.5, self.publish_trajectory_type_once, callback_group=None)
        self.trajectory_type_published = False

        # Subscriber pour savoir quand le point est atteint
        self.create_subscription(
            Int32, '/target_reached', self.target_reached_callback, 10
        )
        self.create_subscription(
            Int32, '/master', self.master_callback, 10
        )
        
        self.get_logger().info('Rectangle trajectory node initialized')
        self.get_logger().info(f'Vertices defined: {self.vertices}')
      
        # self.publish_next_point()
        
    def publish_trajectory_type_once(self):
        if not self.trajectory_type_published:
            msg = String()
            msg.data = self.trajectory_type
            self.trajectory_type_pub.publish(msg)
            self.get_logger().info(f"Republished trajectory type: {self.trajectory_type}")
            self.trajectory_type_published = True

    def master_callback(self, msg):
        if msg.data == 1:
            # Republier le type de trajectoire à chaque activation
            msg_type = String()
            msg_type.data = self.trajectory_type
            self.trajectory_type_pub.publish(msg_type)
            self.get_logger().info(f"Republished trajectory type on /master: {self.trajectory_type}")
            # Publier le premier point cible lors de l'activation
            self.current_vertex = 0
            self.last_point_sent = False
            self.publish_next_point()

    def target_reached_callback(self, msg):
        """Callback appelé quand un message est reçu sur le topic /target_reached"""
        if msg.data == 1:
            self.get_logger().info('Target reached, moving to next point')
            if self.last_point_sent:
                # Tous les points ont été atteints, publier la fin
                msg = Int32()
                msg.data = 1
                self.square_finished_pub.publish(msg)
                self.get_logger().info("Rectangle terminé, publication sur /travel_finished (1)")
                msg_zero = Int32()
                msg_zero.data = 0
                self.square_finished_pub.publish(msg_zero)
                self.get_logger().info("Publication de 0 sur /travel_finished")
                self.last_point_sent = False  # Reset pour un éventuel redémarrage
            else:
                self.publish_next_point()
    
    def publish_next_point(self):
        """Publie le prochain sommet du rectangle"""
        if self.current_vertex < len(self.vertices):
            x, y = self.vertices[self.current_vertex]
            
            # Créer et publier le message
            point_msg = Point()
            point_msg.x = float(x)
            point_msg.y = float(y)
            point_msg.z = 0.0
            
            self.goal_publisher.publish(point_msg)
            self.get_logger().info(f'Published new goal point: vertex {self.current_vertex}: x={x:.4f}, y={y:.4f}')
            
            self.current_vertex += 1
            
            # Si c'était le dernier point à envoyer, activer le flag
            if self.current_vertex == len(self.vertices):
                self.last_point_sent = True

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
