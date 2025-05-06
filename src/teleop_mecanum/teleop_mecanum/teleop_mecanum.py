#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading

# Configuration des touches pour un clavier AZERTY avec boutons dédiés pour les diagonales, virages et déplacements horizontaux
key_mapping = {
    'z': (0.14, 0.0, 0.0, 0.0),    # Avancer
    's': (-0.14, 0.0, 0.0, 0.0),   # Reculer
    'a': (0.099, 0.099, 0.0, 0.0),  # Diagonale avant-gauche
    'e': (0.099, -0.099, 0.0, 0.0),  # Diagonale avant-droit
    'w': (-0.099, 0.099, 0.0, 0.0),  # Diagonale arrière-gauche
    'x': (-0.099, -0.099, 0.0, 0.0),  # Diagonale arrière-droit
    'r': (0.0, 0.0, 0.0, 1.0),    # Rotation gauche sur place
    't': (0.0, 0.0, 0.0, -1.0),   # Rotation droite sur place
    'f': (0.07, 0.0, 0.0, 1.0),  # Virage avant-gauche
    'g': (0.07, 0.0, 0.0, -1.0), # Virage avant-droit
    'c': (-0.07, 0.0, 0.0, 1.0), # Virage arrière-gauche
    'v': (-0.07, 0.0, 0.0, -1.0),# Virage arrière-droit
    'q': (0.0, 0.14, 0.0, 0.0),    # Translation gauche
    'd': (0.0, -0.14, 0.0, 0.0),   # Translation droite
    ' ': (0.0, 0.0, 0.0, 0.0),    # Stop
}

def get_key():
    """Lit une touche clavier sans bloquer et restaure le terminal correctement."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(fd)
        dr, _, _ = select.select([sys.stdin], [], [], 0.01)  # Réduire le délai ici
        if dr:
            key = sys.stdin.read(1)  # Lire une seule touche à la fois
        else:
            key = ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Toujours restaurer les réglages
    
    return key

class TeleopMecanum(Node):
    def __init__(self, topic_name):
        super().__init__('teleop_mecanum')
        self.publisher = self.create_publisher(Twist, topic_name, 1)
        self.get_logger().info(f'Teleop Mecanum prêt. Publie sur {topic_name}')
        self.running = True

    def spin_thread(self):
        """Thread pour exécuter rclpy.spin."""
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.01)

    def run(self):
        spin_thread = threading.Thread(target=self.spin_thread)
        spin_thread.start()

        try:
            while rclpy.ok():
                key = get_key()
                twist = Twist()

                if key == '\x03':  # CTRL+C pour quitter
                    break

                if key in key_mapping:
                    dx, dy, _, dth = key_mapping[key]
                    twist.linear.x = dx
                    twist.linear.y = dy
                    twist.angular.z = dth * 1.0

                    # Ajouter un log pour afficher les valeurs publiées
                    self.get_logger().info(
                        f'Publication Twist: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                    )
                    self.publisher.publish(twist)
                elif key:  # Si une touche non définie est pressée
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0

                    # Ajouter un log pour afficher les valeurs publiées
                    self.get_logger().info(
                        f'Publication Twist: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                    )
                    self.publisher.publish(twist)
                # Si aucune touche n'est pressée, ne rien envoyer
        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')
        finally:
            self.running = False
            spin_thread.join()

def main(args=None):
    rclpy.init()
    topic = sys.argv[1] if len(sys.argv) > 1 else '/cmd_vel'
    node = TeleopMecanum(topic)
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
