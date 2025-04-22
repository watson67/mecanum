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
    'z': (1, 0, 0, 0),    # Avancer
    's': (-1, 0, 0, 0),   # Reculer
    'a': (0.707, 0.707, 0, 0),  # Diagonale avant-gauche
    'e': (0.707, -0.707, 0, 0),  # Diagonale avant-droit
    'w': (-0.707, 0.707, 0, 0),  # Diagonale arrière-gauche
    'x': (-0.707, -0.707, 0, 0),  # Diagonale arrière-droit
    'r': (0, 0, 0, 1),    # Rotation gauche sur place
    't': (0, 0, 0, -1),   # Rotation droite sur place
    'f': (0.5, 0, 0, 1),  # Virage avant-gauche
    'g': (0.5, 0, 0, -1), # Virage avant-droit
    'c': (-0.5, 0, 0, 1), # Virage arrière-gauche
    'v': (-0.5, 0, 0, -1),# Virage arrière-droit
    'q': (0, 1, 0, 0),    # Translation gauche
    'd': (0, -1, 0, 0),   # Translation droite
    ' ': (0, 0, 0, 0),    # Stop
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
    def __init__(self):
        super().__init__('teleop_mecanum')
        self.topic_publishers = {  # Renommer publishers en topic_publishers
            'porthos': self.create_publisher(Twist, '/Porthos/cmd_vel', 1),
            'athos': self.create_publisher(Twist, '/Athos/cmd_vel', 1),
            'aramis': self.create_publisher(Twist, '/Aramis/cmd_vel', 1),
        }
        self.get_logger().info('Teleop Mecanum prêt. Publie sur /Porthos/cmd_vel, /Athos/cmd_vel, /Aramis/cmd_vel')
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
                    twist.linear.x = dx * 0.5
                    twist.linear.y = dy * 0.5
                    twist.angular.z = dth * 1.0

                    # Ajouter un log pour afficher les valeurs publiées
                    self.get_logger().info(
                        f'Publication Twist: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                    )
                    for pub in self.topic_publishers.values():  # Utiliser topic_publishers
                        pub.publish(twist)
                elif key:  # Si une touche non définie est pressée
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0

                    # Ajouter un log pour afficher les valeurs publiées
                    self.get_logger().info(
                        f'Publication Twist: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                    )
                    for pub in self.topic_publishers.values():  # Utiliser topic_publishers
                        pub.publish(twist)
                # Si aucune touche n'est pressée, ne rien envoyer
        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')
        finally:
            self.running = False
            spin_thread.join()

def main(args=None):
    rclpy.init()
    node = TeleopMecanum()
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
