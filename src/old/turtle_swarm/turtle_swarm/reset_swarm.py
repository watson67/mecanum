#!/usr/bin/env python3

import rclpy
import sys
import os
import select
import termios
import tty
from rclpy.node import Node
from std_msgs.msg import Int8

class TurtleResetter(Node):
    def __init__(self):
        super().__init__('turtle_resetter')
        self.get_logger().info('Démarrage du contrôleur de réinitialisation des tortues')
        
        # Création du publisher pour le topic reset_turtlesim
        self.reset_publisher = self.create_publisher(Int8, 'reset_turtlesim', 10)
        
        # État actuel (0 ou 1)
        self.reset_state = 0
        
        # Sauvegarde des paramètres du terminal et configuration de l'entrée clavier
        try:
            self.is_tty = os.isatty(sys.stdin.fileno())
            if self.is_tty:
                self.old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                self.get_logger().info('Paramètres du terminal configurés pour l\'entrée clavier directe')
        except Exception as e:
            self.is_tty = False
            self.get_logger().warn(f'Échec de la configuration du terminal: {e}')
        
        # Création d'un minuteur pour vérifier l'entrée clavier
        self.create_timer(0.1, self.check_keyboard_input)
        
        self.get_logger().info('Appuyez sur ESPACE pour basculer l\'état de réinitialisation')
    
    def __del__(self):
        # Restauration des paramètres du terminal
        if hasattr(self, 'is_tty') and self.is_tty and hasattr(self, 'old_settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def check_keyboard_input(self):
        if not self.is_tty:
            return
            
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == ' ':  # Touche espace
                # Basculer l'état (0->1 ou 1->0)
                self.reset_state = 1 if self.reset_state == 0 else 0
                self.get_logger().info(f'Touche ESPACE pressée, état de réinitialisation: {self.reset_state}')
                
                # Publier l'état sur le topic
                msg = Int8()
                msg.data = self.reset_state
                self.reset_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleResetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'is_tty') and node.is_tty and hasattr(node, 'old_settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
