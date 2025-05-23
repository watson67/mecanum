#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading

def get_key():
    """Lit une touche clavier sans bloquer."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(fd)
        dr, _, _ = select.select([sys.stdin], [], [], 0.01)  # Délai court pour ne pas bloquer
        if dr:
            key = sys.stdin.read(1)  # Lire une seule touche à la fois
        else:
            key = ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  
    
    return key

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_control_node')
        
        # État actuel (1 ou 0)
        self.state = 0
        self.running = True
        
        # Publishers
        self.master_pub = self.create_publisher(Int32, '/master', 10)
        self.athos_pub = self.create_publisher(Twist, '/Athos/cmd_vel', 10)
        self.aramis_pub = self.create_publisher(Twist, '/Aramis/cmd_vel', 10)
        self.porthos_pub = self.create_publisher(Twist, '/Porthos/cmd_vel', 10)
        
        self.get_logger().info("Nœud de contrôle maître démarré. Appuyez sur espace pour lancer les autres noeuds.")
    
    def toggle_state(self):
        # Basculer l'état
        self.state = 1 if self.state == 0 else 0
        
        # Publier l'état
        msg = Int32()
        msg.data = self.state
        self.master_pub.publish(msg)
        self.get_logger().info(f"État publié: {self.state}")
        
        # Si l'état est 0, envoyer des messages Twist
        if self.state == 0:
            self.send_twist_messages()
    
    def send_twist_messages(self):
        twist_msg = Twist()
        # Configurer le message Twist selon les besoins
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        
        # Envoyer à tous les robots
        self.athos_pub.publish(twist_msg)
        self.aramis_pub.publish(twist_msg)
        self.porthos_pub.publish(twist_msg)
        
        self.get_logger().info("Messages Twist envoyés à tous les robots pour les stopper")
        
    def spin_thread(self):
        """Thread pour exécuter rclpy.spin."""
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.01)
            
    def run(self):
        # Démarrer le thread de spinning
        spin_thread = threading.Thread(target=self.spin_thread)
        spin_thread.start()
        
        try:
            while rclpy.ok() and self.running:
                key = get_key()
                
                if key == '\x03':  # CTRL+C pour quitter
                    self.running = False
                    break
                
                if key == ' ':  # Touche espace
                    self.toggle_state()
                    
        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')
        finally:
            self.running = False
            spin_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
