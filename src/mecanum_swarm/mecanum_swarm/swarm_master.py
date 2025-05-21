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
        self.state_formation = 0
        self.running = True
        
        # Publishers
        self.master_pub = self.create_publisher(Int32, '/master', 10)
        self.formation_pub = self.create_publisher(Int32, '/formation', 10)

        self.athos_pub = self.create_publisher(Twist, '/Athos/cmd_vel', 10)
        self.aramis_pub = self.create_publisher(Twist, '/Aramis/cmd_vel', 10)
        self.porthos_pub = self.create_publisher(Twist, '/Porthos/cmd_vel', 10)
        
        self.get_logger().info("Nœud de contrôle maître démarré. Appuyez sur espace pour lancer les autres noeuds.")
        
        self.create_subscription(
            Int32, '/travel_finished', self.travel_finished_callback, 10
        )
    
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

    def toggle_state_formation(self):
        # Basculer l'état de formation
        self.state_formation = 1 if self.state_formation == 0 else 0
        
        # Publier l'état de formation
        msg = Int32()
        msg.data = self.state_formation
        self.get_logger().info(f"État de formation publié: {self.state_formation}")
        self.formation_pub.publish(msg)
    
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
        
    def travel_finished_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("Circuit terminé, arrêt de tous les robots.")
            stop_msg = Int32()
            stop_msg.data = 0
            self.master_pub.publish(stop_msg)
            self.state = 0  # Mettre à jour l'état interne si besoin
            self.send_twist_messages()
        
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

                if key == 'w':
                    self.toggle_state_formation()
                    self.get_logger().info("État de formation basculé")

                    
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
