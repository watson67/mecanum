#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import sys
import termios
import tty
import select
import threading

# Configuration des touches pour un clavier AZERTY avec commandes dans le repère global
key_mapping = {
    'z': (1, 0, 0, 0),     # Avancer dans le repère global (+X)
    's': (-1, 0, 0, 0),    # Reculer dans le repère global (-X)
    'q': (0, 1, 0, 0),     # Translation gauche dans le repère global (+Y)
    'd': (0, -1, 0, 0),    # Translation droite dans le repère global (-Y)
    'a': (0.707, 0.707, 0, 0),  # Diagonale avant-gauche dans le repère global
    'e': (0.707, -0.707, 0, 0),  # Diagonale avant-droit dans le repère global
    'w': (-0.707, 0.707, 0, 0),  # Diagonale arrière-gauche dans le repère global
    'x': (-0.707, -0.707, 0, 0),  # Diagonale arrière-droit dans le repère global
    'r': (0, 0, 0, 1),     # Rotation gauche dans le repère global
    't': (0, 0, 0, -1),    # Rotation droite dans le repère global
    ' ': (0, 0, 0, 0),     # Stop
}

def get_key():
    """Lit une touche clavier sans bloquer et restaure le terminal correctement."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(fd)
        dr, _, _ = select.select([sys.stdin], [], [], 0.01)
        if dr:
            key = sys.stdin.read(1)
        else:
            key = ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    return key

class GlobalFrameTeleop(Node):
    def __init__(self, topic_name, global_frame='odom', robot_frame='base_link'):
        super().__init__('global_frame_teleop')
        
        # Configuration des frames
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        
        # Création du publisher pour les commandes de vitesse
        self.publisher = self.create_publisher(Twist, topic_name, 1)
        
        # Configuration TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f'Global Frame Teleop prêt. Publie sur {topic_name}')
        self.get_logger().info(f'Utilise {self.global_frame} comme repère global et {self.robot_frame} comme repère du robot')
        
        self.running = True

    def transform_velocity(self, global_lin_x, global_lin_y, global_ang_z):
        """Transforme les vitesses du repère global vers le repère du robot en utilisant tf2."""
        try:
            # Assurer que les entrées sont des float
            global_lin_x = float(global_lin_x)
            global_lin_y = float(global_lin_y)
            global_ang_z = float(global_ang_z)
            
            # Créer un vecteur de vitesse linéaire avec un timestamp
            v_global = Vector3Stamped()
            v_global.header.frame_id = self.global_frame
            v_global.header.stamp = self.get_clock().now().to_msg()
            v_global.vector.x = global_lin_x
            v_global.vector.y = global_lin_y
            v_global.vector.z = 0.0

            # Transformer le vecteur du repère global au repère du robot
            v_robot = self.tf_buffer.transform(v_global, self.robot_frame, timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Extraire les composantes transformées
            robot_lin_x = v_robot.vector.x
            robot_lin_y = v_robot.vector.y
            
            # La vitesse angulaire reste la même car elle est autour de l'axe Z
            # Mais elle pourrait aussi être transformée si nécessaire
            robot_ang_z = global_ang_z
            
            return robot_lin_x, robot_lin_y, robot_ang_z
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException, TransformException) as e:
            self.get_logger().error(f'TF2:  {e}')
            return global_lin_x, global_lin_y, global_ang_z  # En cas d'erreur, renvoyer les vitesses globales

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
                    global_dx, global_dy, _, global_dth = key_mapping[key]
                    
                    # Transformer les vitesses du repère global au repère du robot
                    robot_dx, robot_dy, robot_dth = self.transform_velocity(global_dx, global_dy, global_dth)
                    
                    # Créer le message Twist avec les vitesses dans le repère du robot
                    twist.linear.x = robot_dx * 0.5
                    twist.linear.y = robot_dy * 0.5
                    twist.angular.z = robot_dth * 1.0

                    # Ajouter un log pour afficher les vitesses globales et les vitesses du robot
                    self.get_logger().info(
                        f'Global: linear.x={global_dx*0.5}, linear.y={global_dy*0.5}, angular.z={global_dth*1.0} -> '
                        f'Robot: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                    )
                    self.publisher.publish(twist)
                elif key:  # Si une touche non définie est pressée
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0

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
    rclpy.init(args=args)
    
    # Récupérer les paramètres de ligne de commande
    topic = sys.argv[1] if len(sys.argv) > 1 else '/cmd_vel'
    global_frame = sys.argv[2] if len(sys.argv) > 2 else 'odom'
    robot_frame = sys.argv[3] if len(sys.argv) > 3 else 'base_link'
    
    node = GlobalFrameTeleop(topic, global_frame, robot_frame)
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
