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
import os
import csv
from datetime import datetime
import math
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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

GLOBAL_FRAME = "mocap"
PAIRS = [("Aramis", "Athos"), ("Aramis", "Porthos"), ("Athos", "Porthos")]
ALL_ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]

class MasterNode(Node):
    def __init__(self, mode='classic', csv_filename='distances_master.csv'):
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

        # CSV logging setup
        self.mode = mode
        self.csv_dir = os.path.expanduser(f'~/mecanum/csv/{mode}')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_path = os.path.join(self.csv_dir, csv_filename)
        self._init_csv()
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def _init_csv(self):
        header = ['timestamp']
        for a, b in PAIRS:
            header.append(f"{a}_{b}_distance")
        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(header)
            self.get_logger().info(f"Fichier CSV initialisé : {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la création du CSV : {e}")

    def log_distances(self):
        positions = {}
        try:
            for name in ALL_ROBOT_NAMES:
                trans = self.tf_buffer.lookup_transform(
                    GLOBAL_FRAME,
                    f"{name}/base_link",
                    rclpy.time.Time()
                )
                pos = trans.transform.translation
                positions[name] = (pos.x, pos.y, pos.z)
        except Exception:
            self.get_logger().warn("Impossible de récupérer toutes les positions robots pour le log CSV.")
            return

        now = datetime.now().isoformat()
        row = [now]
        for a, b in PAIRS:
            ax, ay, az = positions[a]
            bx, by, bz = positions[b]
            dist = math.sqrt((ax-bx)**2 + (ay-by)**2)
            row.append(dist)
        try:
            with open(self.csv_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row)
            self.get_logger().info("Distances entre robots enregistrées dans le CSV.")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'écriture dans le CSV : {e}")

    def toggle_state(self):
        # Basculer l'état
        self.state = 1 if self.state == 0 else 0
        
        if self.state == 1:
            self.get_logger().info("État basculé à 1, réinitisalisation csv.")
            self._init_csv()
        # Publier l'état
        msg = Int32()
        msg.data = self.state
        self.master_pub.publish(msg)
        self.get_logger().info(f"État publié: {self.state}")
        
        # Si l'état est 0, envoyer des messages Twist
        if self.state == 0:
            self.send_twist_messages()

        self.log_distances()

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
    # Parse mode and filename from sys.argv
    mode = 'classic'
    csv_filename = 'distances_master.csv'
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    if len(sys.argv) > 2:
        csv_filename = sys.argv[2]
    node = MasterNode(mode, csv_filename)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
