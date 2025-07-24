#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Int32
import csv
from datetime import datetime
import os
import sys
import glob
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy

from swarm_manager.config import ALL_ROBOT_NAMES

# QoS compatible avec les publishers des contrôleurs distribués
comm_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1
)

class PredictiveLogger(Node):
    def __init__(self):
        super().__init__('predictive_logger')
        
        # Créer le répertoire CSV pour le mode prédictif
        self.csv_dir = os.path.expanduser('~/mecanum/csv/predictive')
        os.makedirs(self.csv_dir, exist_ok=True)
        
        # Nettoyer les anciens fichiers CSV
        self._cleanup_old_csvs()
        
        # Chemins des fichiers CSV pour chaque robot
        self.csv_paths = {
            name: os.path.join(self.csv_dir, f"{name}_predictive_data.csv") 
            for name in ALL_ROBOT_NAMES
        }
        
        # Stockage temporaire des données pour synchronisation
        self.latest_velocities = {name: None for name in ALL_ROBOT_NAMES}
        
        # État d'activation
        self.active = False
        
        # Initialiser les fichiers CSV
        self._init_csvs()
        
        # Subscribers pour les positions publiées
        self.position_subscribers = {}
        for name in ALL_ROBOT_NAMES:
            self.position_subscribers[name] = self.create_subscription(
                Point, 
                f"/{name}/published_pose",
                self._make_position_callback(name),
                comm_qos  # Use compatible QoS
            )
        
        # Subscribers pour les vitesses actuelles (prédiction)
        self.velocity_subscribers = {}
        for name in ALL_ROBOT_NAMES:
            self.velocity_subscribers[name] = self.create_subscription(
                Vector3,
                f"/{name}/current_velocity", 
                self._make_velocity_callback(name),
                comm_qos  # Use compatible QoS
            )
        
        # Subscriber pour le contrôle d'activation
        self.create_subscription(
            Int32,
            "/master",
            self.master_callback,
            10
        )

    def _cleanup_old_csvs(self):
        """Supprime tous les fichiers CSV existants se terminant par predictive_data.csv"""
        pattern = os.path.join(self.csv_dir, "*predictive_data.csv")
        csv_files = glob.glob(pattern)
        for csv_file in csv_files:
            try:
                os.remove(csv_file)
                self.get_logger().info(f"Fichier CSV supprimé : {csv_file}")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la suppression du fichier {csv_file} : {e}")

    def _init_csvs(self):
        """Initialise les fichiers CSV avec les en-têtes"""
        headers = [
            'timestamp',
            'pos_x', 'pos_y',  # Position publiée
            'vel_x', 'vel_y'   # Vitesse de prédiction
        ]
        
        for name in ALL_ROBOT_NAMES:
            try:
                with open(self.csv_paths[name], 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(headers)
                self.get_logger().info(f"Fichier CSV initialisé : {self.csv_paths[name]}")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la création du CSV pour {name} : {e}")

    def master_callback(self, msg):
        """Callback pour activer/désactiver l'enregistrement"""
        if msg.data == 1:
            self.get_logger().info("Activation reçue sur /master, réinitialisation des fichiers CSV prédictifs.")
            self._init_csvs()
            self.active = True
        elif msg.data == 0:
            self.get_logger().info("Désactivation reçue sur /master, arrêt de l'écriture dans les CSV prédictifs.")
            self.active = False

    def _make_position_callback(self, robot_name):
        """Crée un callback pour les positions publiées d'un robot"""
        def callback(msg):
            if not self.active:
                return
            
            timestamp = datetime.now().isoformat()
            
            # Récupérer la dernière vitesse connue
            latest_vel = self.latest_velocities.get(robot_name)
            if latest_vel is not None:
                vel_x = latest_vel.get('vx', 0.0)
                vel_y = latest_vel.get('vy', 0.0)
            else:
                vel_x = 0.0
                vel_y = 0.0
            
            # Enregistrer position et vitesse sur la même ligne
            self._log_data(robot_name, timestamp, msg.x, msg.y, vel_x, vel_y)
            
            self.get_logger().debug(f"Données enregistrées pour {robot_name}: pos({msg.x:.3f}, {msg.y:.3f}), vel({vel_x:.3f}, {vel_y:.3f})")
        
        return callback

    def _make_velocity_callback(self, robot_name):
        """Crée un callback pour les vitesses de prédiction d'un robot"""
        def callback(msg):
            # Toujours stocker la dernière vitesse, même si pas actif
            # pour qu'elle soit disponible quand l'enregistrement démarre
            timestamp = datetime.now().isoformat()
            
            # Stocker la dernière vitesse
            self.latest_velocities[robot_name] = {
                'vx': msg.x,
                'vy': msg.y,
                'timestamp': timestamp
            }
            
            if self.active:
                self.get_logger().debug(f"Vitesse mise à jour pour {robot_name}: ({msg.x:.3f}, {msg.y:.3f})")
        
        return callback

    def _log_data(self, robot_name, timestamp, pos_x, pos_y, vel_x, vel_y):
        """Enregistre les données dans le fichier CSV du robot"""
        
        try:
            row_data = [
                timestamp,
                pos_x,
                pos_y,
                vel_x,
                vel_y
            ]
            
            with open(self.csv_paths[robot_name], 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row_data)
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'écriture dans le CSV pour {robot_name} : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PredictiveLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
