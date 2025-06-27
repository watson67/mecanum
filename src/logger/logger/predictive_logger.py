#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from std_msgs.msg import Int32
import csv
from datetime import datetime
import os
import sys
import glob
import time

from swarm_manager.config import ALL_ROBOT_NAMES

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
        self.latest_positions = {name: None for name in ALL_ROBOT_NAMES}
        self.latest_velocities = {name: None for name in ALL_ROBOT_NAMES}
        self.latest_cmd_vel = {name: None for name in ALL_ROBOT_NAMES}
        
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
                10
            )
        
        # Subscribers pour les vitesses actuelles (prédiction)
        self.velocity_subscribers = {}
        for name in ALL_ROBOT_NAMES:
            self.velocity_subscribers[name] = self.create_subscription(
                Vector3,
                f"/{name}/current_velocity", 
                self._make_velocity_callback(name),
                10
            )
        
        # Subscribers pour les commandes de vitesse
        self.cmd_vel_subscribers = {}
        for name in ALL_ROBOT_NAMES:
            self.cmd_vel_subscribers[name] = self.create_subscription(
                Twist,
                f"/{name}/cmd_vel",
                self._make_cmd_vel_callback(name),
                10
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
            'event_type',  # 'position_published', 'velocity_updated', 'cmd_vel_sent'
            'pos_x', 'pos_y',  # Position publiée
            'vel_x', 'vel_y',  # Vitesse de prédiction
            'cmd_vel_x', 'cmd_vel_y', 'cmd_vel_angular'  # Commande de vitesse
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
            
            # Stocker la dernière position
            self.latest_positions[robot_name] = {
                'x': msg.x,
                'y': msg.y,
                'timestamp': timestamp
            }
            
            # Enregistrer l'événement de publication de position
            self._log_event(robot_name, timestamp, 'position_published', 
                          pos_x=msg.x, pos_y=msg.y)
            
            self.get_logger().debug(f"Position publiée enregistrée pour {robot_name}: ({msg.x:.3f}, {msg.y:.3f})")
        
        return callback

    def _make_velocity_callback(self, robot_name):
        """Crée un callback pour les vitesses de prédiction d'un robot"""
        def callback(msg):
            if not self.active:
                return
            
            timestamp = datetime.now().isoformat()
            
            # Stocker la dernière vitesse
            self.latest_velocities[robot_name] = {
                'vx': msg.x,
                'vy': msg.y,
                'timestamp': timestamp
            }
            
            # Enregistrer l'événement de mise à jour de vitesse
            self._log_event(robot_name, timestamp, 'velocity_updated',
                          vel_x=msg.x, vel_y=msg.y)
            
            self.get_logger().debug(f"Vitesse enregistrée pour {robot_name}: ({msg.x:.3f}, {msg.y:.3f})")
        
        return callback

    def _make_cmd_vel_callback(self, robot_name):
        """Crée un callback pour les commandes de vitesse d'un robot"""
        def callback(msg):
            if not self.active:
                return
            
            timestamp = datetime.now().isoformat()
            
            # Stocker la dernière commande de vitesse
            self.latest_cmd_vel[robot_name] = {
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y,
                'angular_z': msg.angular.z,
                'timestamp': timestamp
            }
            
            # Enregistrer l'événement de commande de vitesse
            self._log_event(robot_name, timestamp, 'cmd_vel_sent',
                          cmd_vel_x=msg.linear.x, cmd_vel_y=msg.linear.y, 
                          cmd_vel_angular=msg.angular.z)
            
            self.get_logger().debug(f"Commande vitesse enregistrée pour {robot_name}: ({msg.linear.x:.3f}, {msg.linear.y:.3f})")
        
        return callback

    def _log_event(self, robot_name, timestamp, event_type, 
                   pos_x=None, pos_y=None, vel_x=None, vel_y=None, 
                   cmd_vel_x=None, cmd_vel_y=None, cmd_vel_angular=None):
        """Enregistre un événement dans le fichier CSV du robot"""
        
        try:
            # Récupérer les dernières valeurs connues si pas fournies
            latest_pos = self.latest_positions.get(robot_name, {})
            latest_vel = self.latest_velocities.get(robot_name, {})
            latest_cmd = self.latest_cmd_vel.get(robot_name, {})
            
            # Utiliser les valeurs fournies ou les dernières connues
            row_data = [
                timestamp,
                event_type,
                pos_x if pos_x is not None else latest_pos.get('x', 0.0),
                pos_y if pos_y is not None else latest_pos.get('y', 0.0),
                vel_x if vel_x is not None else latest_vel.get('vx', 0.0),
                vel_y if vel_y is not None else latest_vel.get('vy', 0.0),
                cmd_vel_x if cmd_vel_x is not None else latest_cmd.get('linear_x', 0.0),
                cmd_vel_y if cmd_vel_y is not None else latest_cmd.get('linear_y', 0.0),
                cmd_vel_angular if cmd_vel_angular is not None else latest_cmd.get('angular_z', 0.0)
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
