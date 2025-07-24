#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped, Vector3
from std_msgs.msg import Int32, Float64MultiArray
import math
import socket
import numpy as np
import time
from swarm_manager.formules import *
from swarm_manager.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS
from rclpy.qos import QoSProfile, ReliabilityPolicy

'''
Version distribuée du contrôleur d'essaim avec prédiction.
Chaque robot prédit les positions de ses voisins et publie sa position
seulement quand l'erreur de prédiction dépasse un seuil.
'''

#-----------------------------#
#   Paramètres globaux        #
#-----------------------------#
GLOBAL_FRAME = "mocap"
FIXED_DISTANCE = 0.5
CHOICE = False

# QoS optimisé pour communication
comm_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1
)

def quaternion_to_yaw(x, y, z, w):
    """Extrait l'angle yaw d'un quaternion"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def transform_velocity_manual(global_vx, global_vy, robot_yaw):
    """Transforme une vitesse du repère global vers le repère robot"""
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    
    robot_vx = cos_yaw * global_vx + sin_yaw * global_vy
    robot_vy = -sin_yaw * global_vx + cos_yaw * global_vy
    
    return robot_vx, robot_vy

class DistributedPredictiveSwarmController(Node):
    """
    Contrôleur distribué prédictif pour un robot de l'essaim.
    """
    def __init__(self):
        
        # Déterminer le nom du robot à partir du hostname
        hostname = socket.gethostname().lower()
        if hostname.endswith('-desktop'):
            hostname = hostname[:-8]
        self.robot_name = hostname[:1].upper() + hostname[1:]
        if self.robot_name not in ALL_ROBOT_NAMES:
            print(f"Warning: Robot name '{self.robot_name}' not in known robot list {ALL_ROBOT_NAMES}")
            self.robot_name = "Unknown"
        super().__init__(
            'distributed_predictive_swarm_controller',
            namespace=f'/{self.robot_name}'
        )
        self.get_logger().info(f"Starting distributed predictive controller for robot: {self.robot_name}")

        # Configuration statique des voisins
        self.neighbors_names = ROBOT_NEIGHBORS.get(self.robot_name, [])
        if not self.neighbors_names:
            self.neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        
        self.get_logger().info(f"Robot {self.robot_name} neighbors (static): {self.neighbors_names}")
        
        #-----------------------------#
        #    VRPN Subscriptions       #
        #-----------------------------#
        
        # Subscribe ONLY to own robot's VRPN pose
        self.create_subscription(
            PoseStamped,
            f"/vrpn_mocap/{self.robot_name}/pose",
            self.my_pose_callback,
            comm_qos
        )
        
        # Subscribe to global barycenter
        self.global_barycenter_position = None
        self.create_subscription(
            PoseStamped,
            "/vrpn_mocap/Barycenter/pose",
            self.barycenter_callback,
            comm_qos
        )
        
        #-----------------------------#
        #   Publishers pour compatibilité logger #
        #-----------------------------#
        
        # Topics compatibles avec le logger centralisé
        self.published_pose_publisher = self.create_publisher(
            Point, f"/{self.robot_name}/published_pose", comm_qos
        )
        self.velocity_publisher = self.create_publisher(
            Vector3, f"/{self.robot_name}/current_velocity", comm_qos
        )
        
        # SUPPRESSION des publishers d'estimation de voisins - pas nécessaires en mode distribué
        
        # Publisher pour composantes de contrôle (compatibilité logger)
        self.control_component_publisher = self.create_publisher(
            Float64MultiArray, f"/{self.robot_name}/control_components", 10
        )
        
        #-----------------------------#
        #   Communication inter-robots #
        #-----------------------------#
        
        # Subscribers pour recevoir les positions et vitesses des voisins
        self.neighbor_positions = {}  # Dernières positions publiées par les voisins
        self.neighbor_velocities = {}  # Dernières vitesses publiées par les voisins
        self.predicted_neighbor_positions = {}  # Positions prédites des voisins
        
        for neighbor_name in self.neighbors_names:
            # Subscribe to neighbor's published position
            self.create_subscription(
                Point, f"/{neighbor_name}/published_pose",
                lambda msg, name=neighbor_name: self.neighbor_position_callback(msg, name),
                comm_qos
            )
            
            # Subscribe to neighbor's velocity
            self.create_subscription(
                Vector3, f"/{neighbor_name}/current_velocity",
                lambda msg, name=neighbor_name: self.neighbor_velocity_callback(msg, name),
                comm_qos
            )
            
            # Initialize data structures
            self.neighbor_positions[neighbor_name] = {'x': 0.0, 'y': 0.0, 'timestamp': 0.0}
            self.neighbor_velocities[neighbor_name] = {'vx': 0.0, 'vy': 0.0, 'timestamp': 0.0}
            self.predicted_neighbor_positions[neighbor_name] = {'x': 0.0, 'y': 0.0}

        #-----------------------------#
        #   Publishers ROS2           #
        #-----------------------------#
        self.cmd_vel_publisher = self.create_publisher(
            Twist, f"/{self.robot_name}/cmd_vel", 10
        )
        self.target_status_publisher = self.create_publisher(
            Int32, f"/{self.robot_name}/target_status", 10
        )

        #-----------------------------#
        #   Subscribers ROS2          #
        #-----------------------------#
        self.create_subscription(Int32, "/master", self.master_callback, 10)
        self.create_subscription(Point, "/goal_point", self.goal_point_callback, 10)
        self.create_subscription(Int32, "/formation", self.formation_callback, 10)

        #-----------------------------#
        #   Variables d'état          #
        #-----------------------------#
        self.active = False
        self.my_pose = None
        self.my_position = {'x': 0.0, 'y': 0.0}
        self.my_velocity = {'vx': 0.0, 'vy': 0.0}
        self.my_yaw = 0.0
        self.goal_point = (0.0, 0.0)
        self.goal_point_set = False
        self.formation_initialized = False
        self.desired_distances = {}
        self.initial_relative_vector = None
        self.integral_term = None
        self.derivative_term = None
        self.previous_gamma = None
        self.dt = 0.1
        self.target_tolerance = 0.08
        self.is_target_reached_state = False

        #-----------------------------#
        #   Variables prédictives     #
        #-----------------------------#
        
        # Seuils pour la publication sélective
        self.prediction_error_threshold = 0.05  # Seuil d'erreur pour publier position (m)
        
        # États pour la prédiction
        self.last_publish_time = 0.0
        self.my_published_position = {'x': 0.0, 'y': 0.0, 'timestamp': 0.0}
        self.my_published_velocity = {'vx': 0.0, 'vy': 0.0, 'timestamp': 0.0}
        self.my_estimated_position = {'x': 0.0, 'y': 0.0}  # Mon estimation de ma propre position
        
        # SUPPRESSION du calcul de vitesse - on utilise celle du contrôleur
        self.my_control_velocity = {'vx': 0.0, 'vy': 0.0}  # Vitesse du contrôleur
        
        # NOUVEAUX - Contrôle strict des publications
        self.first_position_published = False
        self.stable_position_count = 0
        
        # Statistiques
        self.position_publications = 0
        self.prediction_updates = 0

        #-----------------------------#
        #   Timers ROS2               #
        #-----------------------------#
        self.create_timer(self.dt, self.timer_callback)

    #-----------------------------#
    #   VRPN Pose Callbacks       #
    #-----------------------------#
    
    def my_pose_callback(self, msg):
        """Callback pour la pose de ce robot depuis VRPN - SEULEMENT position"""
        self.my_pose = msg
        pos = msg.pose.position
        
        # SEULEMENT récupérer la position - PAS de calcul de vitesse
        self.my_position = {'x': pos.x, 'y': pos.y}
        
        quat = msg.pose.orientation
        self.my_yaw = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

    def barycenter_callback(self, msg):
        """Callback pour le barycentre global"""
        pos = msg.pose.position
        self.global_barycenter_position = {'x': pos.x, 'y': pos.y}

    #-----------------------------#
    #   Communication Callbacks   #
    #-----------------------------#
    
    def neighbor_position_callback(self, msg, neighbor_name):
        """Callback pour recevoir la position publiée d'un voisin"""
        current_time = time.time()
        self.neighbor_positions[neighbor_name] = {
            'x': msg.x,
            'y': msg.y,
            'timestamp': current_time
        }
        # Mettre à jour immédiatement l'estimation de ce voisin
        self.predicted_neighbor_positions[neighbor_name] = {
            'x': msg.x,
            'y': msg.y
        }
        self.get_logger().debug(f"Position reçue de {neighbor_name}: ({msg.x:.3f}, {msg.y:.3f})")

    def neighbor_velocity_callback(self, msg, neighbor_name):
        """Callback pour recevoir la vitesse d'un voisin"""
        current_time = time.time()
        self.neighbor_velocities[neighbor_name] = {
            'vx': msg.x,
            'vy': msg.y,
            'timestamp': current_time
        }
        self.get_logger().debug(f"Vitesse reçue de {neighbor_name}: ({msg.x:.3f}, {msg.y:.3f})")

    #-----------------------------#
    #   Callbacks ROS2            #
    #-----------------------------#
    def master_callback(self, msg):
        """Active ou désactive le contrôle"""
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info("Contrôle prédictif actif")
            # PUBLICATION INITIALE pour lancer l'algorithme proprement
            self.force_initial_publication()
        else:
            self.get_logger().info("Contrôle prédictif désactivé")
            self.stop_robot()

    def force_initial_publication(self):
        """Force la publication initiale pour lancer l'algorithme prédictif"""
        if abs(self.my_position['x']) > 0.001 or abs(self.my_position['y']) > 0.001:
            current_time = time.time()
            
            # Publier position immédiatement
            pos_msg = Point()
            pos_msg.x = self.my_position['x']
            pos_msg.y = self.my_position['y']
            pos_msg.z = 0.0
            self.published_pose_publisher.publish(pos_msg)
            
            # Publier vitesse initiale (probablement nulle)
            vel_msg = Vector3()
            vel_msg.x = self.my_control_velocity['vx']
            vel_msg.y = self.my_control_velocity['vy']
            vel_msg.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            
            # Mettre à jour les états
            self.my_published_position = {
                'x': self.my_position['x'],
                'y': self.my_position['y'],
                'timestamp': current_time
            }
            self.my_published_velocity = {
                'vx': self.my_control_velocity['vx'],
                'vy': self.my_control_velocity['vy'],
                'timestamp': current_time
            }
            
            # Marquer comme publié et réinitialiser les compteurs
            self.my_estimated_position = self.my_position.copy()
            self.first_position_published = True
            self.last_publish_time = current_time
            self.position_publications += 1
            
            self.get_logger().info(
                f"PUBLICATION INITIALE FORCÉE - pos({self.my_position['x']:.3f}, {self.my_position['y']:.3f}) "
                f"vel({self.my_control_velocity['vx']:.3f}, {self.my_control_velocity['vy']:.3f})"
            )
        else:
            self.get_logger().warn("Position non valide, publication initiale reportée")

    def should_publish_my_position(self):
        """Détermine si je dois publier ma position et vitesse - ALGORITHME SIMPLE"""
        
        # PLUS BESOIN d'attendre la stabilisation - la publication initiale est forcée
        if not self.first_position_published:
            # Vérifier que la position n'est pas à l'origine (données VRPN valides)
            if abs(self.my_position['x']) < 0.001 and abs(self.my_position['y']) < 0.001:
                return False
            
            # Pas d'attente - publication immédiate si pas encore fait
            self.first_position_published = True
            self.get_logger().info(f"PREMIÈRE PUBLICATION: pos({self.my_position['x']:.3f}, {self.my_position['y']:.3f})")
            return True
        
        # Calculer l'erreur entre position réelle et estimation basée sur vitesse
        position_error = math.sqrt(
            (self.my_position['x'] - self.my_estimated_position['x'])**2 + 
            (self.my_position['y'] - self.my_estimated_position['y'])**2
        )
        
        # CONDITION SIMPLE: Erreur de prédiction trop importante
        if position_error > self.prediction_error_threshold:
            self.get_logger().info(
                f"ERREUR PRÉDICTION: {position_error:.4f}m > {self.prediction_error_threshold}m"
                f" | pos_réel=({self.my_position['x']:.3f}, {self.my_position['y']:.3f})"
                f" | pos_estimé=({self.my_estimated_position['x']:.3f}, {self.my_estimated_position['y']:.3f})"
                f" | vitesse_contrôle=({self.my_control_velocity['vx']:.3f}, {self.my_control_velocity['vy']:.3f})"
            )
            return True
        
        return False

    def publish_my_position_and_velocity(self):
        """Publie ma position ET la vitesse du contrôleur"""
        if self.should_publish_my_position():
            current_time = time.time()
            
            # Publier position réelle
            pos_msg = Point()
            pos_msg.x = self.my_position['x']
            pos_msg.y = self.my_position['y']
            pos_msg.z = 0.0
            self.published_pose_publisher.publish(pos_msg)
            
            # Publier vitesse du contrôleur (pas calculée depuis positions)
            vel_msg = Vector3()
            vel_msg.x = self.my_control_velocity['vx']
            vel_msg.y = self.my_control_velocity['vy']
            vel_msg.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            
            # Mettre à jour les états SEULEMENT après publication
            self.my_published_position = {
                'x': self.my_position['x'],
                'y': self.my_position['y'],
                'timestamp': current_time
            }
            self.my_published_velocity = {
                'vx': self.my_control_velocity['vx'],
                'vy': self.my_control_velocity['vy'],
                'timestamp': current_time
            }
            
            # Mettre à jour mon estimation avec la vraie position
            self.my_estimated_position = self.my_position.copy()
            
            self.last_publish_time = current_time
            self.position_publications += 1
            
            self.get_logger().info(
                f"PUBLICATION ALGORITHME SIMPLE - pos({self.my_position['x']:.3f}, {self.my_position['y']:.3f}) "
                f"vel_contrôle({self.my_control_velocity['vx']:.3f}, {self.my_control_velocity['vy']:.3f}) [#{self.position_publications}]"
            )

    #-----------------------------#
    #   Boucle principale         #
    #-----------------------------#
    def timer_callback(self):
        """Boucle principale avec prédiction - AUCUNE publication automatique"""
        # Mettre à jour mon estimation de ma propre position
        self.update_my_estimated_position()
        
        # Prédire les positions des voisins
        self.predict_neighbor_positions()
        
        # Publier ma position et vitesse SEULEMENT si vraiment nécessaire
        self.publish_my_position_and_velocity()
        
        # Debug périodique avec vérification des publications
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 100 == 0:  # Toutes les 10 secondes
            error = math.sqrt(
                (self.my_position['x'] - self.my_estimated_position['x'])**2 + 
                (self.my_position['y'] - self.my_estimated_position['y'])**2
            )
            
            # Temps depuis dernière publication
            time_since_last = time.time() - self.last_publish_time if self.last_publish_time > 0 else 0
            
            # Vitesse actuelle
            current_speed = math.sqrt(self.my_control_velocity['vx']**2 + self.my_control_velocity['vy']**2)
            
            self.get_logger().info(
                f"STATS ANTI-BRUIT - Publications: {self.position_publications}, "
                f"Erreur: {error:.4f}m, Vitesse: {current_speed:.3f}m/s, "
                f"Temps depuis pub: {time_since_last:.1f}s"
            )
            
        # Initialisation de la formation
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.get_logger().info("Formation initialized with predictive positions")
        
        # Contrôle avec positions prédites
        if self.active and self.formation_initialized and self.goal_point_set:
            self.apply_consensus_control()

    def all_positions_available(self):
        """Vérifie si toutes les positions nécessaires sont disponibles"""
        if self.my_pose is None:
            return False
        if abs(self.my_position['x']) < 0.001 and abs(self.my_position['y']) < 0.001:
            return False
        if self.global_barycenter_position is None:
            return False
        
        # Vérifier qu'on a des données pour tous les voisins
        for neighbor_name in self.neighbors_names:
            if (neighbor_name not in self.neighbor_positions or 
                self.neighbor_positions[neighbor_name]['timestamp'] == 0.0):
                return False
        return True

    def initialize_formation(self):
        """Initialise la formation avec les positions prédites"""
        if self.formation_initialized:
            return
            
        initial_barycenter = self.compute_swarm_center()
        if initial_barycenter is None:
            return
            
        self.initial_relative_vector = np.array([
            self.my_position['x'] - initial_barycenter[0],
            self.my_position['y'] - initial_barycenter[1]
        ])
        
        # Configuration des distances désirées avec les voisins (utilise positions prédites)
        for neighbor_name in self.neighbors_names:
            if neighbor_name in self.predicted_neighbor_positions:
                other_pos = self.predicted_neighbor_positions[neighbor_name]
                dist = math.sqrt(
                    (self.my_position['x'] - other_pos['x'])**2 +
                    (self.my_position['y'] - other_pos['y'])**2
                ) if CHOICE else FIXED_DISTANCE
                self.desired_distances[neighbor_name] = dist
        
        self.formation_initialized = True
        self.get_logger().info(f"Formation initialized with predictive control")

    def compute_swarm_center(self):
        """Retourne le barycentre global"""
        if self.global_barycenter_position is not None:
            return [self.global_barycenter_position['x'], self.global_barycenter_position['y']]
        return None

    def transform_velocity(self, global_lin_x, global_lin_y):
        """Transforme une vitesse du repère global vers le repère du robot"""
        try:
            return transform_velocity_manual(
                float(global_lin_x), float(global_lin_y), self.my_yaw
            )
        except Exception as e:
            self.get_logger().error(f'Erreur de transformation: {e}')
            return global_lin_x, global_lin_y

    def apply_consensus_control(self):
        """Applique le contrôle de consensus avec positions prédites"""
        if not self.goal_point_set or self.global_barycenter_position is None:
            return
            
        global_goal = np.array(self.goal_point)
        pr = global_goal + self.initial_relative_vector
        
        # Utiliser mon estimation de ma propre position pour le contrôle
        pi = np.array([self.my_estimated_position['x'], self.my_estimated_position['y']])
        
        pj_array = []
        dij_list = []
        
        # Utiliser les positions prédites des voisins
        for neighbor_name in self.neighbors_names:
            if neighbor_name in self.predicted_neighbor_positions:
                other_pos = self.predicted_neighbor_positions[neighbor_name]
                pj = np.array([other_pos['x'], other_pos['y']])
                pj_array.append(pj)
                
                dij = self.desired_distances.get(neighbor_name)
                if dij is None:
                    dij = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                dij_list.append(dij)
        
        if not pj_array:
            control_vector = -(c1_gamma * (pi - pr))
            ui_alpha = np.array([0.0, 0.0])
            ui_gamma = control_vector
        else:
            try:
                control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma, updated_gamma = control_with_components(
                    pj_array=pj_array,
                    pi=pi,
                    dij_list=dij_list,
                    pr=pr,
                    dt=self.dt,
                    integral_term=self.integral_term,
                    derivative_term=self.derivative_term,
                    is_rotating=False,
                    logger=None,
                    previous_gamma=self.previous_gamma
                )
                self.previous_gamma = updated_gamma
                self.integral_term = updated_integral
                self.derivative_term = updated_derivative
            except TypeError:
                control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma = control_with_components(
                    pj_array=pj_array,
                    pi=pi,
                    dij_list=dij_list,
                    pr=pr,
                    dt=self.dt,
                    integral_term=self.integral_term,
                    derivative_term=self.derivative_term,
                    is_rotating=False,
                    logger=None,
                    previous_gamma=self.previous_gamma
                )
                self.previous_gamma = ui_gamma
                self.integral_term = updated_integral
                self.derivative_term = updated_derivative
        
        # Publier les composantes de contrôle pour la compatibilité logger
        control_msg = Float64MultiArray()
        control_msg.data = [float(ui_alpha[0]), float(ui_alpha[1]), 
                           float(ui_gamma[0]), float(ui_gamma[1])]
        self.control_component_publisher.publish(control_msg)
        
        # Transformer et publier la commande
        robot_lin_x, robot_lin_y = self.transform_velocity(
            control_vector[0], control_vector[1]
        )
        
        # CAPTURER la vitesse du contrôleur pour l'algorithme prédictif
        # Convertir en vitesse globale pour la prédiction
        cos_yaw = math.cos(self.my_yaw)
        sin_yaw = math.sin(self.my_yaw)
        
        # Transformation inverse : du repère robot vers global
        global_vx = cos_yaw * robot_lin_x - sin_yaw * robot_lin_y
        global_vy = sin_yaw * robot_lin_x + cos_yaw * robot_lin_y
        
        self.my_control_velocity = {'vx': global_vx, 'vy': global_vy}
        
        twist_msg = Twist()
        twist_msg.linear.x = float(robot_lin_x)
        twist_msg.linear.y = float(robot_lin_y)
        
        # Limiter la vitesse
        max_speed = 0.14
        speed = math.sqrt(twist_msg.linear.x**2 + twist_msg.linear.y**2)
        if speed > max_speed:
            scaling = max_speed / speed
            twist_msg.linear.x *= scaling
            twist_msg.linear.y *= scaling
            
            # Adapter aussi la vitesse de prédiction
            self.my_control_velocity['vx'] *= scaling
            self.my_control_velocity['vy'] *= scaling
        
        # Publier la commande
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Vérifier l'atteinte de la cible
        current_state = self.is_robot_target_reached(pi, pr)
        if current_state != self.is_target_reached_state:
            self.is_target_reached_state = current_state
            self.publish_target_status(1 if current_state else 0)

    def is_robot_target_reached(self, robot_pos, target_pos):
        """Vérifie si le robot a atteint sa cible"""
        distance = math.sqrt((robot_pos[0] - target_pos[0])**2 + (robot_pos[1] - target_pos[1])**2)
        return distance <= self.target_tolerance

    def publish_target_status(self, status):
        """Publie le statut d'atteinte de la cible"""
        msg = Int32()
        msg.data = status
        self.target_status_publisher.publish(msg)

    def stop_robot(self):
        """Arrête le robot"""
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        self.get_logger().info(f"Robot {self.robot_name} stopped")

def main(args=None):
    rclpy.init(args=args)
    node = DistributedPredictiveSwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
