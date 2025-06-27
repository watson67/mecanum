#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import Int32, Float64MultiArray
import math
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3Stamped
import tf2_geometry_msgs
from tf2_ros import TransformException
import socket
import time
# Import formules.py
from swarm_manager.formules import *
from swarm_manager.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

'''
Version distribuée du contrôleur d'essaim prédictif avec optimisation des communications.
Ce programme s'exécute sur chaque robot individuellement.
Stratégie: Chaque robot prédit la position de ses voisins et ne publie sa position
que lorsque l'erreur de prédiction dépasse un seuil.
'''

GLOBAL_FRAME = "mocap"

class DistributedPredictiveSwarmController(Node):
    def __init__(self):
        # Déterminer le nom du robot à partir du hostname
        hostname = socket.gethostname().lower()
        if hostname.endswith('-desktop'):
            hostname = hostname[:-8]
        self.robot_name = hostname.capitalize()
        
        if self.robot_name not in ALL_ROBOT_NAMES:
            print(f"Warning: Robot name '{self.robot_name}' not in known robot list {ALL_ROBOT_NAMES}")
            self.robot_name = "Unknown"
        
        super().__init__(f'distributed_predictive_swarm_controller_{self.robot_name.lower()}')
        self.get_logger().info(f"Starting distributed predictive swarm controller for robot: {self.robot_name}")

        #--------------------------------------------------------------------
        # Variables TF2 pour les positions des robots 
        #--------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #--------------------------------------------------------------------
        # Publishers pour ce robot
        #--------------------------------------------------------------------
        # Publisher pour contrôler ce robot
        self.cmd_vel_publisher = self.create_publisher(
            Twist, f"/{self.robot_name}/cmd_vel", 10
        )
        
        # Publisher pour publier ma position (mise à jour sélective)
        self.published_pose_publisher = self.create_publisher(
            Point, f"/{self.robot_name}/published_pose", 10
        )
        
        # Publisher pour publier ma vitesse actuelle
        self.velocity_publisher = self.create_publisher(
            Vector3, f"/{self.robot_name}/current_velocity", 10
        )
        
        # Publisher pour les composantes de contrôle
        self.control_component_publisher = self.create_publisher(
            Float64MultiArray, f"/{self.robot_name}/control_components", 10
        )
        
        # Publisher pour le statut d'atteinte de cible
        self.target_status_publisher = self.create_publisher(
            Int32, f"/{self.robot_name}/target_status", 10
        )

        #--------------------------------------------------------------------
        # Subscribers pour les positions et vitesses des autres robots
        #--------------------------------------------------------------------
        self.position_subscribers = {}
        self.velocity_subscribers = {}
        
        for name in ALL_ROBOT_NAMES:
            if name != self.robot_name:  # Ne pas s'abonner à ses propres topics
                # Souscrire aux positions publiées des autres robots
                self.position_subscribers[name] = self.create_subscription(
                    Point, f"/{name}/published_pose", 
                    lambda msg, robot=name: self.neighbor_position_callback(msg, robot), 10
                )
                # Souscrire aux vitesses des autres robots
                self.velocity_subscribers[name] = self.create_subscription(
                    Vector3, f"/{name}/current_velocity",
                    lambda msg, robot=name: self.neighbor_velocity_callback(msg, robot), 10
                )

        #--------------------------------------------------------------------
        # Subscribers pour le contrôle général
        #--------------------------------------------------------------------
        self.create_subscription(Int32, "/master", self.master_callback, 10)
        self.create_subscription(Point, "/goal_point", self.goal_point_callback, 10)
        self.create_subscription(Int32, "/formation", self.formation_callback, 10)

        #--------------------------------------------------------------------
        # Variables de classe pour la prédiction
        #--------------------------------------------------------------------
        
        # Ma position réelle (via TF2)
        self.my_real_position = {'x': 0.0, 'y': 0.0}
        
        # Ma dernière position publiée
        self.my_published_position = {'x': 0.0, 'y': 0.0, 'timestamp': 0.0}
        
        # Ma vitesse actuelle
        self.my_current_velocity = {'vx': 0.0, 'vy': 0.0, 'timestamp': 0.0}
        
        # Positions publiées par les autres robots (dernière mise à jour reçue)
        self.neighbors_published_positions = {}
        for name in ALL_ROBOT_NAMES:
            if name != self.robot_name:
                self.neighbors_published_positions[name] = {'x': 0.0, 'y': 0.0, 'timestamp': 0.0}
        
        # Vitesses actuelles des autres robots
        self.neighbors_current_velocities = {}
        for name in ALL_ROBOT_NAMES:
            if name != self.robot_name:
                self.neighbors_current_velocities[name] = {'vx': 0.0, 'vy': 0.0, 'timestamp': 0.0}
        
        # Positions prédites des autres robots (calculées localement)
        self.neighbors_predicted_positions = {}
        for name in ALL_ROBOT_NAMES:
            if name != self.robot_name:
                self.neighbors_predicted_positions[name] = {'x': 0.0, 'y': 0.0}
        
        # Dernière fois que j'ai publié ma position
        self.last_publish_time = 0.0
        
        # Seuil d'erreur pour déclencher une mise à jour de position (en mètres)
        self.prediction_error_threshold = 0.10
        
        # Fréquence maximale de publication (pour éviter le spam)
        self.min_publish_interval = 0.1  # 100ms minimum entre publications

        #--------------------------------------------------------------------
        # Variables pour le contrôle de consensus
        #--------------------------------------------------------------------
        self.active = False
        self.desired_distances = {}
        self.initial_relative_vector = None  # Mon vecteur relatif initial
        self.goal_point = (0.0, 0.0)
        self.goal_point_set = False
        self.formation_initialized = False
        self.integral_term = None
        self.derivative_term = None
        self.dt = 0.1
        self.target_tolerance = 0.05
        self.is_target_reached_state = False
        
        # Configuration des voisins
        self.robot_neighbors = ROBOT_NEIGHBORS
        
        # Timer pour recharger la configuration des voisins
        self.create_timer(2.0, self.reload_neighbor_config)
        
        # Timer principal pour le contrôle et la prédiction
        self.create_timer(self.dt, self.timer_callback)
        
        # Statistiques de communication
        self.position_updates_count = 0
        self.total_predictions = 0

    def reload_neighbor_config(self):
        """Recharger la configuration des voisins depuis le fichier YAML"""
        try:
            import importlib
            import swarm_manager.config
            importlib.reload(swarm_manager.config)
            from swarm_manager.config import ROBOT_NEIGHBORS
            
            self.robot_neighbors = ROBOT_NEIGHBORS
            self.get_logger().info(f"Configuration des voisins rechargée: {self.robot_neighbors}")
            
        except Exception as e:
            self.get_logger().warn(f"Impossible de recharger la configuration des voisins: {e}")
            self.robot_neighbors = {robot: [r for r in ALL_ROBOT_NAMES if r != robot] for robot in ALL_ROBOT_NAMES}

    def neighbor_position_callback(self, msg, robot_name):
        """Callback pour recevoir les positions publiées des voisins"""
        if robot_name in self.neighbors_published_positions:
            current_time = time.time()
            
            self.neighbors_published_positions[robot_name] = {
                'x': msg.x,
                'y': msg.y,
                'timestamp': current_time
            }
            
            self.get_logger().debug(f"Position reçue de {robot_name}: ({msg.x:.3f}, {msg.y:.3f})")

    def neighbor_velocity_callback(self, msg, robot_name):
        """Callback pour recevoir les vitesses des voisins"""
        if robot_name in self.neighbors_current_velocities:
            current_time = time.time()
            
            self.neighbors_current_velocities[robot_name] = {
                'vx': msg.x,
                'vy': msg.y,
                'timestamp': current_time
            }
            
            self.get_logger().debug(f"Vitesse reçue de {robot_name}: ({msg.x:.3f}, {msg.y:.3f})")

    def update_my_real_position(self):
        """Mettre à jour ma position réelle via TF2"""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"{self.robot_name}/base_link", rclpy.time.Time()
            )
            pos = trans.transform.translation
            self.my_real_position = {'x': pos.x, 'y': pos.y}

        except Exception as e:
            self.get_logger().warn(f"Echec TF2 {self.robot_name}: {e}")

    def predict_neighbors_positions(self):
        """Prédire les positions des voisins basées sur leur vitesse"""
        current_time = time.time()
        
        for neighbor_name in self.neighbors_predicted_positions.keys():
            # Récupérer la dernière position publiée et vitesse
            pub_pos = self.neighbors_published_positions[neighbor_name]
            velocity = self.neighbors_current_velocities[neighbor_name]
            
            # Calculer le temps écoulé depuis la dernière position publiée
            dt_prediction = current_time - pub_pos['timestamp']
            
            if dt_prediction > 0 and velocity['timestamp'] > 0:
                # Prédire la position actuelle basée sur la vitesse
                predicted_x = pub_pos['x'] + velocity['vx'] * dt_prediction
                predicted_y = pub_pos['y'] + velocity['vy'] * dt_prediction
                
                self.neighbors_predicted_positions[neighbor_name] = {
                    'x': predicted_x,
                    'y': predicted_y
                }
                
                self.total_predictions += 1
                
                self.get_logger().debug(
                    f"Prédiction {neighbor_name}: ({predicted_x:.3f}, {predicted_y:.3f}) "
                    f"dt={dt_prediction:.3f}s"
                )
            else:
                # Utiliser la dernière position connue si pas de données
                self.neighbors_predicted_positions[neighbor_name] = {
                    'x': pub_pos['x'],
                    'y': pub_pos['y']
                }

    def should_publish_my_position(self):
        """Déterminer si je dois publier ma position"""
        current_time = time.time()
        
        # Respecter l'intervalle minimum entre publications
        if current_time - self.last_publish_time < self.min_publish_interval:
            return False
        
        # Calculer l'erreur de prédiction que les autres auraient sur ma position
        # (basée sur ma vitesse et le temps écoulé)
        dt_since_last_publish = current_time - self.my_published_position['timestamp']
        
        if dt_since_last_publish > 0:
            # Position que les autres prédisent pour moi
            predicted_x = self.my_published_position['x'] + self.my_current_velocity['vx'] * dt_since_last_publish
            predicted_y = self.my_published_position['y'] + self.my_current_velocity['vy'] * dt_since_last_publish
            
            # Erreur entre ma vraie position et ce que les autres prédisent
            error = math.sqrt(
                (self.my_real_position['x'] - predicted_x)**2 + 
                (self.my_real_position['y'] - predicted_y)**2
            )
            
            # Publier si l'erreur dépasse le seuil
            return error > self.prediction_error_threshold
        
        return False

    def publish_selective_position_and_velocity(self):
        """Publier ma position et vitesse de manière sélective"""
        current_time = time.time()
        
        # Toujours publier ma vitesse actuelle (nécessaire pour la prédiction)
        # Calculer la vitesse basée sur le changement de position
        if hasattr(self, '_prev_my_position'):
            dt = self.dt
            if dt > 0:
                vx = (self.my_real_position['x'] - self._prev_my_position['x']) / dt
                vy = (self.my_real_position['y'] - self._prev_my_position['y']) / dt
                
                velocity_msg = Vector3()
                velocity_msg.x = vx
                velocity_msg.y = vy
                velocity_msg.z = 0.0
                
                self.velocity_publisher.publish(velocity_msg)
                
                # Mettre à jour ma vitesse locale
                self.my_current_velocity = {
                    'vx': vx,
                    'vy': vy,
                    'timestamp': current_time
                }
        
        # Publier ma position seulement si nécessaire
        if self.should_publish_my_position():
            position_msg = Point()
            position_msg.x = self.my_real_position['x']
            position_msg.y = self.my_real_position['y']
            position_msg.z = 0.0
            
            self.published_pose_publisher.publish(position_msg)
            
            # Mettre à jour ma position publiée
            self.my_published_position = {
                'x': self.my_real_position['x'],
                'y': self.my_real_position['y'],
                'timestamp': current_time
            }
            
            self.last_publish_time = current_time
            self.position_updates_count += 1
            
            self.get_logger().info(
                f"Position publiée pour {self.robot_name}: ({self.my_real_position['x']:.3f}, {self.my_real_position['y']:.3f}) "
                f"[Mise à jour #{self.position_updates_count}]"
            )
        
        # Sauvegarder ma position pour le calcul de vitesse
        self._prev_my_position = self.my_real_position.copy()

    def timer_callback(self):
        # Mettre à jour ma position réelle
        self.update_my_real_position()
        
        # Prédire les positions des voisins
        self.predict_neighbors_positions()
        
        # Publier ma position et vitesse de manière sélective
        self.publish_selective_position_and_velocity()
        
        # Affichage périodique des statistiques
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 100 == 0:  # Toutes les 10 secondes
            self.get_logger().info(
                f"Statistiques communication - Publications: {self.position_updates_count}, Prédictions: {self.total_predictions}"
            )
            self.get_logger().info(f"Ma position: {self.my_real_position}")
            self.get_logger().info(f"Positions prédites des voisins: {self.neighbors_predicted_positions}")
        
        # Initialiser la formation si nécessaire
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.formation_initialized = True
            self.get_logger().info("Formation initialized with predictive positions")
            
        # Appliquer le contrôle de consensus
        if self.active and self.formation_initialized and self.goal_point_set:
            self.apply_consensus_control()

    #--------------------------------------------------------------------
    # Méthodes de contrôle
    #--------------------------------------------------------------------
    
    def master_callback(self, msg):
        """Callback pour le topic de contrôle"""
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info("Contrôle prédictif actif")
        else:
            self.get_logger().info("Contrôle prédictif désactivé")
            self.stop_robot()

    def goal_point_callback(self, msg):
        """Callback pour le topic de position cible"""
        self.goal_point = (msg.x, msg.y)
        self.goal_point_set = True
        self.is_target_reached_state = False
        self.publish_target_status(0)
        self.get_logger().info(f"New goal point set: x={msg.x:.4f}, y={msg.y:.4f}")

    def formation_callback(self, msg):
        """Callback pour réinitialiser la formation sur demande"""
        self.get_logger().info("Received formation reset command, re-initializing formation.")
        self.formation_initialized = False
        self.initialize_formation()

    def initialize_formation(self):
        """Initialise la formation désirée basée sur les positions prédites"""
        if self.formation_initialized:
            self.get_logger().warn("Formation already initialized! Skipping re-initialization.")
            return
            
        # Calculer le barycentre initial
        initial_barycenter = self.compute_swarm_center()
        
        # Calculer et stocker mon vecteur relatif initial
        self.initial_relative_vector = np.array([
            self.my_real_position['x'] - initial_barycenter[0],
            self.my_real_position['y'] - initial_barycenter[1]
        ])
        
        self.get_logger().info(f"Vecteur relatif initial calculé: {self.initial_relative_vector}")
        
        # Calculer les distances initiales avec mes voisins
        for neighbor_name in self.neighbors_predicted_positions.keys():
            neighbor_pos = self.neighbors_predicted_positions[neighbor_name]
            if neighbor_pos['x'] != 0 or neighbor_pos['y'] != 0:
                dist = math.sqrt(
                    (self.my_real_position['x'] - neighbor_pos['x'])**2 + 
                    (self.my_real_position['y'] - neighbor_pos['y'])**2
                )
                self.desired_distances[neighbor_name] = dist
        
        self.formation_initialized = True
        self.get_logger().info(f"Desired formation set using predictive positions")
        self.get_logger().info(f"Initial distances: {self.desired_distances}")

    def compute_swarm_center(self):
        """Calcule le centre de masse de l'essaim en utilisant les positions prédites"""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"barycenter", rclpy.time.Time()
            )
            pos = trans.transform.translation
            return [pos.x, pos.y]
        except Exception as e:
            # Fallback: calculer à partir des positions disponibles
            all_positions = [self.my_real_position]
            for neighbor_pos in self.neighbors_predicted_positions.values():
                if neighbor_pos['x'] != 0 or neighbor_pos['y'] != 0:
                    all_positions.append(neighbor_pos)
            
            if len(all_positions) > 0:
                total_x = sum(pos['x'] for pos in all_positions)
                total_y = sum(pos['y'] for pos in all_positions)
                return [total_x / len(all_positions), total_y / len(all_positions)]
            else:
                return [self.my_real_position['x'], self.my_real_position['y']]

    def apply_consensus_control(self):
        """Applique le contrôle de consensus avec les positions prédites"""
        if not self.goal_point_set:
            return
            
        global_goal = np.array(self.goal_point)
        
        # Utiliser ma position réelle pour le contrôle
        pi = np.array([self.my_real_position['x'], self.my_real_position['y']])
        pr = global_goal + self.initial_relative_vector
        
        # Obtenir mes voisins
        neighbors_names = self.robot_neighbors.get(self.robot_name, [])
        if not neighbors_names:
            neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        
        # Positions des voisins (prédites)
        pj_array = []
        dij_list = []
        
        for neighbor_name in neighbors_names:
            if neighbor_name in self.neighbors_predicted_positions:
                neighbor_pos = self.neighbors_predicted_positions[neighbor_name]
                
                # Vérifier que la position n'est pas à l'origine
                if neighbor_pos['x'] != 0 or neighbor_pos['y'] != 0:
                    pj = np.array([neighbor_pos['x'], neighbor_pos['y']])
                    pj_array.append(pj)
                    
                    dij = self.desired_distances.get(neighbor_name)
                    if dij is None:
                        current_distance = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                        dij = current_distance
                    
                    dij_list.append(dij)
        
        if not pj_array:
            # Pas de voisins disponibles, juste aller vers l'objectif
            control_vector = -(c1_gamma * (pi - pr))
            ui_alpha = np.array([0.0, 0.0])
            ui_gamma = control_vector
        else:
            # Appliquer le contrôle
            control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma = control_with_components(
                pj_array=pj_array,
                pi=pi,
                dij_list=dij_list,
                pr=pr,
                dt=self.dt,
                integral_term=self.integral_term,
                derivative_term=self.derivative_term,
                logger=self.get_logger()
            )
            
            self.integral_term = updated_integral
            self.derivative_term = updated_derivative
        
        # Publier les composantes de contrôle
        control_msg = Float64MultiArray()
        control_msg.data = [float(ui_alpha[0]), float(ui_alpha[1]), 
                           float(ui_gamma[0]), float(ui_gamma[1])]
        self.control_component_publisher.publish(control_msg)
        
        # Transformer et publier les commandes
        robot_lin_x, robot_lin_y = self.transform_velocity(
            control_vector[0], control_vector[1]
        )
        
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
        
        # Vérifier si j'ai atteint ma cible individuelle
        current_state = self.is_robot_target_reached(pi, pr)
        
        if current_state != self.is_target_reached_state:
            self.is_target_reached_state = current_state
            self.publish_target_status(1 if current_state else 0)
        
        self.cmd_vel_publisher.publish(twist_msg)

    def transform_velocity(self, global_lin_x, global_lin_y):
        """Transforme les vitesses du repère global au repère du robot"""
        try:
            global_lin_x = float(global_lin_x)
            global_lin_y = float(global_lin_y)
            
            global_vel = Vector3Stamped()
            global_vel.header.frame_id = GLOBAL_FRAME
            global_vel.header.stamp = self.get_clock().now().to_msg()
            global_vel.vector.x = global_lin_x
            global_vel.vector.y = global_lin_y
            global_vel.vector.z = 0.0
            
            try:
                robot_frame_id = f"{self.robot_name}/base_link"
                
                transform = self.tf_buffer.lookup_transform(
                    robot_frame_id, GLOBAL_FRAME, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                return robot_vel.vector.x, robot_vel.vector.y
                
            except TransformException as ex:
                self.get_logger().error(f'Échec de la transformation TF2: {ex}')
                return global_lin_x, global_lin_y
            
        except Exception as e:
            self.get_logger().error(f'Erreur dans transform_velocity: {e}')
            return global_lin_x, global_lin_y

    def stop_robot(self):
        """Arrêter ce robot"""
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

    def all_positions_available(self):
        """Vérifie si toutes les positions nécessaires sont disponibles"""
        # Vérifier ma position
        if abs(self.my_real_position['x']) < 0.001 and abs(self.my_real_position['y']) < 0.001:
            return False
        
        # Vérifier les positions des voisins
        for neighbor_pos in self.neighbors_predicted_positions.values():
            if neighbor_pos['x'] == 0 and neighbor_pos['y'] == 0:
                return False
        
        return True

    def is_robot_target_reached(self, robot_pos, target_pos):
        """Vérifie si ce robot a atteint sa cible individuelle"""
        distance = math.sqrt((robot_pos[0] - target_pos[0])**2 + (robot_pos[1] - target_pos[1])**2)
        return distance <= self.target_tolerance

    def publish_target_status(self, status):
        """Publier le statut d'atteinte de cible"""
        msg = Int32()
        msg.data = status
        self.target_status_publisher.publish(msg)

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
