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
import time
# Import formules.py
from swarm_manager.old_formules import *
from swarm_manager.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

'''
Contrôleur d'essaim prédictif avec optimisation des communications.
Stratégie: Chaque robot prédit la position de ses voisins basée sur leur vitesse,
et ne met à jour sa position publiée que lorsque l'erreur de prédiction dépasse un seuil.
'''

CHOICE = False  # True: distances mesurées au lancement, False: distance fixe commune
FIXED_DISTANCE = 0.5  # Distance fixe en mètres utilisée si CHOICE est False
GLOBAL_FRAME = "mocap"

class PredictiveSwarmController(Node):
    def __init__(self):
        super().__init__('predictive_swarm_controller')

        #--------------------------------------------------------------------
        # Variables TF2 pour les positions des robots 
        #--------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #--------------------------------------------------------------------
        # Publishers pour les positions et vitesses publiées
        #--------------------------------------------------------------------
        self.published_pose_publishers = {}
        self.velocity_publishers = {}
        # Publishers pour les estimations de position des voisins
        self.neighbor_estimation_publishers = {}
        
        for name in ALL_ROBOT_NAMES:
            # Topic pour la position publiée (mise à jour sélective)
            self.published_pose_publishers[name] = self.create_publisher(
                Point, f"/{name}/published_pose", 10
            )
            # Topic pour la vitesse actuelle
            self.velocity_publishers[name] = self.create_publisher(
                Vector3, f"/{name}/current_velocity", 10
            )
            
            # Publishers pour les estimations des positions des voisins
            self.neighbor_estimation_publishers[name] = {}
            for neighbor_name in ALL_ROBOT_NAMES:
                if neighbor_name != name:  # Ne pas estimer sa propre position
                    self.neighbor_estimation_publishers[name][neighbor_name] = self.create_publisher(
                        Point, f"/{name}/{neighbor_name}_estimated_position", 10
                    )

        #--------------------------------------------------------------------
        # Publishers pour le contrôle des robots
        #--------------------------------------------------------------------
        self.cmd_vel_publishers = {}
        self.control_component_publishers = {}
        for name in ALL_ROBOT_NAMES:
            self.cmd_vel_publishers[name] = self.create_publisher(
                Twist, f"/{name}/cmd_vel", 10
            )
            self.control_component_publishers[name] = self.create_publisher(
                Float64MultiArray, f"/{name}/control_components", 10
            )
            
        # Publisher pour indiquer si la cible est atteinte
        self.target_reached_publisher = self.create_publisher(
            Int32, "/target_reached", 10
        )

        # Publishers pour le statut individuel de chaque robot
        self.target_status_publishers = {}
        for name in ALL_ROBOT_NAMES:
            self.target_status_publishers[name] = self.create_publisher(
                Int32, f"/{name}/target_status", 10
            )

        #--------------------------------------------------------------------
        # Subscribers pour les positions et vitesses des autres robots
        #--------------------------------------------------------------------
        self.position_subscribers = {}
        self.velocity_subscribers = {}
        
        for name in ALL_ROBOT_NAMES:
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
        
        # Positions réelles des robots (via TF2)
        self.real_positions = [{'x': 0.0, 'y': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Positions publiées par chaque robot (dernière mise à jour)
        self.published_positions = [{'x': 0.0, 'y': 0.0, 'timestamp': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Vitesses actuelles de chaque robot
        self.current_velocities = [{'vx': 0.0, 'vy': 0.0, 'timestamp': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Positions prédites des voisins par chaque robot (calculées localement)
        # Structure: self.robot_predictions[robot_index][neighbor_index] = {'x': ..., 'y': ...}
        self.robot_predictions = {}
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            self.robot_predictions[i] = {}
            for j, neighbor_name in enumerate(ALL_ROBOT_NAMES):
                if i != j:  # Un robot ne prédit pas sa propre position
                    self.robot_predictions[i][j] = {'x': 0.0, 'y': 0.0}
        
        # Positions prédites des voisins (calculées localement)
        self.predicted_positions = [{'x': 0.0, 'y': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Dernière fois que chaque robot a publié sa position
        self.last_publish_time = [0.0 for _ in ALL_ROBOT_NAMES]
        
        # Seuil d'erreur pour déclencher une mise à jour de position (en mètres)
        self.prediction_error_threshold = 0.05
        
        # Fréquence maximale de publication (pour éviter le spam)
        self.min_publish_interval = 0.1  # 100ms minimum entre publications

        #--------------------------------------------------------------------
        # Variables pour le contrôle de consensus (comme dans swarm.py)
        #--------------------------------------------------------------------
        self.active = False
        self.desired_distances = {}
        self.initial_relative_vectors = []
        self.goal_point = (0.0, 0.0)
        self.goal_point_set = False
        self.desired_formation = None
        self.formation_initialized = False
        self.integral_terms = [None for _ in ALL_ROBOT_NAMES]
        self.derivative_terms = [None for _ in ALL_ROBOT_NAMES]
        self.dt = 0.1
        self.target_tolerance = 0.08
        self.is_target_reached_state = False
        
        # Timer pour recharger la configuration des voisins
        self.create_timer(2.0, self.reload_neighbor_config)
        
        # Timer principal pour le contrôle et la prédiction
        self.create_timer(self.dt, self.timer_callback)
        
        # Statistiques de communication
        self.position_updates_count = [0 for _ in ALL_ROBOT_NAMES]
        self.total_predictions = [0 for _ in ALL_ROBOT_NAMES]

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
        try:
            robot_index = ALL_ROBOT_NAMES.index(robot_name)
            current_time = time.time()
            
            self.published_positions[robot_index] = {
                'x': msg.x,
                'y': msg.y,
                'timestamp': current_time
            }
            
            self.get_logger().debug(f"Position reçue de {robot_name}: ({msg.x:.3f}, {msg.y:.3f})")
            
        except ValueError:
            self.get_logger().warn(f"Robot inconnu dans neighbor_position_callback: {robot_name}")

    def neighbor_velocity_callback(self, msg, robot_name):
        """Callback pour recevoir les vitesses des voisins"""
        try:
            robot_index = ALL_ROBOT_NAMES.index(robot_name)
            current_time = time.time()
            
            self.current_velocities[robot_index] = {
                'vx': msg.x,
                'vy': msg.y,
                'timestamp': current_time
            }
            
            self.get_logger().debug(f"Vitesse reçue de {robot_name}: ({msg.x:.3f}, {msg.y:.3f})")
            
        except ValueError:
            self.get_logger().warn(f"Robot inconnu dans neighbor_velocity_callback: {robot_name}")

    def update_real_positions(self):
        """Mettre à jour les positions réelles via TF2"""
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            try:
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    GLOBAL_FRAME, f"{robot_name}/base_link", rclpy.time.Time()
                )
                pos = trans.transform.translation
                self.real_positions[i] = {'x': pos.x, 'y': pos.y}

            except Exception as e:
                self.get_logger().warn(f"Echec TF2 {robot_name}: {e}")

    def predict_neighbor_positions_for_all_robots(self):
        """Calculer les prédictions de chaque robot pour tous ses voisins"""
        current_time = time.time()
        
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            # Prédire sa propre position future basée sur sa vitesse actuelle
            if hasattr(self, '_prev_positions') and i < len(self._prev_positions):
                dt = self.dt
                if dt > 0:
                    current_vx = (self.real_positions[i]['x'] - self._prev_positions[i]['x']) / dt
                    current_vy = (self.real_positions[i]['y'] - self._prev_positions[i]['y']) / dt
                    
                    # Prédire sa position dans le prochain cycle
                    predicted_x = self.real_positions[i]['x'] + current_vx * dt
                    predicted_y = self.real_positions[i]['y'] + current_vy * dt
                    
                    self.predicted_positions[i] = {
                        'x': predicted_x,
                        'y': predicted_y
                    }
            
            # Pour chaque voisin de ce robot
            for j, neighbor_name in enumerate(ALL_ROBOT_NAMES):
                if i == j:  # Un robot ne prédit pas sa propre position
                    continue
                    
                # Récupérer la dernière position publiée et vitesse du voisin
                pub_pos = self.published_positions[j]
                velocity = self.current_velocities[j]
                
                # Calculer le temps écoulé depuis la dernière position publiée
                dt_prediction = current_time - pub_pos['timestamp']
                
                if dt_prediction > 0 and velocity['timestamp'] > 0:
                    # Prédire la position actuelle du voisin basée sur sa vitesse
                    predicted_x = pub_pos['x'] + velocity['vx'] * dt_prediction
                    predicted_y = pub_pos['y'] + velocity['vy'] * dt_prediction
                    
                    self.robot_predictions[i][j] = {
                        'x': predicted_x,
                        'y': predicted_y
                    }
                    
                    self.get_logger().debug(
                        f"{robot_name} prédit {neighbor_name}: ({predicted_x:.3f}, {predicted_y:.3f}) "
                        f"dt={dt_prediction:.3f}s"
                    )
                else:
                    # Utiliser la dernière position connue si pas de données
                    self.robot_predictions[i][j] = {
                        'x': pub_pos['x'],
                        'y': pub_pos['y']
                    }

    def publish_neighbor_estimations(self):
        """Publier les estimations de position des voisins pour chaque robot"""
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            for j, neighbor_name in enumerate(ALL_ROBOT_NAMES):
                if i == j:  # Un robot ne publie pas sa propre estimation
                    continue
                    
                # Créer le message de position estimée
                estimated_pos_msg = Point()
                estimated_pos_msg.x = self.robot_predictions[i][j]['x']
                estimated_pos_msg.y = self.robot_predictions[i][j]['y']
                estimated_pos_msg.z = 0.0
                
                # Publier l'estimation
                self.neighbor_estimation_publishers[robot_name][neighbor_name].publish(estimated_pos_msg)
                
                self.get_logger().debug(
                    f"Publié estimation de {robot_name} pour {neighbor_name}: "
                    f"({estimated_pos_msg.x:.3f}, {estimated_pos_msg.y:.3f})"
                )

    def get_robot_predicted_positions(self, robot_index):
        """Récupérer les positions prédites par un robot spécifique pour ses voisins"""
        predicted_positions = [{'x': 0.0, 'y': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Position réelle du robot lui-même
        predicted_positions[robot_index] = self.real_positions[robot_index].copy()
        
        # Positions prédites de ses voisins
        for neighbor_index, prediction in self.robot_predictions[robot_index].items():
            predicted_positions[neighbor_index] = prediction.copy()
            
        return predicted_positions

    def should_publish_position(self, robot_index):
        """Déterminer si un robot doit publier sa position"""
        current_time = time.time()
        
        # Respecter l'intervalle minimum entre publications
        if current_time - self.last_publish_time[robot_index] < self.min_publish_interval:
            return False
        
        # Première publication - toujours publier
        if self.last_publish_time[robot_index] == 0.0:
            return True
        
        # Récupérer la position réelle actuelle du robot
        real_pos = self.real_positions[robot_index]
        
        # Récupérer la dernière position publiée et vitesse de ce robot
        last_published = self.published_positions[robot_index]
        current_velocity = self.current_velocities[robot_index]
        
        # Calculer l'estimation de position basée sur la dernière position publiée et la vitesse
        if last_published['timestamp'] > 0 and current_velocity['timestamp'] > 0:
            # Temps écoulé depuis la dernière publication
            dt_since_publish = current_time - last_published['timestamp']
            
            # Position estimée = dernière position publiée + vitesse * temps écoulé
            estimated_x = last_published['x'] + current_velocity['vx'] * dt_since_publish
            estimated_y = last_published['y'] + current_velocity['vy'] * dt_since_publish
            
            # Calculer l'erreur entre position réelle et estimation
            error = math.sqrt(
                (real_pos['x'] - estimated_x)**2 + 
                (real_pos['y'] - estimated_y)**2
            )
            
            # Log de debug pour comprendre les décisions
            if error > self.prediction_error_threshold:
                self.get_logger().info(
                    f"{ALL_ROBOT_NAMES[robot_index]} - Erreur estimation: {error:.4f}m > {self.prediction_error_threshold}m, publication nécessaire"
                    f" (réel: {real_pos['x']:.3f}, {real_pos['y']:.3f} vs estimé: {estimated_x:.3f}, {estimated_y:.3f})"
                )
            
            # Publier si l'erreur dépasse le seuil
            return error > self.prediction_error_threshold
        else:
            # Si pas de données précédentes, publier
            return True

    def publish_selective_positions_and_velocities(self):
        """Publier les positions et vitesses de manière sélective"""
        current_time = time.time()
        
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            # Publier la position seulement si nécessaire
            if self.should_publish_position(i):
                # Calculer la vitesse basée sur le changement de position
                if hasattr(self, '_prev_positions'):
                    dt = self.dt
                    if dt > 0:
                        vx = (self.real_positions[i]['x'] - self._prev_positions[i]['x']) / dt
                        vy = (self.real_positions[i]['y'] - self._prev_positions[i]['y']) / dt
                    else:
                        vx, vy = 0.0, 0.0
                else:
                    vx, vy = 0.0, 0.0
                
                # Publier la position
                position_msg = Point()
                position_msg.x = self.real_positions[i]['x']
                position_msg.y = self.real_positions[i]['y']
                position_msg.z = 0.0
                
                self.published_pose_publishers[robot_name].publish(position_msg)
                
                # Publier la vitesse en même temps
                velocity_msg = Vector3()
                velocity_msg.x = vx
                velocity_msg.y = vy
                velocity_msg.z = 0.0
                
                self.velocity_publishers[robot_name].publish(velocity_msg)
                
                # Mettre à jour les données locales
                self.published_positions[i] = {
                    'x': self.real_positions[i]['x'],
                    'y': self.real_positions[i]['y'],
                    'timestamp': current_time
                }
                
                self.current_velocities[i] = {
                    'vx': vx,
                    'vy': vy,
                    'timestamp': current_time
                }
                
                self.last_publish_time[i] = current_time
                self.position_updates_count[i] += 1
                
                self.get_logger().info(
                    f"Position et vitesse publiées pour {robot_name}: pos({self.real_positions[i]['x']:.3f}, {self.real_positions[i]['y']:.3f}) "
                    f"vel({vx:.3f}, {vy:.3f}) [Mise à jour #{self.position_updates_count[i]}]"
                )
        
        # Sauvegarder les positions pour le calcul de vitesse
        self._prev_positions = [pos.copy() for pos in self.real_positions]

    def timer_callback(self):
        # Mettre à jour les positions réelles
        self.update_real_positions()
        
        # Calculer les prédictions de tous les robots pour leurs voisins
        self.predict_neighbor_positions_for_all_robots()
        
        # Publier les estimations de position des voisins
        self.publish_neighbor_estimations()
        
        # Publier positions et vitesses de manière sélective
        self.publish_selective_positions_and_velocities()
        
        # Affichage périodique des statistiques
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 100 == 0:  # Toutes les 10 secondes
            total_updates = sum(self.position_updates_count)
            total_pred = sum(self.total_predictions)
            self.get_logger().info(
                f"Statistiques communication - Publications: {total_updates}, Prédictions: {total_pred}"
            )
            
            for i, name in enumerate(ALL_ROBOT_NAMES):
                self.get_logger().info(
                    f"{name}: {self.position_updates_count[i]} mises à jour, {self.total_predictions[i]} prédictions"
                )
        
        # Initialiser la formation si nécessaire
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.formation_initialized = True
            self.get_logger().info("Formation initialized with predictive positions")
            
        # Appliquer le contrôle de consensus
        if self.active and self.formation_initialized and self.goal_point_set:
            self.apply_consensus_control()
            
            # Vérifier si chaque robot a atteint sa cible individuelle
            for i, robot_name in enumerate(ALL_ROBOT_NAMES):
                # Utiliser les positions prédites par ce robot pour le contrôle
                robot_predicted_positions = self.get_robot_predicted_positions(i)
                robot_pos = np.array([robot_predicted_positions[i]['x'], robot_predicted_positions[i]['y']])
                
                # Point cible individuel pour ce robot
                robot_target = np.array(self.goal_point) + self.initial_relative_vectors[i]
                
                # Vérifier si ce robot a atteint sa cible
                robot_reached = self.is_robot_target_reached(robot_pos, robot_target)
                
                # Publier le statut pour ce robot
                self.publish_individual_target_status(robot_name, 1 if robot_reached else 0)

    def is_robot_target_reached(self, robot_pos, target_pos):
        """
        Vérifie si un robot individuel est suffisamment proche de son point cible.
        
        :param robot_pos: Position actuelle du robot [x, y]
        :param target_pos: Position cible du robot [x, y]
        :return: True si la cible est atteinte, False sinon
        """
        # Calculer la distance entre le robot et son point cible
        distance = math.sqrt((robot_pos[0] - target_pos[0])**2 + (robot_pos[1] - target_pos[1])**2)
        # Vérifier si la distance est inférieure à la tolérance
        return distance <= self.target_tolerance

    def master_callback(self, msg):
        """Callback pour le topic de contrôle"""
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info("Contrôle prédictif actif")
        else:
            self.get_logger().info("Contrôle prédictif désactivé")
            self.stop_all_robots()

    def goal_point_callback(self, msg):
        """Callback pour le topic de position cible"""
        self.goal_point = (msg.x, msg.y)
        self.goal_point_set = True
        self.is_target_reached_state = False
        
        for name in ALL_ROBOT_NAMES:
            self.publish_individual_target_status(name, 0)
            
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
            
        # Utiliser les positions prédites (ou réelles si pas de prédiction)
        positions_for_formation = []
        for i in range(len(ALL_ROBOT_NAMES)):
            if self.predicted_positions[i]['x'] != 0 or self.predicted_positions[i]['y'] != 0:
                positions_for_formation.append(self.predicted_positions[i])
            else:
                positions_for_formation.append(self.real_positions[i])
        
        # Calculer le barycentre initial
        initial_barycenter = self.compute_swarm_center()
        
        # Calculer et stocker les vecteurs relatifs initiaux
        self.initial_relative_vectors = []
        for i, pos in enumerate(positions_for_formation):
            relative_vector = np.array([
                pos['x'] - initial_barycenter[0],
                pos['y'] - initial_barycenter[1]
            ])
            self.initial_relative_vectors.append(relative_vector)
            
        self.get_logger().info(f"Vecteurs relatifs initiaux calculés: {self.initial_relative_vectors}")
            
        # Calculer la formation désirée
        self.desired_formation = []
        for pos in positions_for_formation:
            self.desired_formation.append((pos['x'], pos['y']))
        
        # Calculer les distances initiales entre robots
        for i in range(len(ALL_ROBOT_NAMES)):
            for j in range(i+1, len(ALL_ROBOT_NAMES)):
                if CHOICE:
                    pos_i = positions_for_formation[i]
                    pos_j = positions_for_formation[j]
                    dist = math.sqrt((pos_i['x'] - pos_j['x'])**2 + (pos_i['y'] - pos_j['y'])**2)
                else:
                    dist = FIXED_DISTANCE
                
                self.desired_distances[(i, j)] = dist
                self.desired_distances[(j, i)] = dist
        
        self.formation_initialized = True
        mode_str = "mesurées" if CHOICE else f"fixes ({FIXED_DISTANCE}m)"
        self.get_logger().info(f"Desired formation set using predictive positions: {self.desired_formation}")
        self.get_logger().info(f"Distances inter-robots ({mode_str}): {self.desired_distances}")

    def compute_swarm_center(self):
        """Calcule le centre de masse de l'essaim en utilisant les positions prédites"""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"barycenter", rclpy.time.Time()
            )
            pos = trans.transform.translation
            return [pos.x, pos.y]
        except Exception as e:
            # Fallback: calculer à partir des positions prédites
            total_x = sum(pos['x'] for pos in self.predicted_positions)
            total_y = sum(pos['y'] for pos in self.predicted_positions)
            return [total_x / len(self.predicted_positions), total_y / len(self.predicted_positions)]

    def apply_consensus_control(self):
        """Applique le contrôle de consensus avec les positions prédites par chaque robot"""
        if not self.goal_point_set:
            return
            
        swarm_center = self.compute_swarm_center()
        global_goal = np.array(self.goal_point)
        current_time = time.time()
        
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            # Utiliser les positions prédites par ce robot spécifique
            robot_predicted_positions = self.get_robot_predicted_positions(i)
            
            # Position du robot (réelle)
            pi = np.array([robot_predicted_positions[i]['x'], robot_predicted_positions[i]['y']])
            pr = global_goal + self.initial_relative_vectors[i]
            
            # Calculer la distance au point cible
            target_distance = math.sqrt((pi[0] - pr[0])**2 + (pi[1] - pr[1])**2)
            
            # Calculer le temps depuis la dernière publication
            time_since_last_pub = current_time - self.last_publish_time[i]
            
            # Log des informations utiles
            self.get_logger().debug(
                f"{robot_name}: distance_cible={target_distance:.4f}m, "
                f"temps_depuis_pub={time_since_last_pub:.2f}s"
            )
            
            # Positions des voisins (prédites par ce robot)
            pj_array = []
            dij_list = []
            
            neighbors_names = getattr(self, 'robot_neighbors', ROBOT_NEIGHBORS).get(robot_name, [])
            if not neighbors_names:
                neighbors_names = [r for r in ALL_ROBOT_NAMES if r != robot_name]
            
            for neighbor_name in neighbors_names:
                try:
                    j = ALL_ROBOT_NAMES.index(neighbor_name)
                except ValueError:
                    continue
                
                # Utiliser la prédiction de ce robot pour son voisin
                pj = np.array([robot_predicted_positions[j]['x'], robot_predicted_positions[j]['y']])
                pj_array.append(pj)
                
                dij = self.desired_distances.get((i, j))
                if dij is None:
                    current_distance = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                    dij = current_distance
                
                dij_list.append(dij)
            
            if not pj_array:
                twist_msg = Twist()
                self.cmd_vel_publishers[robot_name].publish(twist_msg)
                continue
            
            # Appliquer le contrôle
            control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma = control_with_components(
                pj_array=pj_array,
                pi=pi,
                dij_list=dij_list,
                pr=pr,
                dt=self.dt,
                integral_term=self.integral_terms[i],
                derivative_term=self.derivative_terms[i],
            )
            
            self.integral_terms[i] = updated_integral
            self.derivative_terms[i] = updated_derivative
            
            # Ne plus publier les composantes de contrôle
            # control_msg = Float64MultiArray()
            # control_msg.data = [float(ui_alpha[0]), float(ui_alpha[1]), 
            #                    float(ui_gamma[0]), float(ui_gamma[1])]
            # self.control_component_publishers[robot_name].publish(control_msg)
            
            # Transformer et publier les commandes
            robot_lin_x, robot_lin_y = self.transform_velocity(
                control_vector[0], control_vector[1], robot_name
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
            
            self.cmd_vel_publishers[robot_name].publish(twist_msg)

        # Log périodique des statistiques détaillées
        if hasattr(self, '_control_debug_counter'):
            self._control_debug_counter += 1
        else:
            self._control_debug_counter = 0
            
        if self._control_debug_counter % 50 == 0:  # Toutes les 5 secondes
            self.get_logger().info("=== Statistiques de contrôle ===")
            for i, robot_name in enumerate(ALL_ROBOT_NAMES):
                robot_predicted_positions = self.get_robot_predicted_positions(i)
                pi = np.array([robot_predicted_positions[i]['x'], robot_predicted_positions[i]['y']])
                pr = global_goal + self.initial_relative_vectors[i]
                target_distance = math.sqrt((pi[0] - pr[0])**2 + (pi[1] - pr[1])**2)
                time_since_last_pub = current_time - self.last_publish_time[i]
                
                self.get_logger().info(
                    f"{robot_name}: dist_cible={target_distance:.4f}m, "
                    f"temps_pub={time_since_last_pub:.2f}s, "
                    f"publications={self.position_updates_count[i]}"
                )

    def transform_velocity(self, global_lin_x, global_lin_y, robot_name):
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
                now = rclpy.time.Time()
                robot_frame_id = f"{robot_name}/base_link"
                
                transform = self.tf_buffer.lookup_transform(
                    robot_frame_id, GLOBAL_FRAME, now,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                return robot_vel.vector.x, robot_vel.vector.y
                
            except TransformException as ex:
                self.get_logger().error(f'Échec de la transformation TF2 pour {robot_name}: {ex}')
                return global_lin_x, global_lin_y
            
        except Exception as e:
            self.get_logger().error(f'Erreur dans transform_velocity: {e}')
            return global_lin_x, global_lin_y

    def stop_all_robots(self):
        """Arrêter tous les robots"""
        stop_cmd = Twist()
        for robot_name in ALL_ROBOT_NAMES:
            self.cmd_vel_publishers[robot_name].publish(stop_cmd)

    def all_positions_available(self):
        """Vérifie si toutes les positions sont disponibles"""
        # Vérifier les positions réelles
        for pos in self.real_positions:
            if abs(pos['x']) < 0.001 and abs(pos['y']) < 0.001:
                return False
        return True

    def publish_individual_target_status(self, robot_name, status):
        """Publier le statut d'atteinte de cible pour un robot individuel"""
        msg = Int32()
        msg.data = status
        self.target_status_publishers[robot_name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PredictiveSwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()