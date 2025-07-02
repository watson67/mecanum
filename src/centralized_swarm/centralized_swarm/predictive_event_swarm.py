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
Contrôleur d'essaim prédictif événementiel avec optimisation maximale des communications.
Stratégie combinée:
1. Prédiction des positions basée sur les vitesses
2. Contrôle événementiel pour ne calculer que quand nécessaire
3. Publication sélective des positions et vitesses
'''

GLOBAL_FRAME = "mocap"

class PredictiveEventBasedSwarmController(Node):
    def __init__(self):
        super().__init__('predictive_event_swarm_controller')

        #--------------------------------------------------------------------
        # Variables TF2 pour les positions des robots 
        #--------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #--------------------------------------------------------------------
        # Publishers pour les positions et vitesses publiées (prédictif)
        #--------------------------------------------------------------------
        self.published_pose_publishers = {}
        self.velocity_publishers = {}
        for name in ALL_ROBOT_NAMES:
            self.published_pose_publishers[name] = self.create_publisher(
                Point, f"/{name}/published_pose", 10
            )
            self.velocity_publishers[name] = self.create_publisher(
                Vector3, f"/{name}/current_velocity", 10
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
            
        # Publishers pour le statut
        self.target_reached_publisher = self.create_publisher(
            Int32, "/target_reached", 10
        )
        self.target_status_publishers = {}
        for name in ALL_ROBOT_NAMES:
            self.target_status_publishers[name] = self.create_publisher(
                Int32, f"/{name}/target_status", 10
            )

        #--------------------------------------------------------------------
        # Subscribers pour les positions et vitesses (prédictif)
        #--------------------------------------------------------------------
        self.position_subscribers = {}
        self.velocity_subscribers = {}
        
        for name in ALL_ROBOT_NAMES:
            self.position_subscribers[name] = self.create_subscription(
                Point, f"/{name}/published_pose", 
                lambda msg, robot=name: self.neighbor_position_callback(msg, robot), 10
            )
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
        # Variables pour la prédiction
        #--------------------------------------------------------------------
        # Positions réelles des robots (via TF2)
        self.real_positions = [{'x': 0.0, 'y': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Positions publiées par chaque robot (dernière mise à jour)
        self.published_positions = [{'x': 0.0, 'y': 0.0, 'timestamp': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Vitesses actuelles de chaque robot
        self.current_velocities = [{'vx': 0.0, 'vy': 0.0, 'timestamp': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Positions prédites (utilisées pour le contrôle)
        self.predicted_positions = [{'x': 0.0, 'y': 0.0} for _ in ALL_ROBOT_NAMES]
        
        # Dernière fois que chaque robot a publié sa position
        self.last_publish_time = [0.0 for _ in ALL_ROBOT_NAMES]
        
        # Seuils pour la prédiction
        self.prediction_error_threshold = 0.08  # Seuil pour republier position/vitesse
        self.min_publish_interval = 0.1  # Intervalle minimum entre publications

        #--------------------------------------------------------------------
        # Variables pour le contrôle événementiel
        #--------------------------------------------------------------------
        # Seuils pour déclencher le recalcul du contrôle
        self.distance_threshold = 0.05  # Seuil d'écart de distance avec voisins (m)
        self.target_threshold = 0.02  # Seuil de changement de position cible (m)
        self.target_distance_threshold = 0.1  # Seuil de distance à la cible (m)
        self.prediction_change_threshold = 0.03  # Seuil de changement de prédiction (m)

        # Stockage des valeurs précédentes pour détection d'événements
        self.prev_neighbor_errors = [{} for _ in ALL_ROBOT_NAMES]
        self.prev_individual_targets = [None for _ in ALL_ROBOT_NAMES]
        self.prev_goal_point = None
        self.prev_control_vectors = [np.array([0.0, 0.0]) for _ in ALL_ROBOT_NAMES]
        self.prev_target_distances = [float('inf') for _ in ALL_ROBOT_NAMES]
        self.prev_predicted_positions = [{'x': 0.0, 'y': 0.0} for _ in ALL_ROBOT_NAMES]

        # Dernière fois qu'un contrôle a été calculé pour chaque robot
        self.last_control_time = [0.0 for _ in ALL_ROBOT_NAMES]

        #--------------------------------------------------------------------
        # Variables pour le contrôle de consensus
        #--------------------------------------------------------------------
        self.active = False
        self.desired_distances = {}
        self.initial_relative_vectors = []
        self.goal_point = (0.0, 0.0)
        self.goal_point_set = False
        self.formation_initialized = False
        self.integral_terms = [None for _ in ALL_ROBOT_NAMES]
        self.derivative_terms = [None for _ in ALL_ROBOT_NAMES]
        self.dt = 0.1
        self.target_tolerance = 0.05
        self.is_target_reached_state = False
        
        # Configuration des voisins
        self.robot_neighbors = ROBOT_NEIGHBORS
        
        # Timers
        self.create_timer(2.0, self.reload_neighbor_config)
        self.create_timer(self.dt, self.timer_callback)
        
        # Statistiques
        self.position_updates_count = [0 for _ in ALL_ROBOT_NAMES]
        self.total_predictions = [0 for _ in ALL_ROBOT_NAMES]
        self.control_calculations_count = [0 for _ in ALL_ROBOT_NAMES]
        self.event_triggers_count = [0 for _ in ALL_ROBOT_NAMES]

    def reload_neighbor_config(self):
        """Recharger la configuration des voisins"""
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
            self.get_logger().warn(f"Robot inconnu: {robot_name}")

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
            self.get_logger().warn(f"Robot inconnu: {robot_name}")

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

    def predict_positions(self):
        """Prédire les positions basées sur les vitesses"""
        current_time = time.time()
        
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            pub_pos = self.published_positions[i]
            velocity = self.current_velocities[i]
            
            dt_prediction = current_time - pub_pos['timestamp']
            
            if dt_prediction > 0 and velocity['timestamp'] > 0:
                predicted_x = pub_pos['x'] + velocity['vx'] * dt_prediction
                predicted_y = pub_pos['y'] + velocity['vy'] * dt_prediction
                
                self.predicted_positions[i] = {
                    'x': predicted_x,
                    'y': predicted_y
                }
                
                self.total_predictions[i] += 1
                
            else:
                self.predicted_positions[i] = {
                    'x': pub_pos['x'],
                    'y': pub_pos['y']
                }

    def should_publish_position(self, robot_index):
        """Déterminer si un robot doit republier sa position (logique prédictive)"""
        current_time = time.time()
        
        if current_time - self.last_publish_time[robot_index] < self.min_publish_interval:
            return False
        
        real_pos = self.real_positions[robot_index]
        predicted_pos = self.predicted_positions[robot_index]
        
        error = math.sqrt(
            (real_pos['x'] - predicted_pos['x'])**2 + 
            (real_pos['y'] - predicted_pos['y'])**2
        )
        
        return error > self.prediction_error_threshold

    def publish_selective_positions_and_velocities(self):
        """Publier positions et vitesses de manière sélective (logique prédictive)"""
        current_time = time.time()
        
        # Calculer les vitesses actuelles
        if hasattr(self, '_prev_positions'):
            dt = self.dt
            if dt > 0:
                for i, robot_name in enumerate(ALL_ROBOT_NAMES):
                    vx = (self.real_positions[i]['x'] - self._prev_positions[i]['x']) / dt
                    vy = (self.real_positions[i]['y'] - self._prev_positions[i]['y']) / dt
                    
                    self.current_velocities[i] = {
                        'vx': vx,
                        'vy': vy,
                        'timestamp': current_time
                    }
        
        # Publier si l'estimation est fausse
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            if self.should_publish_position(i):
                # Publier position ET vitesse
                position_msg = Point()
                position_msg.x = self.real_positions[i]['x']
                position_msg.y = self.real_positions[i]['y']
                position_msg.z = 0.0
                self.published_pose_publishers[robot_name].publish(position_msg)
                
                if hasattr(self, '_prev_positions'):
                    velocity_msg = Vector3()
                    velocity_msg.x = self.current_velocities[i]['vx']
                    velocity_msg.y = self.current_velocities[i]['vy']
                    velocity_msg.z = 0.0
                    self.velocity_publishers[robot_name].publish(velocity_msg)
                
                # Mettre à jour les données locales
                self.published_positions[i] = {
                    'x': self.real_positions[i]['x'],
                    'y': self.real_positions[i]['y'],
                    'timestamp': current_time
                }
                
                self.last_publish_time[i] = current_time
                self.position_updates_count[i] += 1
                
                self.get_logger().info(
                    f"[PRÉDICTIF] Position/vitesse publiées pour {robot_name} [#{self.position_updates_count[i]}]"
                )
        
        self._prev_positions = [pos.copy() for pos in self.real_positions]

    def get_robots_needing_control_update(self):
        """Détermine quels robots nécessitent une mise à jour de commande (même logique que event_swarm.py)"""
        robots_to_update = []
        
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            event_triggered = False
            
            # Position du robot courant (utiliser les positions prédites)
            pi = np.array([self.predicted_positions[i]['x'], self.predicted_positions[i]['y']])
            
            # 1. Vérifier l'écart de distance avec chaque voisin
            neighbors_names = self.robot_neighbors.get(robot_name, [])
            if not neighbors_names:
                neighbors_names = [r for r in ALL_ROBOT_NAMES if r != robot_name]
            
            for neighbor_name in neighbors_names:
                try:
                    j = ALL_ROBOT_NAMES.index(neighbor_name)
                    pj = np.array([self.predicted_positions[j]['x'], self.predicted_positions[j]['y']])
                    
                    # Distance actuelle
                    current_distance = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                    
                    # Distance désirée
                    dij = self.desired_distances.get((i, j))
                    if dij is not None:
                        error = abs(current_distance - dij)
                        prev_error = self.prev_neighbor_errors[i].get(neighbor_name, 0.0)
                        
                        if error > self.distance_threshold:
                            event_triggered = True
                            self.get_logger().debug(f"Distance event triggered for {robot_name}-{neighbor_name}: error={error:.3f}")
                        
                        self.prev_neighbor_errors[i][neighbor_name] = error
                        
                except ValueError:
                    continue
            
            # 2. Vérifier le changement de position cible individuelle
            if self.goal_point_set:
                global_goal = np.array(self.goal_point)
                current_individual_target = global_goal + self.initial_relative_vectors[i]
                
                if self.prev_individual_targets[i] is not None:
                    target_change = np.linalg.norm(current_individual_target - self.prev_individual_targets[i])
                    if target_change > self.target_threshold:
                        event_triggered = True
                        self.get_logger().debug(f"Individual target change event triggered for {robot_name}: change={target_change:.3f}")
                
                self.prev_individual_targets[i] = current_individual_target.copy()
                
                # 3. Vérifier la distance à la cible individuelle
                current_target_distance = np.linalg.norm(pi - current_individual_target)
                prev_target_distance = self.prev_target_distances[i]
                
                # Déclencher si on s'approche de la cible ou si on s'en éloigne significativement
                distance_change = abs(current_target_distance - prev_target_distance)
                if (current_target_distance < self.target_distance_threshold or  # Proche de la cible
                    distance_change > self.target_threshold):  # Changement significatif de distance
                    event_triggered = True
                    self.get_logger().debug(f"Target distance event triggered for {robot_name}: distance={current_target_distance:.3f}, change={distance_change:.3f}")
                
                self.prev_target_distances[i] = current_target_distance
            
            # 4. Vérifier si le goal point a changé
            if self.prev_goal_point != self.goal_point:
                event_triggered = True
                self.get_logger().debug(f"Goal point change event triggered for {robot_name}")
            
            if event_triggered:
                robots_to_update.append(i)
                self.event_triggers_count[i] += 1
        
        # Mettre à jour prev_goal_point après vérification
        self.prev_goal_point = self.goal_point
        
        return robots_to_update

    def apply_event_based_predictive_control(self, robots_to_update):
        """Applique le contrôle de consensus uniquement aux robots nécessitant une mise à jour (même logique que event_swarm.py mais avec positions prédites)"""
        # Ne rien faire si aucun goal n'a été reçu
        if not self.goal_point_set:
            return

        # Calcul du centre de masse actuel comme référence
        swarm_center = self.compute_swarm_center()
        
        # Point de référence global (goal point de l'essaim)
        global_goal = np.array(self.goal_point)
        self.get_logger().info(
            f"Goal point global : X:{global_goal[0]:.3f} ; Y:{global_goal[1]:.3f}"
        )
        self.get_logger().info(
            f"Barycentre : X:{swarm_center[0]:.3f} ; Y:{swarm_center[1]:.3f}"
        )
        
        # Pour chaque robot nécessitant une mise à jour
        for i in robots_to_update:
            robot_name = ALL_ROBOT_NAMES[i]
            
            # Position du robot courant (pi) - utiliser les positions prédites
            pi = np.array([self.predicted_positions[i]['x'], self.predicted_positions[i]['y']])
            
            # Calculer le point cible individuel (pr) pour ce robot
            pr = global_goal + self.initial_relative_vectors[i]
            
            self.get_logger().info(
                f"Robot {robot_name} - Goal individuel : X:{pr[0]:.3f} ; Y:{pr[1]:.3f} "
                f"(vecteur relatif: X:{self.initial_relative_vectors[i][0]:.3f}, Y:{self.initial_relative_vectors[i][1]:.3f})"
            )
            
            # Liste des positions des voisins (pj_array)
            pj_array = []
            # Liste des distances désirées aux voisins (dij_list)
            dij_list = []
            
            # Obtenir la liste des voisins pour ce robot depuis la configuration
            neighbors_names = self.robot_neighbors.get(robot_name, [])
            
            if not neighbors_names:
                self.get_logger().warn(f"Aucun voisin défini pour {robot_name}, utilisation de tous les autres robots")
                neighbors_names = [r for r in ALL_ROBOT_NAMES if r != robot_name]
            
            self.get_logger().debug(f"Robot {robot_name} a pour voisins: {neighbors_names}")
            
            # Liste pour stocker les informations de distance à afficher
            distance_info = []
            
            # Pour chaque voisin défini dans la configuration
            for neighbor_name in neighbors_names:
                try:
                    j = ALL_ROBOT_NAMES.index(neighbor_name)
                except ValueError:
                    self.get_logger().warn(f"Voisin {neighbor_name} non trouvé dans ALL_ROBOT_NAMES")
                    continue
                
                # Position du voisin j (utiliser les positions prédites)
                pj = np.array([self.predicted_positions[j]['x'], self.predicted_positions[j]['y']])
                pj_array.append(pj)
                
                # Calculer la distance actuelle
                current_distance = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                
                # Distance désirée entre i et j
                dij = self.desired_distances.get((i, j))
                if dij is None:
                    dij = current_distance
                    self.get_logger().debug(f"Distance initiale non trouvée pour {robot_name}-{neighbor_name}, utilisation de la distance actuelle: {dij:.3f}m")
                
                dij_list.append(dij)
                distance_info.append(f"{neighbor_name}: actuelle={current_distance:.3f}m, désirée={dij:.3f}m")
            
            # Afficher les distances avec les voisins
            if distance_info:
                distances_str = ", ".join(distance_info)
                self.get_logger().info(f"Robot {robot_name} - Distances: {distances_str}")
            
            # Si aucun voisin valide n'a été trouvé, passer au robot suivant
            if not pj_array:
                self.get_logger().warn(f"Aucun voisin valide pour {robot_name}, robot arrêté")
                twist_msg = Twist()
                self.cmd_vel_publishers[robot_name].publish(twist_msg)
                self.prev_control_vectors[i] = np.array([0.0, 0.0])
                continue
            
            # Appliquer la fonction de contrôle avec composantes
            control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma = control_with_components(
                pj_array=pj_array,
                pi=pi,
                dij_list=dij_list,
                pr=pr,
                dt=self.dt,
                integral_term=self.integral_terms[i],
                derivative_term=self.derivative_terms[i],
                logger=self.get_logger()
            )
            
            # Mettre à jour les termes intégral et dérivé pour ce robot
            self.integral_terms[i] = updated_integral
            self.derivative_terms[i] = updated_derivative
            self.control_calculations_count[i] += 1
            
            # Publier les composantes de contrôle
            control_msg = Float64MultiArray()
            control_msg.data = [float(ui_alpha[0]), float(ui_alpha[1]), 
                               float(ui_gamma[0]), float(ui_gamma[1])]
            self.control_component_publishers[robot_name].publish(control_msg)
            
            # Transformer les vitesses du repère global au repère du robot
            robot_lin_x, robot_lin_y = self.transform_velocity(
                control_vector[0], control_vector[1], robot_name
            )
            
            # Conversion en message Twist avec les vitesses transformées
            twist_msg = Twist()
            twist_msg.linear.x = float(robot_lin_x)
            twist_msg.linear.y = float(robot_lin_y)
            
            # Limiter la vitesse
            max_speed = 0.14  # m/s
            speed = math.sqrt(twist_msg.linear.x**2 + twist_msg.linear.y**2)
            if speed > max_speed:
                scaling = max_speed / speed
                twist_msg.linear.x *= scaling
                twist_msg.linear.y *= scaling
            
            # Publier la commande et stocker pour réutilisation
            self.cmd_vel_publishers[robot_name].publish(twist_msg)
            self.prev_control_vectors[i] = np.array([twist_msg.linear.x, twist_msg.linear.y])
            self.last_control_time[i] = time.time()
            
            self.get_logger().info(
                f"Robot {robot_name} (voisins: {len(neighbors_names)}): Global:{control_vector[0]:.3f},{control_vector[1]:.3f} -> Robot:{twist_msg.linear.x:.3f},{twist_msg.linear.y:.3f}"
            )
        
        # NE PLUS envoyer les dernières commandes aux robots qui n'ont pas d'événements
        self.get_logger().info(f"Commandes mises à jour pour {len(robots_to_update)} robots sur {len(ALL_ROBOT_NAMES)}")

    def timer_callback(self):
        # 1. Mettre à jour les positions réelles
        self.update_real_positions()
        
        # 2. Prédire les positions basées sur les vitesses
        self.predict_positions()
        
        # 3. Publier positions/vitesses de manière sélective (logique prédictive)
        self.publish_selective_positions_and_velocities()
        
        # Affichage périodique des statistiques
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 100 == 0:  # Toutes les 10 secondes
            total_pos_updates = sum(self.position_updates_count)
            total_predictions = sum(self.total_predictions)
            total_control_calcs = sum(self.control_calculations_count)
            total_events = sum(self.event_triggers_count)
            
            self.get_logger().info(
                f"[STATISTIQUES] Pos. publiées: {total_pos_updates}, Prédictions: {total_predictions}, "
                f"Contrôles calculés: {total_control_calcs}, Événements: {total_events}"
            )
        
        # 4. Initialiser la formation si nécessaire
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.formation_initialized = True
            self.get_logger().info("Formation initialized with predictive positions")
            
        # 5. Appliquer le contrôle événementiel prédictif si actif
        if self.active and self.formation_initialized and self.goal_point_set:
            # Déterminer quels robots nécessitent un recalcul (logique événementielle identique à event_swarm.py)
            robots_needing_update = self.get_robots_needing_control_update()
            
            if robots_needing_update:
                # Recalculer le contrôle seulement pour ces robots
                self.apply_event_based_predictive_control(robots_needing_update)
            else:
                # Aucun événement - les robots continuent avec leur dernière commande
                self.get_logger().info("Aucun événement détecté - aucune nouvelle commande envoyée")
            
            # Vérifier l'atteinte des cibles individuelles
            for i, robot_name in enumerate(ALL_ROBOT_NAMES):
                robot_pos = np.array([self.predicted_positions[i]['x'], self.predicted_positions[i]['y']])
                robot_target = np.array(self.goal_point) + self.initial_relative_vectors[i]
                robot_reached = self.is_robot_target_reached(robot_pos, robot_target)
                self.publish_individual_target_status(robot_name, 1 if robot_reached else 0)

    def master_callback(self, msg):
        """Callback pour le topic de contrôle"""
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info("Contrôle prédictif événementiel actif")
        else:
            self.get_logger().info("Contrôle prédictif événementiel désactivé")
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
        """Callback pour réinitialiser la formation"""
        self.get_logger().info("Received formation reset command, re-initializing formation.")
        self.formation_initialized = False
        self.initialize_formation()

    def initialize_formation(self):
        """Initialise la formation désirée basée sur les positions prédites"""
        if self.formation_initialized:
            self.get_logger().warn("Formation already initialized! Skipping re-initialization.")
            return
            
        # Utiliser les positions prédites
        positions_for_formation = []
        for i in range(len(ALL_ROBOT_NAMES)):
            if self.predicted_positions[i]['x'] != 0 or self.predicted_positions[i]['y'] != 0:
                positions_for_formation.append(self.predicted_positions[i])
            else:
                positions_for_formation.append(self.real_positions[i])
        
        # Calculer le barycentre initial
        initial_barycenter = self.compute_swarm_center()
        
        # Calculer les vecteurs relatifs initiaux
        self.initial_relative_vectors = []
        for i, pos in enumerate(positions_for_formation):
            relative_vector = np.array([
                pos['x'] - initial_barycenter[0],
                pos['y'] - initial_barycenter[1]
            ])
            self.initial_relative_vectors.append(relative_vector)
            
        # Calculer les distances initiales entre robots
        for i in range(len(ALL_ROBOT_NAMES)):
            for j in range(i+1, len(ALL_ROBOT_NAMES)):
                pos_i = positions_for_formation[i]
                pos_j = positions_for_formation[j]
                
                dist = math.sqrt((pos_i['x'] - pos_j['x'])**2 + (pos_i['y'] - pos_j['y'])**2)
                
                self.desired_distances[(i, j)] = dist
                self.desired_distances[(j, i)] = dist
        
        self.formation_initialized = True
        self.get_logger().info(f"Formation initialisée avec logique prédictive événementielle")

    def compute_swarm_center(self):
        """Calcule le centre de masse de l'essaim"""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"barycenter", rclpy.time.Time()
            )
            pos = trans.transform.translation
            return [pos.x, pos.y]
        except Exception as e:
            total_x = sum(pos['x'] for pos in self.predicted_positions)
            total_y = sum(pos['y'] for pos in self.predicted_positions)
            return [total_x / len(self.predicted_positions), total_y / len(self.predicted_positions)]

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
                self.get_logger().error(f'Échec TF2 pour {robot_name}: {ex}')
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
        for pos in self.predicted_positions:
            if abs(pos['x']) < 0.001 and abs(pos['y']) < 0.001:
                return False
        return True

    def is_robot_target_reached(self, robot_pos, target_pos):
        """Vérifie si un robot a atteint sa cible"""
        distance = math.sqrt((robot_pos[0] - target_pos[0])**2 + (robot_pos[1] - target_pos[1])**2)
        return distance <= self.target_tolerance

    def publish_individual_target_status(self, robot_name, status):
        """Publier le statut d'atteinte de cible pour un robot"""
        msg = Int32()
        msg.data = status
        self.target_status_publishers[robot_name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PredictiveEventBasedSwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
