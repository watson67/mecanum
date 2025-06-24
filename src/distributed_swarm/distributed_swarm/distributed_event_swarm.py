#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32, String, Float64MultiArray
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3Stamped
import tf2_geometry_msgs
from tf2_ros import TransformException
import socket
import numpy as np
# Import formules.py
from swarm_manager.formules import *
from swarm_manager.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

'''
Version distribuée du contrôleur d'essaim.
Ce programme doit s'exécuter sur chaque robot individuellement.
Chaque robot contrôle son propre mouvement tout en maintenant la formation avec les autres.
'''

#--------------------------------------------------------------------
# Variables globales
#--------------------------------------------------------------------
# Noms possibles des robots dans l'essaim
GLOBAL_FRAME = "mocap"  # nom du repère global, celui ci est défini dans tf2_manager

# Note : Les topics et repères tf2 utilisés seront de la forme :
# ------
# /{robot_name}/cmd_vel
# /{robot_name}/base_link
# Il faut donc s'assurer au préalable que ces topics et repères existent.

# Le nom du robot est déterminé automatiquement par le hostname de la machine, 
# permettant ainsi de lancer exactement le même code sur chaque robot, facilitant la mise
# en place de l'essaim.

#--------------------------------------------------------------------

class DistributedSwarmController(Node):
    def __init__(self):
        # Déterminer le nom du robot à partir du hostname
        hostname = socket.gethostname().lower()
        #hostname = "aramis-desktop"  # Pour le test, forcer le nom du robot à Aramis
        # Supprimer le suffixe '-desktop' si présent
        if hostname.endswith('-desktop'):
            hostname = hostname[:-8]
        self.robot_name = hostname.capitalize()  # Première lettre en majuscule
        # Vérifier si le nom est dans la liste des robots connus
        if self.robot_name not in ALL_ROBOT_NAMES:
            print(f"Warning: Robot name '{self.robot_name}' not in known robot list {ALL_ROBOT_NAMES}")
            self.robot_name = "Unknown"  # Fallback au cas où
        
        super().__init__(
            f'distributed_swarm_controller',
            namespace=f"/{self.robot_name}"
        )
        self.get_logger().info(f"Starting distributed swarm controller for robot: {self.robot_name}")

        

        #--------------------------------------------------------------------
        # Variables TF2 pour les positions des robots 
        #--------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #--------------------------------------------------------------------
        # Publishers 
        #--------------------------------------------------------------------
        # Publisher pour contrôler ce robot uniquement
        self.cmd_vel_publisher = self.create_publisher(
            Twist, f"/{self.robot_name}/cmd_vel", 10
        )
        
        # Publisher pour partager sa position avec les autres robots (simulation d'un essaim 
        # sans motion capture ou chaque robot détermine sa propre position)
        self.position_publisher = self.create_publisher(
            Point, f"/{self.robot_name}/robot_positions", 10
        )

        # Contribution à la détection d'atteinte de cible
        self.target_status_publisher = self.create_publisher(
            Int32, f"/{self.robot_name}/target_status", 10
        )

        # Publisher pour les composantes de contrôle
        self.control_component_publisher = self.create_publisher(
            Float64MultiArray, f"/{self.robot_name}/control_components", 10
        )

        #--------------------------------------------------------------------
        # Subscribers 
        #--------------------------------------------------------------------
        # Souscription au topic de contrôle de l'essaim (arret ou démarrage)
        self.create_subscription(
            Int32, "/master", self.master_callback, 10
        )

        # Souscription au topic de position cible
        self.create_subscription(
            Point, "/goal_point", self.goal_point_callback, 10
        )
        
        # Souscription aux positions des autres robots (si nécessaire)
        self.robot_positions = {}
        for name in ALL_ROBOT_NAMES:
            if name != self.robot_name:  # Ne pas s'abonner à sa propre position
                self.create_subscription(
                    Point, f"/{name}/robot_positions", 
                    lambda msg, robot_name=name: self.robot_position_callback(msg, robot_name),
                    10
                )
                self.robot_positions[name] = None

        #--------------------------------------------------------------------
        # Variables de classe 
        #--------------------------------------------------------------------
        self.active = False
        
        # Les distances désirées entre robots
        self.desired_distances = {}
        
        # Vecteurs relatifs initiaux de chaque robot par rapport au barycentre initial
        self.initial_relative_vectors = []  # Vecteur du barycentre vers chaque robot
        
        # Position actuelle du robot
        self.my_position = {'x': 0.0, 'y': 0.0}
        
        # Position des autres robots (mise à jour via TF2)
        self.other_robot_positions = {}
        
        # Objectif de l'essaim
        self.goal_point = (0.0, 0.0)
        self.goal_point_set = False
        
        self.formation_initialized = False
        
        # Terme intégral pour le contrôle
        self.integral_term = None
        
        # Terme dérivé pour le contrôle
        self.derivative_term = None
        
        # Pas de temps pour l'intégration
        self.dt = 0.1
        
        # Tolérance pour considérer que la cible est atteinte
        self.target_tolerance = 0.05  # aligné avec swarm.py
        
        # État d'atteinte de la cible
        self.is_target_reached_state = False

        # Configuration des voisins
        self.robot_neighbors = ROBOT_NEIGHBORS

        # Timer pour recharger la configuration des voisins
        self.create_timer(2.0, self.reload_neighbor_config)  # Recharger toutes les 2 secondes

        # Timer pour le contrôle périodique
        self.create_timer(self.dt, self.timer_callback)

        # Souscription au topic '/formation' pour réinitialiser la formation à la demande
        self.create_subscription(
            Int32, "/formation", self.formation_callback, 10
        )

        # Seuils pour la commande événementielle
        self.distance_threshold = 0.05  # seuil d'écart de distance avec voisins (m)
        self.target_threshold = 0.02  # seuil de changement de position cible individuelle (m)

        # Stockage des valeurs précédentes
        self.prev_neighbor_errors = {}
        self.prev_individual_target = None
        self.prev_goal_point = None
        self.prev_control_vector = np.array([0.0, 0.0])

    #--------------------------------------------------------------------
    # Callbacks pour les topics de contrôle
    #--------------------------------------------------------------------
    def master_callback(self, msg):
        """Callback pour le topic de contrôle global"""
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info("Contrôle actif")
        else:
            self.get_logger().info("Contrôle désactivé")
            self.stop_robot()

    def goal_point_callback(self, msg):
        """Callback pour le topic de position cible"""
        self.goal_point = (msg.x, msg.y)
        self.goal_point_set = True
        self.is_target_reached_state = False  # Réinitialiser l'état
        self.publish_target_status(0)         # Indiquer que la cible n'est pas encore atteinte
        self.get_logger().info(f"New goal point set: x={msg.x:.4f}, y={msg.y:.4f}")

    def robot_position_callback(self, msg, robot_name):
        """Callback pour les positions des autres robots"""
        self.robot_positions[robot_name] = {'x': msg.x, 'y': msg.y}

    def formation_callback(self, msg):
        """Callback pour réinitialiser la formation sur demande"""
        self.get_logger().info("Received formation reset command, re-initializing formation.")
        self.formation_initialized = False
        self.initialize_formation()

    def reload_neighbor_config(self):
        """Recharger la configuration des voisins depuis le fichier YAML"""
        try:
            # Recharger le module config pour obtenir les dernières relations
            import importlib
            import swarm_manager.config
            importlib.reload(swarm_manager.config)
            from swarm_manager.config import ROBOT_NEIGHBORS
            
            self.robot_neighbors = ROBOT_NEIGHBORS
            self.get_logger().info(f"Configuration des voisins rechargée: {self.robot_neighbors}")
            
        except Exception as e:
            self.get_logger().warn(f"Impossible de recharger la configuration des voisins: {e}")
            # Utiliser tous les robots comme voisins par défaut
            self.robot_neighbors = {robot: [r for r in ALL_ROBOT_NAMES if r != robot] for robot in ALL_ROBOT_NAMES}

    def all_positions_available(self):
        """Vérifie si toutes les positions des robots sont connues (non None)"""
        # Vérifie la position du robot courant
        if abs(self.my_position['x']) < 0.001 and abs(self.my_position['y']) < 0.001:
            self.get_logger().debug("Ma position semble être à l'origine (0,0)")
            return False
        
        # Vérifie les positions des autres robots
        missing_robots = []
        available_robots = []
        for name in ALL_ROBOT_NAMES:
            if name != self.robot_name:
                if name not in self.other_robot_positions or self.other_robot_positions[name] is None:
                    missing_robots.append(name)
                else:
                    available_robots.append(name)
        
        if missing_robots:
            self.get_logger().debug(f"Positions manquantes: {missing_robots}, disponibles: {available_robots}")
            return False
            
        self.get_logger().info(f"Toutes les positions sont disponibles pour: {[self.robot_name] + available_robots}")
        return True

    #--------------------------------------------------------------------
    # Boucle principale de contrôle
    #--------------------------------------------------------------------
    def timer_callback(self):
        # Mettre à jour ma position
        self.update_my_position()
        
        # Publier ma position pour les autres robots
        self.publish_my_position()
        
        # Mettre à jour les positions des autres robots via TF2
        self.update_other_robot_positions()
        
        # Afficher l'état des positions toutes les 50 itérations (environ 5 secondes)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 50 == 0:
            self.get_logger().info(f"Ma position: {self.my_position}")
            self.get_logger().info(f"Positions autres robots: {self.other_robot_positions}")
            self.get_logger().info(f"Positions publiées reçues: {self.robot_positions}")
            self.get_logger().info(f"Formation initialisée: {self.formation_initialized}")
            
            # Lister les frames TF2 disponibles pour debug
            try:
                available_frames = self.tf_buffer.all_frames_as_string()
                self.get_logger().info(f"Frames TF2 disponibles: {available_frames}")
            except Exception as e:
                self.get_logger().debug(f"Impossible de lister les frames TF2: {e}")
        
        # Initialiser la formation si ce n'est pas déjà fait et toutes les positions sont connues
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.get_logger().info("Formation initialized")
            
        # Déterminer le goal point à utiliser
        if self.goal_point is not None:
            goal_point = self.goal_point
        else:
            barycentre = self.compute_swarm_center()
            goal_point = (barycentre[0], barycentre[1])

        # Appliquer le contrôle de consensus événementiel si actif
        if self.active and self.formation_initialized and self.goal_point_set:
            # Calculer si un événement doit déclencher une nouvelle commande
            event_triggered = self.should_update_command(goal_point)
            if event_triggered:
                self.apply_consensus_control(goal_point)
            else:
                # Continuer avec la dernière commande
                self.publish_last_command()
                self.get_logger().info(
                    f"Pas de nouvelle commande - utilisation de la dernière commande"
                )
            
            # Vérifier si la cible individuelle est atteinte
            # Position de ce robot
            pi = np.array([self.my_position['x'], self.my_position['y']])
            # Point cible individuel pour ce robot
            global_goal = np.array(goal_point)
            pr = global_goal + self.initial_relative_vectors
            
            current_state = self.is_robot_target_reached(pi, pr)
            
            # Si l'état a changé, publier le statut
            if current_state != self.is_target_reached_state:
                self.is_target_reached_state = current_state
                self.publish_target_status(1 if current_state else 0)
                
                if current_state:
                    self.get_logger().info("Individual target reached!")
                else:
                    self.get_logger().info("Individual target not reached")

    #--------------------------------------------------------------------
    # Mise à jour des positions
    #--------------------------------------------------------------------
    def update_my_position(self):
        """Mettre à jour ma position via TF2"""
        try:
            trans = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"{self.robot_name}/base_link", rclpy.time.Time()
            )
            pos = trans.transform.translation
            self.my_position = {'x': pos.x, 'y': pos.y}
        except Exception as e:
            self.get_logger().warn(f"Échec TF2 pour {self.robot_name}: {e}")
    
    def publish_my_position(self):
        """Publier ma position pour les autres robots"""
        point_msg = Point()
        point_msg.x = float(self.my_position['x'])
        point_msg.y = float(self.my_position['y'])
        point_msg.z = 0.0
        self.position_publisher.publish(point_msg)
    
    def update_other_robot_positions(self):
        """Mettre à jour les positions des autres robots via TF2"""
        for robot_name in ALL_ROBOT_NAMES:
            if robot_name != self.robot_name:
                try:
                    trans = self.tf_buffer.lookup_transform(
                        GLOBAL_FRAME, f"{robot_name}/base_link", rclpy.time.Time()
                    )
                    pos = trans.transform.translation
                    self.other_robot_positions[robot_name] = {'x': pos.x, 'y': pos.y}
                except Exception as e:
                    # Si TF2 échoue, essayer d'utiliser la position publiée (si disponible)
                    if robot_name in self.robot_positions and self.robot_positions[robot_name] is not None:
                        self.other_robot_positions[robot_name] = self.robot_positions[robot_name]
                    else:
                        # Pas de mise à jour si aucune position n'est disponible
                        self.get_logger().debug(f"Pas de position disponible pour {robot_name}")

    #--------------------------------------------------------------------
    # Calcul du contrôle et de la formation
    #--------------------------------------------------------------------
    def initialize_formation(self):
        """Initialiser les distances désirées entre les robots"""
        if self.formation_initialized:
            self.get_logger().warn("Formation already initialized! Skipping re-initialization.")
            return
            
        # Calculer le barycentre initial
        initial_barycenter = self.compute_swarm_center()
        
        # Calculer et stocker le vecteur relatif initial de ce robot
        relative_vector = np.array([
            self.my_position['x'] - initial_barycenter[0],
            self.my_position['y'] - initial_barycenter[1]
        ])
        self.initial_relative_vectors = relative_vector
        
        self.get_logger().info(f"Vecteur relatif initial calculé: {self.initial_relative_vectors}")
        
        for other_name in ALL_ROBOT_NAMES:
            if other_name != self.robot_name and other_name in self.other_robot_positions:
                other_pos = self.other_robot_positions[other_name]
                if other_pos is not None:
                    # Calculer la distance entre ce robot et l'autre
                    dist = math.sqrt(
                        (self.my_position['x'] - other_pos['x'])**2 + 
                        (self.my_position['y'] - other_pos['y'])**2
                    )
                    
                    # Stocker la distance désirée
                    self.desired_distances[other_name] = dist
                
        self.formation_initialized = True  # Verrouille l'initialisation ici
        self.get_logger().info(f"Initialized formation with distances: {self.desired_distances}")
        # Afficher la position du barycentre à l'initialisation
        barycentre = self.compute_swarm_center()
        self.get_logger().info(
            f"Barycentre (init): X:{barycentre[0]:.3f} ; Y:{barycentre[1]:.3f}"
        )

    def compute_swarm_center(self):
        """Calculer le centre de l'essaim"""
        try:
            trans = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"barycenter", rclpy.time.Time()
            )
            pos = trans.transform.translation
            return [pos.x, pos.y]
        except Exception as e:
            self.get_logger().warn(f"Echec TF2 barycentre {e}")
            # Calculer manuellement si TF2 barycenter n'est pas disponible
            all_positions = [self.my_position] + list(self.other_robot_positions.values())
            valid_positions = [pos for pos in all_positions if pos is not None]
            if not valid_positions:
                return [self.my_position['x'], self.my_position['y']]
                
            total_x = sum(pos['x'] for pos in valid_positions)
            total_y = sum(pos['y'] for pos in valid_positions)
            count = len(valid_positions)
            
            return [total_x / count, total_y / count]

    def transform_velocity(self, global_lin_x, global_lin_y):
        """Transformer les vitesses du repère global vers le repère du robot"""
        try:
            # Convertir les entrées en float
            global_lin_x = float(global_lin_x)
            global_lin_y = float(global_lin_y)
            
            # Créer un vecteur estampillé pour représenter la vitesse globale
            global_vel = Vector3Stamped()
            global_vel.header.frame_id = GLOBAL_FRAME
            global_vel.header.stamp = self.get_clock().now().to_msg()
            global_vel.vector.x = global_lin_x
            global_vel.vector.y = global_lin_y
            global_vel.vector.z = 0.0
            
            # Récupérer la transformation entre les frames
            try:
                robot_frame_id = f"{self.robot_name}/base_link"
                
                transform = self.tf_buffer.lookup_transform(
                    robot_frame_id,  # Frame cible
                    GLOBAL_FRAME,    # Frame source
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Appliquer la transformation
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                return robot_vel.vector.x, robot_vel.vector.y
                
            except TransformException as ex:
                self.get_logger().error(f'Échec de la transformation TF2: {ex}')
                return global_lin_x, global_lin_y
                
        except Exception as e:
            self.get_logger().error(f'Erreur de transformation: {e}')
            return global_lin_x, global_lin_y

    def apply_consensus_control(self, goal_point):
        """Appliquer le contrôle de consensus pour ce robot (et stocker la commande)"""
        # Ne rien faire si aucun goal n'a été reçu
        if not self.goal_point_set:
            return
            
        # Point de référence global
        global_goal = np.array(goal_point)
        self.get_logger().info(
            f"Goal point global : X:{global_goal[0]:.3f} ; Y:{global_goal[1]:.3f}"
        )
        
        # Calculer le point cible individuel (pr) pour ce robot
        # pr = goal_point_global + vecteur_relatif_initial
        pr = global_goal + self.initial_relative_vectors
        
        self.get_logger().info(
            f"Robot {self.robot_name} - Goal individuel : X:{pr[0]:.3f} ; Y:{pr[1]:.3f} "
            f"(vecteur relatif: X:{self.initial_relative_vectors[0]:.3f}, Y:{self.initial_relative_vectors[1]:.3f})"
        )
        
        # Position de ce robot
        pi = np.array([self.my_position['x'], self.my_position['y']])
        
        # Obtenir la liste des voisins pour ce robot depuis la configuration
        neighbors_names = self.robot_neighbors.get(self.robot_name, [])
        
        if not neighbors_names:
            self.get_logger().warn(f"Aucun voisin défini pour {self.robot_name}, utilisation de tous les autres robots")
            neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        
        self.get_logger().debug(f"Robot {self.robot_name} a pour voisins: {neighbors_names}")
        
        # Positions des voisins et distances désirées
        pj_array = []
        dij_list = []
        
        # Liste pour stocker les informations de distance
        distance_info = []
        
        # Pour chaque voisin défini dans la configuration
        for neighbor_name in neighbors_names:
            if neighbor_name in self.other_robot_positions and self.other_robot_positions[neighbor_name] is not None:
                # Position du voisin
                other_pos = self.other_robot_positions[neighbor_name]
                pj = np.array([other_pos['x'], other_pos['y']])
                pj_array.append(pj)
                
                # Calculer la distance actuelle
                current_distance = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                
                # Distance désirée
                dij = self.desired_distances.get(neighbor_name)
                if dij is None:
                    dij = current_distance
                    self.get_logger().debug(f"Distance initiale non trouvée pour {self.robot_name}-{neighbor_name}, utilisation de la distance actuelle: {dij:.3f}m")
                
                dij_list.append(dij)
                
                # Ajouter les informations de distance pour l'affichage
                distance_info.append(f"{neighbor_name}: actuelle={current_distance:.3f}m, désirée={dij:.3f}m")
        
        # Afficher les distances avec les voisins
        if distance_info:
            distances_str = ", ".join(distance_info)
            self.get_logger().info(f"Robot {self.robot_name} - Distances: {distances_str}")
        
        # S'il n'y a pas de voisins, juste aller vers l'objectif individuel
        if not pj_array:
            # Vecteur simple vers l'objectif individuel
            control_vector = -(c1_gamma * (pi - pr))
            self.integral_term = None
            self.derivative_term = None
            ui_alpha = np.array([0.0, 0.0])
            ui_gamma = control_vector
        else:
            # Appliquer la fonction de contrôle avec composantes
            control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma = control_with_components(
                pj_array=pj_array,
                pi=pi,
                dij_list=dij_list,
                pr=pr,  # Utiliser le point cible individuel
                dt=self.dt,
                integral_term=self.integral_term,
                derivative_term=self.derivative_term,
                logger=self.get_logger()
            )
            
            # Mettre à jour les termes intégral et dérivé
            self.integral_term = updated_integral
            self.derivative_term = updated_derivative
        
        # Publier les composantes de contrôle
        control_msg = Float64MultiArray()
        control_msg.data = [float(ui_alpha[0]), float(ui_alpha[1]), 
                           float(ui_gamma[0]), float(ui_gamma[1])]
        self.control_component_publisher.publish(control_msg)
        
        # Transformer les vitesses dans le repère du robot
        robot_lin_x, robot_lin_y = self.transform_velocity(
            control_vector[0], control_vector[1]
        )
        
        # Créer le message Twist
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
        
        # Publier la commande de vitesse
        self.cmd_vel_publisher.publish(twist_msg)
        self.prev_control_vector = np.array([twist_msg.linear.x, twist_msg.linear.y])
        self.get_logger().info(
            f"Robot {self.robot_name} (voisins: {len(neighbors_names)}): Global:{control_vector[0]:.3f},{control_vector[1]:.3f} -> Robot:{twist_msg.linear.x:.3f},{twist_msg.linear.y:.3f}"
        )

    def is_robot_target_reached(self, robot_pos, target_pos):
        """
        Vérifie si ce robot est suffisamment proche de son point cible individuel.
        
        :param robot_pos: Position actuelle du robot [x, y]
        :param target_pos: Position cible du robot [x, y]
        :return: True si la cible est atteinte, False sinon
        """
        # Calculer la distance entre le robot et son point cible
        distance = math.sqrt((robot_pos[0] - target_pos[0])**2 + (robot_pos[1] - target_pos[1])**2)
        self.get_logger().info(f"distance to individual goal: {distance:.3f}")
        return distance <= self.target_tolerance

    def should_update_command(self, goal_point):
        """Détermine si une nouvelle commande doit être calculée (commande événementielle)"""
        # 1. Vérifier l'écart de distance avec chaque voisin
        event_triggered = False
        
        # Obtenir la liste des voisins pour ce robot depuis la configuration
        neighbors_names = self.robot_neighbors.get(self.robot_name, [])
        if not neighbors_names:
            neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        
        for neighbor_name in neighbors_names:
            if neighbor_name in self.other_robot_positions and self.other_robot_positions[neighbor_name] is not None:
                other_pos = self.other_robot_positions[neighbor_name]
                if neighbor_name in self.desired_distances:
                    dij = self.desired_distances[neighbor_name]
                    dist = math.sqrt(
                        (self.my_position['x'] - other_pos['x'])**2 +
                        (self.my_position['y'] - other_pos['y'])**2
                    )
                    error = abs(dist - dij)
                    prev_error = self.prev_neighbor_errors.get(neighbor_name, 0.0)
                    if error > self.distance_threshold:
                        event_triggered = True
                        self.get_logger().debug(f"Distance event triggered for {neighbor_name}: error={error:.3f}")
                    self.prev_neighbor_errors[neighbor_name] = error

        # 2. Vérifier le changement de position cible individuelle
        global_goal = np.array(goal_point)
        current_individual_target = global_goal + self.initial_relative_vectors
        
        if self.prev_individual_target is not None:
            target_change = np.linalg.norm(current_individual_target - self.prev_individual_target)
            if target_change > self.target_threshold:
                event_triggered = True
                self.get_logger().debug(f"Individual target change event triggered: change={target_change:.3f}")
        
        self.prev_individual_target = current_individual_target.copy()

        # 3. Vérifier si le goal point a changé
        if self.prev_goal_point != goal_point:
            event_triggered = True
            self.prev_goal_point = goal_point
            self.get_logger().debug("Goal point change event triggered")

        return event_triggered

    def publish_last_command(self):
        """Publier la dernière commande calculée (événementielle)"""
        twist_msg = Twist()
        twist_msg.linear.x = float(self.prev_control_vector[0])
        twist_msg.linear.y = float(self.prev_control_vector[1])
        self.cmd_vel_publisher.publish(twist_msg)
    
    def publish_target_status(self, status):
        """Publier le statut d'atteinte de la cible par ce robot"""
        msg = Int32()
        msg.data = status
        self.target_status_publisher.publish(msg)
    
    def stop_robot(self):
        """Arrêter ce robot"""
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        self.get_logger().info(f"Robot {self.robot_name} stopped")

def main(args=None):
    rclpy.init(args=args)
    node = DistributedSwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
