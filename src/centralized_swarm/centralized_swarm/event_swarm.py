#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32, Float64MultiArray
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3Stamped
import tf2_geometry_msgs
from tf2_ros import TransformException
import numpy as np
# Import formules.py
from swarm_manager.formules import *
from swarm_manager.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

'''
Version centralisée du contrôleur d'essaim avec contrôle événementiel.
Ce programme contrôle tous les robots de manière centralisée en utilisant
un contrôle événementiel pour réduire la fréquence des mises à jour de commande.
'''

#--------------------------------------------------------------------
# Variables globales
#--------------------------------------------------------------------
# Liste des noms des robots
GLOBAL_FRAME = "mocap" # nom du repère global, celui ci est défini dans tf2_manager

#--------------------------------------------------------------------

class EventBasedSwarmController(Node):
    def __init__(self):
        super().__init__('event_swarm_controller')

        #--------------------------------------------------------------------
        # Variables TF2 pour les positions des robots 
        #--------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #--------------------------------------------------------------------
        # Publishers 
        #--------------------------------------------------------------------
        # Publishers pour piloter chaque robot
        self.cmd_vel_publishers = {}
        self.control_component_publishers = {}
        for name in ALL_ROBOT_NAMES:
            self.cmd_vel_publishers[name] = self.create_publisher(
                Twist, f"/{name}/cmd_vel", 10
            )
            # Publisher pour les composantes de contrôle
            self.control_component_publishers[name] = self.create_publisher(
                Float64MultiArray, f"/{name}/control_components", 10
            )
            
        # Publisher pour indiquer si la cible est atteinte
        self.target_reached_publisher = self.create_publisher(
            Int32, "/target_reached", 10
        )

        # Publishers pour partager les positions des robots
        self.position_publishers = {}
        for name in ALL_ROBOT_NAMES:
            self.position_publishers[name] = self.create_publisher(
                Point, f"/{name}/robot_positions", 10
            )

        # Publishers pour le statut individuel de chaque robot
        self.target_status_publishers = {}
        for name in ALL_ROBOT_NAMES:
            self.target_status_publishers[name] = self.create_publisher(
                Int32, f"/{name}/target_status", 10
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

        # Souscription au topic '/formation' pour réinitialiser la formation à la demande
        self.create_subscription(
            Int32, "/formation", self.formation_callback, 10
        )

        #--------------------------------------------------------------------
        # Variables de classe 
        #--------------------------------------------------------------------
        self.active = False
        
        # Les distances initiales entre les robots seront retenues
        self.desired_distances = {}
        
        # Vecteurs relatifs initiaux de chaque robot par rapport au barycentre initial
        self.initial_relative_vectors = []
        
        # Positions des robots
        self.robot_positions = [{'x': 0, 'y': 0} for _ in ALL_ROBOT_NAMES]
        
        # Objectifs de l'essaim
        self.goal_point = (0.0, 0.0)
        self.goal_point_set = False
        
        # La formation désirée sera définie en fonction des positions initiales
        self.desired_formation = None
        self.formation_initialized = False
        
        # Stockage des termes intégraux pour chaque robot
        self.integral_terms = [None for _ in ALL_ROBOT_NAMES]
        
        # Stockage des termes dérivés pour chaque robot
        self.derivative_terms = [None for _ in ALL_ROBOT_NAMES]
        
        # Pas de temps pour l'intégration
        self.dt = 0.1
        
        # Tolérance pour considérer que la cible est atteinte (en mètres)
        self.target_tolerance = 0.05
        
        # État actuel d'atteinte de la cible
        self.is_target_reached_state = False

        # Configuration des voisins
        self.robot_neighbors = ROBOT_NEIGHBORS

        # Timer pour recharger la configuration des voisins
        self.create_timer(2.0, self.reload_neighbor_config)
        
        # Timer pour l'affichage périodique des positions et le contrôle
        self.create_timer(self.dt, self.timer_callback)

        # Seuils pour la commande événementielle
        self.distance_threshold = 0.05  # seuil d'écart de distance avec voisins (m)
        self.target_threshold = 0.02  # seuil de changement de position cible individuelle (m)
        self.target_distance_threshold = 0.1  # seuil de distance à la cible individuelle (m)

        # Stockage des valeurs précédentes pour chaque robot
        self.prev_neighbor_errors = [{} for _ in ALL_ROBOT_NAMES]
        self.prev_individual_targets = [None for _ in ALL_ROBOT_NAMES]
        self.prev_goal_point = None
        self.prev_control_vectors = [np.array([0.0, 0.0]) for _ in ALL_ROBOT_NAMES]
        self.prev_target_distances = [float('inf') for _ in ALL_ROBOT_NAMES]  # Distance précédente à la cible

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

    #--------------------------------------------------------------------
    # callbacks pour le topic de contrôle /master
    #--------------------------------------------------------------------
    def master_callback(self, msg):
        """Callback pour le topic de contrôle"""
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info("Contrôle actif")
        else:
            self.get_logger().info("Contrôle désactivé")
            # Arrêter tous les robots lorsque le contrôle est désactivé
            self.stop_all_robots()

    #--------------------------------------------------------------------
    # callback pour le timer
    #--------------------------------------------------------------------
    def timer_callback(self):
        # Mettre à jour les positions des robots
        self.update_robot_positions()
        
        # Publier les positions des robots pour les autres nœuds
        self.publish_robot_positions()
        
        # Afficher l'état des positions toutes les 50 itérations (environ 5 secondes)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 50 == 0:
            self.get_logger().info(f"Positions des robots: {self.robot_positions}")
            self.get_logger().info(f"Formation initialisée: {self.formation_initialized}")
        
        # Initialiser la formation si ce n'est pas déjà fait
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.formation_initialized = True
            self.get_logger().info("Formation initialized")
            
        # Appliquer le contrôle de consensus événementiel si actif ET goal_point_set
        if self.active and self.formation_initialized and self.goal_point_set:
            # Déterminer quels robots nécessitent une mise à jour de commande
            robots_to_update = self.get_robots_needing_update()
            
            if robots_to_update:
                self.apply_event_based_consensus_control(robots_to_update)
            else:
                # NE PAS envoyer de commandes - les robots continuent avec leur dernière vitesse
                self.get_logger().info("Aucun événement détecté - aucune nouvelle commande envoyée")
            
            # Vérifier si chaque robot a atteint sa cible individuelle
            for i, robot_name in enumerate(ALL_ROBOT_NAMES):
                # Position actuelle du robot
                robot_pos = np.array([self.robot_positions[i]['x'], self.robot_positions[i]['y']])
                
                # Point cible individuel pour ce robot
                robot_target = np.array(self.goal_point) + self.initial_relative_vectors[i]
                
                # Vérifier si ce robot a atteint sa cible
                robot_reached = self.is_robot_target_reached(robot_pos, robot_target)
                
                # Publier le statut pour ce robot
                self.publish_individual_target_status(robot_name, 1 if robot_reached else 0)

    def get_robots_needing_update(self):
        """Détermine quels robots nécessitent une mise à jour de commande"""
        robots_to_update = []
        
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            event_triggered = False
            
            # Position du robot courant
            pi = np.array([self.robot_positions[i]['x'], self.robot_positions[i]['y']])
            
            # 1. Vérifier l'écart de distance avec chaque voisin
            neighbors_names = self.robot_neighbors.get(robot_name, [])
            if not neighbors_names:
                neighbors_names = [r for r in ALL_ROBOT_NAMES if r != robot_name]
            
            for neighbor_name in neighbors_names:
                try:
                    j = ALL_ROBOT_NAMES.index(neighbor_name)
                    pj = np.array([self.robot_positions[j]['x'], self.robot_positions[j]['y']])
                    
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
            global_goal = np.array(self.goal_point)
            current_individual_target = global_goal + self.initial_relative_vectors[i]
            
            if self.prev_individual_targets[i] is not None:
                target_change = np.linalg.norm(current_individual_target - self.prev_individual_targets[i])
                if target_change > self.target_threshold:
                    event_triggered = True
                    self.get_logger().debug(f"Individual target change event triggered for {robot_name}: change={target_change:.3f}")
            
            self.prev_individual_targets[i] = current_individual_target.copy()
            
            # 3. Vérifier si le goal point a changé
            if self.prev_goal_point != self.goal_point:
                event_triggered = True
                self.get_logger().debug(f"Goal point change event triggered for {robot_name}")
            
            # 4. NOUVEAU: Vérifier la distance à la cible individuelle
            current_target_distance = np.linalg.norm(pi - current_individual_target)
            prev_target_distance = self.prev_target_distances[i]
            
            # Déclencher si on s'approche de la cible ou si on s'en éloigne significativement
            distance_change = abs(current_target_distance - prev_target_distance)
            if (current_target_distance < self.target_distance_threshold or  # Proche de la cible
                distance_change > self.target_threshold):  # Changement significatif de distance
                event_triggered = True
                self.get_logger().debug(f"Target distance event triggered for {robot_name}: distance={current_target_distance:.3f}, change={distance_change:.3f}")
            
            self.prev_target_distances[i] = current_target_distance
            
            if event_triggered:
                robots_to_update.append(i)
        
        # Mettre à jour prev_goal_point après vérification
        self.prev_goal_point = self.goal_point
        
        return robots_to_update

    def apply_event_based_consensus_control(self, robots_to_update):
        """Applique le contrôle de consensus uniquement aux robots nécessitant une mise à jour"""
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
            
            # Position du robot courant (pi)
            pi = np.array([self.robot_positions[i]['x'], self.robot_positions[i]['y']])
            
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
                
                # Position du voisin j
                pj = np.array([self.robot_positions[j]['x'], self.robot_positions[j]['y']])
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
            
            self.get_logger().info(
                f"Robot {robot_name} (voisins: {len(neighbors_names)}): Global:{control_vector[0]:.3f},{control_vector[1]:.3f} -> Robot:{twist_msg.linear.x:.3f},{twist_msg.linear.y:.3f}"
            )
        
        # NE PLUS envoyer les dernières commandes aux robots qui n'ont pas d'événements
        self.get_logger().info(f"Commandes mises à jour pour {len(robots_to_update)} robots sur {len(ALL_ROBOT_NAMES)}")

    #--------------------------------------------------------------------
    # Méthodes liées à l'atteinte de la cible
    #--------------------------------------------------------------------
    def is_robot_target_reached(self, robot_pos, target_pos):
        """Vérifie si un robot individuel est suffisamment proche de son point cible"""
        distance = math.sqrt((robot_pos[0] - target_pos[0])**2 + (robot_pos[1] - target_pos[1])**2)
        return distance <= self.target_tolerance

    def publish_target_reached(self, status):
        """Publie un message indiquant si la cible est atteinte"""
        msg = Int32()
        msg.data = status
        self.target_reached_publisher.publish(msg)

    #--------------------------------------------------------------------
    # Mise à jour des positions des robots
    #--------------------------------------------------------------------
    def update_robot_positions(self):
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            try:
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    GLOBAL_FRAME, f"{robot_name}/base_link", rclpy.time.Time()
                )
                pos = trans.transform.translation
                self.robot_positions[i] = {'x': pos.x, 'y': pos.y}
            except Exception as e:
                self.get_logger().warn(f"Echec TF2 {robot_name}: {e}")

    #--------------------------------------------------------------------
    # Calcul du contrôle de consensus
    #--------------------------------------------------------------------
    def initialize_formation(self):
        """Initialise la formation désirée basée sur les positions actuelles des robots"""
        if self.formation_initialized:
            self.get_logger().warn("Formation already initialized! Skipping re-initialization.")
            return
            
        # Calculer le barycentre initial
        initial_barycenter = self.compute_swarm_center()
        
        # Calculer et stocker les vecteurs relatifs initiaux
        self.initial_relative_vectors = []
        for i, pos in enumerate(self.robot_positions):
            relative_vector = np.array([
                pos['x'] - initial_barycenter[0],
                pos['y'] - initial_barycenter[1]
            ])
            self.initial_relative_vectors.append(relative_vector)
            
        self.get_logger().info(f"Vecteurs relatifs initiaux calculés: {self.initial_relative_vectors}")
            
        # Calculer les positions relatives par rapport au centre
        self.desired_formation = []
        for pos in self.robot_positions:
            rel_x = pos['x'] 
            rel_y = pos['y']
            self.desired_formation.append((rel_x, rel_y))
        
        # Calculer et stocker les distances initiales entre chaque paire de robots
        for i in range(len(ALL_ROBOT_NAMES)):
            for j in range(i+1, len(ALL_ROBOT_NAMES)):
                pos_i = self.robot_positions[i]
                pos_j = self.robot_positions[j]
                
                dist = math.sqrt((pos_i['x'] - pos_j['x'])**2 + (pos_i['y'] - pos_j['y'])**2)
                
                self.desired_distances[(i, j)] = dist
                self.desired_distances[(j, i)] = dist
        
        self.formation_initialized = True
        self.get_logger().info(f"Desired formation set to initial positions: {self.desired_formation}")
        self.get_logger().info(f"Initial inter-robot distances captured: {self.desired_distances}")
        
        # Afficher la position du barycentre à l'initialisation
        barycentre = self.compute_swarm_center()
        self.get_logger().info(f"Barycentre (init): X:{barycentre[0]:.3f} ; Y:{barycentre[1]:.3f}")

    def compute_swarm_center(self):
        """Calcule le centre de masse de l'essaim"""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"barycenter", rclpy.time.Time()
            )
            pos = trans.transform.translation
            return [pos.x, pos.y]
        except Exception as e:
            self.get_logger().warn(f"Echec TF2 barycentre {e}")
            total_x = sum(robot['x'] for robot in self.robot_positions)
            total_y = sum(robot['y'] for robot in self.robot_positions)
            return [total_x / len(self.robot_positions), total_y / len(self.robot_positions)]

    def transform_velocity(self, global_lin_x, global_lin_y, robot_name):
        """Transforme les vitesses du repère global au repère du robot en utilisant TF2"""
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

    def goal_point_callback(self, msg):
        """Callback pour le topic de position cible"""
        self.goal_point = (msg.x, msg.y)
        self.goal_point_set = True
        self.is_target_reached_state = False
        
        # Publier le statut pour chaque robot individuellement (tous à 0 au début)
        for name in ALL_ROBOT_NAMES:
            self.publish_individual_target_status(name, 0)
            
        self.get_logger().info(f"New goal point set: x={msg.x:.4f}, y={msg.y:.4f}")

    def formation_callback(self, msg):
        """Callback pour réinitialiser la formation sur demande"""
        self.get_logger().info("Received formation reset command, re-initializing formation.")
        self.formation_initialized = False
        self.initialize_formation()

    def all_positions_available(self):
        """Vérifie si toutes les positions des robots sont connues (non nulles)"""
        for pos in self.robot_positions:
            if abs(pos['x']) < 0.001 and abs(pos['y']) < 0.001:
                return False
        return True

    def publish_robot_positions(self):
        """Publier les positions de tous les robots"""
        for i, name in enumerate(ALL_ROBOT_NAMES):
            point_msg = Point()
            point_msg.x = float(self.robot_positions[i]['x'])
            point_msg.y = float(self.robot_positions[i]['y'])
            point_msg.z = 0.0
            self.position_publishers[name].publish(point_msg)

    def publish_individual_target_status(self, robot_name, status):
        """Publier le statut d'atteinte de cible pour un robot individuel"""
        msg = Int32()
        msg.data = status
        self.target_status_publishers[robot_name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EventBasedSwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
