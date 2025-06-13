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
import socket
import numpy as np

# Import formules.py
from mecanum_swarm.formules import *
from mecanum_swarm.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

'''
Ce programme est un contrôleur d'essaim de robots distribué utilisant ROS2.
Chaque robot exécute sa propre instance de ce contrôleur.
'''

# Variables globales
GLOBAL_FRAME = "mocap"

class DecentralizedSwarmController(Node):
    def __init__(self):
        super().__init__('decentralized_swarm_controller')

        # Déterminer le nom du robot à partir du hostname
        hostname = socket.gethostname().lower()
        if hostname.endswith('-desktop'):
            hostname = hostname[:-8]
        self.robot_name = hostname.capitalize()
        
        self.get_logger().info(f"Initializing decentralized controller for robot: {self.robot_name}")

        # Variables TF2 pour les positions des robots 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist, f"/{self.robot_name}/cmd_vel", 10
        )
        
        self.control_component_publisher = self.create_publisher(
            Float64MultiArray, f"/{self.robot_name}/control_components", 10
        )
        
        self.target_reached_publisher = self.create_publisher(
            Int32, "/target_reached", 10
        )
        
        self.position_publisher = self.create_publisher(
            Point, f"/{self.robot_name}/robot_positions", 10
        )
        
        self.target_status_publisher = self.create_publisher(
            Int32, f"/{self.robot_name}/target_status", 10
        )

        # Subscribers
        self.create_subscription(
            Int32, "/master", self.master_callback, 10
        )
        
        self.create_subscription(
            Point, "/goal_point", self.goal_point_callback, 10
        )
        
        self.create_subscription(
            Int32, "/formation", self.formation_callback, 10
        )

        # Subscribers pour les positions des autres robots
        self.robot_position_subscribers = {}
        for robot_name in ALL_ROBOT_NAMES:
            if robot_name != self.robot_name:
                self.robot_position_subscribers[robot_name] = self.create_subscription(
                    Point, f"/{robot_name}/robot_positions", 
                    lambda msg, name=robot_name: self.robot_position_callback(msg, name), 10
                )

        # Variables de classe
        self.active = False
        self.desired_distances = {}
        
        # Positions des robots (dictionnaire avec noms comme clés)
        self.robot_positions = {name: {'x': 0.0, 'y': 0.0} for name in ALL_ROBOT_NAMES}
        
        # Objectifs de l'essaim
        self.goal_point = (0.0, 0.0)
        self.goal_point_set = False
        
        # Formation
        self.desired_formation = None
        self.formation_initialized = False
        
        # Termes de contrôle pour ce robot
        self.integral_term = None
        self.derivative_term = None
        
        # Paramètres
        self.dt = 0.1
        self.target_tolerance = 0.05
        self.is_target_reached_state = False

        # Timer pour recharger la configuration des voisins
        self.create_timer(2.0, self.reload_neighbor_config)
        
        # Timer principal
        self.create_timer(self.dt, self.timer_callback)

    def reload_neighbor_config(self):
        """Recharger la configuration des voisins depuis le fichier YAML"""
        try:
            import importlib
            import mecanum_swarm.config
            importlib.reload(mecanum_swarm.config)
            from mecanum_swarm.config import ROBOT_NEIGHBORS
            
            self.robot_neighbors = ROBOT_NEIGHBORS
            self.get_logger().info(f"Configuration des voisins rechargée: {self.robot_neighbors}")
            
        except Exception as e:
            self.get_logger().warn(f"Impossible de recharger la configuration des voisins: {e}")
            self.robot_neighbors = {robot: [r for r in ALL_ROBOT_NAMES if r != robot] for robot in ALL_ROBOT_NAMES}

    def master_callback(self, msg):
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info(f"Robot {self.robot_name}: Contrôle actif")
        else:
            self.get_logger().info(f"Robot {self.robot_name}: Contrôle désactivé")
            self.stop_robot()

    def timer_callback(self):
        # Mettre à jour la position de ce robot
        self.update_own_position()
        
        # Publier la position de ce robot
        self.publish_own_position()
        
        # Debug périodique
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 50 == 0:
            self.get_logger().info(f"Robot {self.robot_name} position: {self.robot_positions[self.robot_name]}")
            self.get_logger().info(f"Formation initialisée: {self.formation_initialized}")
        
        # Initialiser la formation si nécessaire
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.formation_initialized = True
            self.get_logger().info(f"Robot {self.robot_name}: Formation initialized")
            
        # Appliquer le contrôle
        if self.active and self.formation_initialized and self.goal_point_set:
            self.apply_consensus_control()
            
            # Vérifier si la cible est atteinte
            if self.goal_point_set:
                swarm_center = self.compute_swarm_center()
                current_state = self.is_target_reached(swarm_center, self.goal_point)
                
                if current_state != self.is_target_reached_state:
                    self.is_target_reached_state = current_state
                    self.publish_target_reached(1 if current_state else 0)
                    self.publish_individual_target_status(1 if current_state else 0)
                    
                    if current_state:
                        self.get_logger().info(f"Robot {self.robot_name}: Target reached!")

    def robot_position_callback(self, msg, robot_name):
        """Callback pour recevoir les positions des autres robots"""
        self.robot_positions[robot_name] = {'x': msg.x, 'y': msg.y}

    def update_own_position(self):
        """Mettre à jour la position de ce robot via TF2"""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"{self.robot_name}/base_link", rclpy.time.Time()
            )
            pos = trans.transform.translation
            self.robot_positions[self.robot_name] = {'x': pos.x, 'y': pos.y}

        except Exception as e:
            self.get_logger().warn(f"Echec TF2 {self.robot_name}: {e}")

    def publish_own_position(self):
        """Publier la position de ce robot"""
        point_msg = Point()
        point_msg.x = float(self.robot_positions[self.robot_name]['x'])
        point_msg.y = float(self.robot_positions[self.robot_name]['y'])
        point_msg.z = 0.0
        self.position_publisher.publish(point_msg)

    def initialize_formation(self):
        """Initialiser la formation basée sur les positions actuelles"""
        if self.formation_initialized:
            return
            
        # Calculer les distances initiales entre ce robot et ses voisins
        robot_idx = ALL_ROBOT_NAMES.index(self.robot_name)
        
        for i, robot_name in enumerate(ALL_ROBOT_NAMES):
            if i != robot_idx:
                pos_i = self.robot_positions[self.robot_name]
                pos_j = self.robot_positions[robot_name]
                
                dist = math.sqrt((pos_i['x'] - pos_j['x'])**2 + (pos_i['y'] - pos_j['y'])**2)
                
                self.desired_distances[(robot_idx, i)] = dist
                self.desired_distances[(i, robot_idx)] = dist
        
        self.formation_initialized = True
        self.get_logger().info(f"Robot {self.robot_name}: Distances initiales capturées: {self.desired_distances}")

    def compute_swarm_center(self):
        """Calculer le centre de masse de l'essaim"""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"barycenter", rclpy.time.Time()
            )
            pos = trans.transform.translation
            return [pos.x, pos.y]
        except Exception as e:
            # Fallback: calculer manuellement
            total_x = sum(robot['x'] for robot in self.robot_positions.values())
            total_y = sum(robot['y'] for robot in self.robot_positions.values())
            return [total_x / len(self.robot_positions), total_y / len(self.robot_positions)]

    def transform_velocity(self, global_lin_x, global_lin_y):
        """Transformer les vitesses du repère global au repère du robot"""
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
                robot_frame_id = f"{self.robot_name}/base_link"
                
                transform = self.tf_buffer.lookup_transform(
                    robot_frame_id, GLOBAL_FRAME, now,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                return robot_vel.vector.x, robot_vel.vector.y
                
            except TransformException as ex:
                self.get_logger().error(f'Échec transformation TF2 pour {self.robot_name}: {ex}')
                return global_lin_x, global_lin_y
            
        except Exception as e:
            self.get_logger().error(f'Erreur dans transform_velocity: {e}')
            return global_lin_x, global_lin_y

    def apply_consensus_control(self):
        """Appliquer le contrôle de consensus pour ce robot"""
        if not self.goal_point_set:
            return

        # Point de référence
        pr = np.array(self.goal_point)
        
        # Position de ce robot
        robot_idx = ALL_ROBOT_NAMES.index(self.robot_name)
        pi = np.array([self.robot_positions[self.robot_name]['x'], 
                      self.robot_positions[self.robot_name]['y']])
        
        # Obtenir les voisins de ce robot
        neighbors_names = getattr(self, 'robot_neighbors', ROBOT_NEIGHBORS).get(self.robot_name, [])
        
        if not neighbors_names:
            self.get_logger().warn(f"Aucun voisin défini pour {self.robot_name}")
            neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        
        # Préparer les données des voisins
        pj_array = []
        dij_list = []
        
        for neighbor_name in neighbors_names:
            try:
                neighbor_idx = ALL_ROBOT_NAMES.index(neighbor_name)
            except ValueError:
                continue
                
            # Position du voisin
            pj = np.array([self.robot_positions[neighbor_name]['x'],
                          self.robot_positions[neighbor_name]['y']])
            pj_array.append(pj)
            
            # Distance désirée
            dij = self.desired_distances.get((robot_idx, neighbor_idx))
            if dij is None:
                current_distance = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                dij = current_distance
                
            dij_list.append(dij)
        
        if not pj_array:
            self.get_logger().warn(f"Aucun voisin valide pour {self.robot_name}")
            self.stop_robot()
            return
        
        # Appliquer le contrôle avec composantes (incluant les modifications de c1_gamma)
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
        
        # Mettre à jour les termes de contrôle
        self.integral_term = updated_integral
        self.derivative_term = updated_derivative
        
        # Publier les composantes de contrôle
        control_msg = Float64MultiArray()
        control_msg.data = [float(ui_alpha[0]), float(ui_alpha[1]), 
                           float(ui_gamma[0]), float(ui_gamma[1])]
        self.control_component_publisher.publish(control_msg)
        
        # Transformer les vitesses
        robot_lin_x, robot_lin_y = self.transform_velocity(
            control_vector[0], control_vector[1]
        )
        
        # Créer et publier la commande
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
        
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(
            f"Robot {self.robot_name}: Global:{control_vector[0]:.3f},{control_vector[1]:.3f} -> Robot:{twist_msg.linear.x:.3f},{twist_msg.linear.y:.3f}"
        )

    def stop_robot(self):
        """Arrêter ce robot"""
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

    def goal_point_callback(self, msg):
        """Callback pour la position cible"""
        self.goal_point = (msg.x, msg.y)
        self.goal_point_set = True
        self.is_target_reached_state = False
        self.publish_target_reached(0)
        self.publish_individual_target_status(0)
        self.get_logger().info(f"Robot {self.robot_name}: New goal point: x={msg.x:.4f}, y={msg.y:.4f}")

    def formation_callback(self, msg):
        """Callback pour réinitialiser la formation"""
        self.get_logger().info(f"Robot {self.robot_name}: Formation reset command received")
        self.formation_initialized = False
        self.initialize_formation()

    def all_positions_available(self):
        """Vérifier si toutes les positions des robots sont disponibles"""
        for name, pos in self.robot_positions.items():
            if abs(pos['x']) < 0.001 and abs(pos['y']) < 0.001:
                return False
        return True

    def is_target_reached(self, swarm_center, goal):
        """Vérifier si la cible est atteinte"""
        distance = math.sqrt((swarm_center[0] - goal[0])**2 + (swarm_center[1] - goal[1])**2)
        return distance <= self.target_tolerance

    def publish_target_reached(self, status):
        """Publier le statut d'atteinte de la cible"""
        msg = Int32()
        msg.data = status
        self.target_reached_publisher.publish(msg)

    def publish_individual_target_status(self, status):
        """Publier le statut individuel de ce robot"""
        msg = Int32()
        msg.data = status
        self.target_status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DecentralizedSwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
