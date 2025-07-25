#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Int32, Float64MultiArray
import math
import socket
import numpy as np
from swarm_manager.formules import *
from swarm_manager.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS
from rclpy.qos import QoSProfile, ReliabilityPolicy

'''
Version distribuée du contrôleur d'essaim avec contrôle événementiel.
Chaque robot décide individuellement quand publier ses commandes de vitesse
basé sur des seuils événementiels locaux.
'''

#-----------------------------#
#   Paramètres globaux        #
#-----------------------------#
GLOBAL_FRAME = "mocap"
FIXED_DISTANCE = 0.5
CHOICE = False

# QoS optimisé pour VRPN
vrpn_qos = QoSProfile(
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

def calculate_angle_between_vectors(v1, v2):
    """Calcule l'angle entre deux vecteurs en radians"""
    if np.linalg.norm(v1) < 0.001 or np.linalg.norm(v2) < 0.001:
        return 0.0
    
    # Normaliser les vecteurs
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)
    
    # Calculer l'angle
    dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
    angle = math.acos(dot_product)
    
    return angle

class DistributedEventSwarmController(Node):
    """
    Contrôleur distribué événementiel pour un robot de l'essaim.
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
            'distributed_event_swarm_controller',
            namespace=f'/{self.robot_name}'
        )
        self.get_logger().info(f"Starting distributed event-based controller for robot: {self.robot_name}")

        # Configuration statique des voisins
        self.neighbors_names = ROBOT_NEIGHBORS.get(self.robot_name, [])
        if not self.neighbors_names:
            self.neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        
        self.get_logger().info(f"Robot {self.robot_name} neighbors (static): {self.neighbors_names}")
        
        #-----------------------------#
        #    VRPN Subscriptions       #
        #-----------------------------#
        
        # Subscribe to own robot's VRPN pose
        self.create_subscription(
            PoseStamped,
            f"/vrpn_mocap/{self.robot_name}/pose",
            self.my_pose_callback,
            vrpn_qos
        )
        
        # Subscribe to global barycenter
        self.global_barycenter_position = None
        self.create_subscription(
            PoseStamped,
            "/vrpn_mocap/Barycenter/pose",
            self.barycenter_callback,
            vrpn_qos
        )
        
        # Subscribe ONLY to neighbors' VRPN poses
        self.other_robot_poses = {}
        for name in self.neighbors_names:
            self.create_subscription(
                PoseStamped,
                f"/vrpn_mocap/{name}/pose",
                lambda msg, robot_name=name: self.other_robot_pose_callback(msg, robot_name),
                vrpn_qos
            )
            self.other_robot_poses[name] = None

        #-----------------------------#
        #   Publishers ROS2           #
        #-----------------------------#
        self.cmd_vel_publisher = self.create_publisher(
            Twist, f"/{self.robot_name}/cmd_vel", 10
        )
        self.position_publisher = self.create_publisher(
            Point, f"/{self.robot_name}/robot_positions", 10
        )
        self.control_component_publisher = self.create_publisher(
            Float64MultiArray, f"/{self.robot_name}/control_components", 10
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
        self.my_yaw = 0.0
        self.other_robot_positions = {}
        self.goal_point = (0.0, 0.0)
        self.goal_point_set = False
        self.formation_initialized = False
        self.desired_distances = {}
        self.initial_relative_vector = None
        self.integral_term = None
        self.derivative_term = None
        self.previous_gamma = None
        self.initial_neighbor_angles = {}  # Angles initiaux avec les voisins
        self.rotation_count = 0  # Compteur de rotations
        self.is_rotating = False  # État de rotation
        self.previous_c1_gamma = None  # Pour le lissage de c1_gamma
        self.dt = 0.1
        self.target_tolerance = 0.08
        self.is_target_reached_state = False

        #-----------------------------#
        #   Variables événementielles #
        #-----------------------------#
        
        # Seuils événementiels
        self.distance_threshold = 0.05  # Seuil d'écart de distance avec voisins (m)
        self.target_threshold = 0.02   # Seuil de changement de position cible (m)
        self.target_distance_threshold = 0.1  # Seuil de distance à la cible (m)
        self.angle_threshold = 10.0 * math.pi / 180.0  # Seuil d'angle (~10°) entre vitesse et cible
        
        # États précédents pour détection d'événements
        self.prev_neighbor_errors = {}
        self.prev_individual_target = None
        self.prev_goal_point = None
        self.prev_target_distance = float('inf')
        self.prev_velocity_direction = None
        self.last_control_time = 0.0
        
        # Statistiques
        self.control_updates_count = 0
        self.event_triggers = {
            'distance': 0,
            'target_change': 0,
            'goal_change': 0,
            'target_proximity': 0,
            'angle_deviation': 0
        }

        #-----------------------------#
        #   Timers ROS2               #
        #-----------------------------#
        self.create_timer(self.dt, self.timer_callback)

    #-----------------------------#
    #   VRPN Pose Callbacks       #
    #-----------------------------#
    
    def my_pose_callback(self, msg):
        """Callback pour la pose de ce robot depuis VRPN"""
        self.my_pose = msg
        pos = msg.pose.position
        self.my_position = {'x': pos.x, 'y': pos.y}
        
        quat = msg.pose.orientation
        self.my_yaw = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

    def other_robot_pose_callback(self, msg, robot_name):
        """Callback pour la pose d'un voisin depuis VRPN"""
        self.other_robot_poses[robot_name] = msg
        pos = msg.pose.position
        self.other_robot_positions[robot_name] = {'x': pos.x, 'y': pos.y}

    def barycenter_callback(self, msg):
        """Callback pour le barycentre global depuis tf2_manager"""
        pos = msg.pose.position
        self.global_barycenter_position = {'x': pos.x, 'y': pos.y}

    #-----------------------------#
    #   Callbacks ROS2            #
    #-----------------------------#
    def master_callback(self, msg):
        """Active ou désactive le contrôle"""
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info("Contrôle événementiel actif")
        else:
            self.get_logger().info("Contrôle événementiel désactivé")
            self.stop_robot()

    def goal_point_callback(self, msg):
        """Met à jour le goal point global"""
        self.goal_point = (msg.x, msg.y)
        self.goal_point_set = True
        self.is_target_reached_state = False
        self.publish_target_status(0)
        self.get_logger().info(f"New goal point set: x={msg.x:.4f}, y={msg.y:.4f}")

    def formation_callback(self, msg):
        """Réinitialise la formation sur demande"""
        self.get_logger().info("Received formation reset command, re-initializing formation.")
        self.formation_initialized = False
        self.initialize_formation()

    #-----------------------------#
    #   Détection d'événements    #
    #-----------------------------#
    def should_trigger_control_update(self):
        """Détermine si un événement nécessite une mise à jour du contrôle"""
        import time
        current_time = time.time()
        
        event_triggered = False
        trigger_reason = []
        
        pi = np.array([self.my_position['x'], self.my_position['y']])
        
        # 1. Vérifier l'écart de distance avec chaque voisin
        for neighbor_name in self.neighbors_names:
            if neighbor_name in self.other_robot_positions and self.other_robot_positions[neighbor_name] is not None:
                other_pos = self.other_robot_positions[neighbor_name]
                pj = np.array([other_pos['x'], other_pos['y']])
                
                current_distance = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                dij = self.desired_distances.get(neighbor_name, current_distance)
                error = abs(current_distance - dij)
                
                prev_error = self.prev_neighbor_errors.get(neighbor_name, 0.0)
                
                if error > self.distance_threshold:
                    event_triggered = True
                    trigger_reason.append(f"distance_{neighbor_name}")
                    self.event_triggers['distance'] += 1
                
                self.prev_neighbor_errors[neighbor_name] = error
        
        # 2. Vérifier le changement de position cible individuelle
        if self.goal_point_set and self.global_barycenter_position is not None:
            global_goal = np.array(self.goal_point)
            current_individual_target = global_goal + self.initial_relative_vector
            
            if self.prev_individual_target is not None:
                target_change = np.linalg.norm(current_individual_target - self.prev_individual_target)
                if target_change > self.target_threshold:
                    event_triggered = True
                    trigger_reason.append("target_change")
                    self.event_triggers['target_change'] += 1
            
            self.prev_individual_target = current_individual_target.copy()
        
        # 3. Vérifier si le goal point a changé
        if self.prev_goal_point != self.goal_point:
            event_triggered = True
            trigger_reason.append("goal_change")
            self.event_triggers['goal_change'] += 1
            self.prev_goal_point = self.goal_point
        
        # 4. Vérifier la distance à la cible individuelle
        if self.goal_point_set and self.global_barycenter_position is not None:
            global_goal = np.array(self.goal_point)
            individual_target = global_goal + self.initial_relative_vector
            current_target_distance = np.linalg.norm(pi - individual_target)
            
            distance_change = abs(current_target_distance - self.prev_target_distance)
            if (current_target_distance < self.target_distance_threshold or 
                distance_change > self.target_threshold):
                event_triggered = True
                trigger_reason.append("target_proximity")
                self.event_triggers['target_proximity'] += 1
            
            self.prev_target_distance = current_target_distance
            
            # 5. NOUVEAU: Vérifier l'angle entre direction de vitesse et direction vers cible
            if hasattr(self, 'my_velocity') and self.formation_initialized:
                velocity_vector = np.array([self.my_velocity.get('vx', 0.0), self.my_velocity.get('vy', 0.0)])
                target_direction = individual_target - pi
                
                # Calculer seulement si la vitesse n'est pas nulle et qu'on n'est pas très proche de la cible
                if (np.linalg.norm(velocity_vector) > 0.01 and 
                    np.linalg.norm(target_direction) > 0.05):
                    
                    angle_deviation = calculate_angle_between_vectors(velocity_vector, target_direction)
                    
                    if angle_deviation > self.angle_threshold:
                        event_triggered = True
                        trigger_reason.append(f"angle_deviation({angle_deviation*180/math.pi:.1f}°)")
                        self.event_triggers['angle_deviation'] += 1
                        
                    self.get_logger().debug(
                        f"Angle deviation: {angle_deviation*180/math.pi:.1f}° "
                        f"(threshold: {self.angle_threshold*180/math.pi:.1f}°)"
                    )
        
        return event_triggered, trigger_reason

    #-----------------------------#
    #   Boucle principale         #
    #-----------------------------#
    def timer_callback(self):
        """Boucle principale avec contrôle événementiel"""
        self.update_my_position()
        self.publish_my_position()
        self.update_other_robot_positions()
        
        # Debug périodique
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 100 == 0:  # Toutes les 10 secondes
            self.get_logger().info(
                f"EVENT STATS - Updates: {self.control_updates_count}, "
                f"Distance: {self.event_triggers['distance']}, "
                f"Target: {self.event_triggers['target_change']}, "
                f"Goal: {self.event_triggers['goal_change']}, "
                f"Proximity: {self.event_triggers['target_proximity']}, "
                f"Angle: {self.event_triggers['angle_deviation']}"
            )
        
        # Initialisation de la formation
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.get_logger().info("Formation initialized")
        
        # Contrôle événementiel
        if self.active and self.formation_initialized and self.goal_point_set:
            event_triggered, trigger_reasons = self.should_trigger_control_update()
            
            if event_triggered:
                self.apply_consensus_control()
                import time
                self.last_control_time = time.time()
                self.control_updates_count += 1
                
                self.get_logger().info(
                    f"EVENT TRIGGERED: {', '.join(trigger_reasons)} "
                    f"[Update #{self.control_updates_count}]"
                )
            else:
                # Pas d'événement - ne pas envoyer de nouvelle commande
                self.get_logger().debug("No event detected - maintaining last command")

    def update_my_position(self):
        """Position déjà mise à jour via callback VRPN"""
        if self.my_pose is None:
            self.get_logger().warn(f"Pas de pose VRPN reçue pour {self.robot_name}")
            return
        
        # Calculer la vitesse si on a une position précédente
        if hasattr(self, 'prev_my_position') and self.prev_my_position is not None:
            dt = self.dt
            if dt > 0:
                self.my_velocity = {
                    'vx': (self.my_position['x'] - self.prev_my_position['x']) / dt,
                    'vy': (self.my_position['y'] - self.prev_my_position['y']) / dt
                }
        else:
            self.my_velocity = {'vx': 0.0, 'vy': 0.0}
        
        self.prev_my_position = self.my_position.copy()

    def publish_my_position(self):
        """Publie la position du robot courant"""
        point_msg = Point()
        point_msg.x = float(self.my_position['x'])
        point_msg.y = float(self.my_position['y'])
        point_msg.z = 0.0
        self.position_publisher.publish(point_msg)

    def update_other_robot_positions(self):
        """Positions déjà mises à jour via callbacks VRPN"""
        for robot_name in self.neighbors_names:
            if robot_name not in self.other_robot_positions:
                self.other_robot_positions[robot_name] = None

    def all_positions_available(self):
        """Vérifie si toutes les positions nécessaires sont disponibles"""
        if self.my_pose is None:
            return False
        if abs(self.my_position['x']) < 0.001 and abs(self.my_position['y']) < 0.001:
            return False
        if self.global_barycenter_position is None:
            return False
        
        for name in self.neighbors_names:
            if name not in self.other_robot_positions or self.other_robot_positions[name] is None:
                return False
        return True

    def initialize_formation(self):
        """Initialise la formation avec le barycentre global ET calcul des angles initiaux"""
        if self.formation_initialized:
            return
            
        initial_barycenter = self.compute_swarm_center()
        if initial_barycenter is None:
            return
            
        self.initial_relative_vector = np.array([
            self.my_position['x'] - initial_barycenter[0],
            self.my_position['y'] - initial_barycenter[1]
        ])
        
        # Configuration des distances désirées avec les voisins ET calcul des angles initiaux
        neighbor_idx = 0
        for neighbor_name in self.neighbors_names:
            if neighbor_name in self.other_robot_positions and self.other_robot_positions[neighbor_name] is not None:
                other_pos = self.other_robot_positions[neighbor_name]
                dist = math.sqrt(
                    (self.my_position['x'] - other_pos['x'])**2 +
                    (self.my_position['y'] - other_pos['y'])**2
                ) if CHOICE else FIXED_DISTANCE
                self.desired_distances[neighbor_name] = dist
                
                # Calcul de l'angle initial vers ce voisin (comme dans swarm2.py)
                vector_to_neighbor = np.array([
                    other_pos['x'] - self.my_position['x'],
                    other_pos['y'] - self.my_position['y']
                ])
                
                if np.linalg.norm(vector_to_neighbor) > 0.01:
                    angle = math.atan2(vector_to_neighbor[1], vector_to_neighbor[0])
                    self.initial_neighbor_angles[neighbor_idx] = angle
                    self.get_logger().info(f"Angle initial vers {neighbor_name}: {angle:.3f} rad ({math.degrees(angle):.1f}°)")
                
                neighbor_idx += 1
        
        self.formation_initialized = True
        self.get_logger().info(f"Formation initialized with event-based control and c1_gamma attenuation")
        self.get_logger().info(f"Angles initiaux calculés: {self.initial_neighbor_angles}")

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
        """Applique le contrôle de consensus avec atténuation dynamique de c1_gamma"""
        if not self.goal_point_set or self.global_barycenter_position is None:
            return
            
        global_goal = np.array(self.goal_point)
        pr = global_goal + self.initial_relative_vector
        pi = np.array([self.my_position['x'], self.my_position['y']])
        
        pj_array = []
        dij_list = []
        
        for neighbor_name in self.neighbors_names:
            if neighbor_name in self.other_robot_positions and self.other_robot_positions[neighbor_name] is not None:
                other_pos = self.other_robot_positions[neighbor_name]
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
                    is_rotating=self.is_rotating,
                    logger=self.get_logger(),
                    previous_gamma=self.previous_gamma,
                    initial_angles=self.initial_neighbor_angles,
                    rotation_count=self.rotation_count,
                    previous_c1_gamma=self.previous_c1_gamma
                )
                self.previous_gamma = updated_gamma
                self.integral_term = updated_integral
                self.derivative_term = updated_derivative
            except TypeError:
                # Fallback pour ancienne version sans atténuation de c1_gamma
                control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma = control_with_components(
                    pj_array=pj_array,
                    pi=pi,
                    dij_list=dij_list,
                    pr=pr,
                    dt=self.dt,
                    integral_term=self.integral_term,
                    derivative_term=self.derivative_term,
                    is_rotating=self.is_rotating,
                    logger=self.get_logger(),
                    previous_gamma=self.previous_gamma
                )
                self.previous_gamma = ui_gamma
                self.integral_term = updated_integral
                self.derivative_term = updated_derivative
        
        # Transformer et publier la commande
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
    node = DistributedEventSwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
