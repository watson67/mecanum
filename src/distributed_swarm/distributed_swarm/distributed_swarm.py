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
from swarm_manager.formules import *
from swarm_manager.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS
from rclpy.qos import QoSProfile, ReliabilityPolicy  # Ajouté

#-----------------------------#
#   Paramètres globaux        #
#-----------------------------#
GLOBAL_FRAME = "mocap"
FIXED_DISTANCE = 0.5
CHOICE = False  # True: distances mesurées, False: distance fixe

class DistributedSwarm2Controller(Node):
    """
    Contrôleur distribué pour un robot de l'essaim.
    Chaque robot exécute ce code et ne contrôle que son propre mouvement.
    """
    def __init__(self):
        
        # Déterminer le nom du robot à partir du hostname
        hostname = socket.gethostname().lower()
        #hostname='aramis-desktop'  # Forcing the robot name for testing purposes
        if hostname.endswith('-desktop'):
            hostname = hostname[:-8]
        self.robot_name = hostname[:1].upper() + hostname[1:]
        if self.robot_name not in ALL_ROBOT_NAMES:
            print(f"Warning: Robot name '{self.robot_name}' not in known robot list {ALL_ROBOT_NAMES}")
            self.robot_name = "Unknown"
        super().__init__(
            'distributed_swarm_controller',
            namespace=f'/{self.robot_name}'
        )
        self.get_logger().info(f"Starting distributed swarm2 controller for robot: {self.robot_name}")

        #-----------------------------#
        #   Initialisation TF2        #
        #-----------------------------#
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, qos=tf_qos)

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

        # Souscription aux positions des autres robots (uniquement pour debug ou fallback)
        self.robot_positions = {}
        for name in ALL_ROBOT_NAMES:
            if name != self.robot_name:
                self.create_subscription(
                    Point, f"/{name}/robot_positions",
                    lambda msg, robot_name=name: self.robot_position_callback(msg, robot_name),
                    10
                )
                self.robot_positions[name] = None

        #-----------------------------#
        #   Variables d'état          #
        #-----------------------------#
        self.active = False  # Contrôle actif ou non
        self.my_position = {'x': 0.0, 'y': 0.0}  # Position courante du robot
        self.other_robot_positions = {}  # Positions des autres robots
        self.goal_point = (0.0, 0.0)  # Objectif global
        self.goal_point_set = False  # Indique si un goal a été reçu
        self.formation_initialized = False  # Formation initialisée ou non
        self.desired_distances = {}  # Distances désirées avec les voisins
        self.initial_relative_vector = None  # Vecteur relatif initial au barycentre
        self.integral_term = None  # Terme intégral du contrôle
        self.derivative_term = None  # Terme dérivé du contrôle
        self.previous_gamma = None   # Ajouté pour le lissage de ui_gamma
        self.dt = 0.1  # Pas de temps pour le contrôle
        self.target_tolerance = 0.08  # Tolérance pour considérer la cible atteinte
        self.is_target_reached_state = False  # Statut d'atteinte de la cible

        # Voisins du robot (chargés dynamiquement)
        self.robot_neighbors = ROBOT_NEIGHBORS

        #-----------------------------#
        #   Timers ROS2               #
        #-----------------------------#
        self.create_timer(2.0, self.reload_neighbor_config)  # Reload config voisins
        self.create_timer(self.dt, self.timer_callback)      # Boucle principale

    #-----------------------------#
    #   Callbacks ROS2            #
    #-----------------------------#
    def reload_neighbor_config(self):
        """
        Recharge la configuration des voisins depuis le fichier YAML.
        """
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

    def master_callback(self, msg):
        """
        Active ou désactive le contrôle selon la commande reçue sur /master.
        """
        self.active = (msg.data == 1)
        if self.active:
            self.get_logger().info("Contrôle actif")
        else:
            self.get_logger().info("Contrôle désactivé")
            self.stop_robot()

    def goal_point_callback(self, msg):
        """
        Met à jour le goal point global de l'essaim.
        """
        self.goal_point = (msg.x, msg.y)
        self.goal_point_set = True
        self.is_target_reached_state = False
        self.original_goal_point = None
        self.publish_target_status(0)
        self.get_logger().info(f"New goal point set: x={msg.x:.4f}, y={msg.y:.4f}")

    def formation_callback(self, msg):
        """
        Réinitialise la formation sur demande.
        """
        self.get_logger().info("Received formation reset command, re-initializing formation.")
        self.formation_initialized = False
        self.initialize_formation()

    def robot_position_callback(self, msg, robot_name):
        """
        Met à jour la position d'un autre robot reçue via topic.
        """
        self.robot_positions[robot_name] = {'x': msg.x, 'y': msg.y}

    #-----------------------------#
    #   Boucle principale         #
    #-----------------------------#
    def timer_callback(self):
        """
        Boucle principale appelée périodiquement.
        Met à jour les positions, initialise la formation et applique le contrôle.
        """
        self.update_my_position()
        self.publish_my_position()
        self.update_other_robot_positions()
        # Affichage périodique pour debug
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
        if self._debug_counter % 50 == 0:
            self.get_logger().info(f"Ma position: {self.my_position}")
            self.get_logger().info(f"Positions autres robots: {self.other_robot_positions}")
            self.get_logger().info(f"Formation initialisée: {self.formation_initialized}")
        # Initialisation de la formation si toutes les positions sont connues
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.get_logger().info("Formation initialized")
        # Application du contrôle si tout est prêt
        if self.active and self.formation_initialized and self.goal_point_set:
            self.apply_consensus_control()

    #-----------------------------#
    #   Gestion des positions     #
    #-----------------------------#
    def update_my_position(self):
        """
        Met à jour la position du robot courant via TF2.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"{self.robot_name}/base_link", rclpy.time.Time()
            )
            pos = trans.transform.translation
            self.my_position = {'x': pos.x, 'y': pos.y}
        except Exception as e:
            self.get_logger().warn(f"Échec TF2 pour {self.robot_name}: {e}")

    def publish_my_position(self):
        """
        Publie la position du robot courant sur un topic dédié.
        """
        point_msg = Point()
        point_msg.x = float(self.my_position['x'])
        point_msg.y = float(self.my_position['y'])
        point_msg.z = 0.0
        self.position_publisher.publish(point_msg)

    def update_other_robot_positions(self):
        """
        Met à jour les positions des voisins via TF2 ou via les topics.
        """
        # Récupérer la liste des voisins depuis la config
        neighbors_names = self.robot_neighbors.get(self.robot_name, [])
        if not neighbors_names:
            neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        for robot_name in neighbors_names:
            position_updated = False
            try:
                trans = self.tf_buffer.lookup_transform(
                    GLOBAL_FRAME, f"{robot_name}/base_link", rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                pos = trans.transform.translation
                self.other_robot_positions[robot_name] = {'x': pos.x, 'y': pos.y}
                position_updated = True
            except Exception:
                pass
            if not position_updated:
                if robot_name in self.robot_positions and self.robot_positions[robot_name] is not None:
                    self.other_robot_positions[robot_name] = self.robot_positions[robot_name]
                else:
                    self.other_robot_positions[robot_name] = None

    def all_positions_available(self):
        """
        Vérifie si toutes les positions nécessaires sont connues (seulement pour les voisins).
        """
        if abs(self.my_position['x']) < 0.001 and abs(self.my_position['y']) < 0.001:
            return False
        neighbors_names = self.robot_neighbors.get(self.robot_name, [])
        if not neighbors_names:
            neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        for name in neighbors_names:
            if name not in self.other_robot_positions or self.other_robot_positions[name] is None:
                return False
        return True

    #-----------------------------#
    #   Initialisation formation  #
    #-----------------------------#
    def initialize_formation(self):
        """
        Initialise la formation : distances désirées et vecteur relatif initial.
        """
        if self.formation_initialized:
            self.get_logger().warn("Formation already initialized! Skipping re-initialization.")
            return
        initial_barycenter = self.compute_swarm_center()
        self.initial_relative_vector = np.array([
            self.my_position['x'] - initial_barycenter[0],
            self.my_position['y'] - initial_barycenter[1]
        ])
        self.get_logger().info(f"Vecteur relatif initial calculé: {self.initial_relative_vector}")
        neighbors_names = self.robot_neighbors.get(self.robot_name, [])
        if not neighbors_names:
            neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        for neighbor_name in neighbors_names:
            if neighbor_name in self.other_robot_positions and self.other_robot_positions[neighbor_name] is not None:
                other_pos = self.other_robot_positions[neighbor_name]
                dist = math.sqrt(
                    (self.my_position['x'] - other_pos['x'])**2 +
                    (self.my_position['y'] - other_pos['y'])**2
                ) if CHOICE else FIXED_DISTANCE
                self.desired_distances[neighbor_name] = dist
        self.formation_initialized = True
        self.get_logger().info(f"Initialized formation with distances: {self.desired_distances}")
        barycentre = self.compute_swarm_center()
        self.get_logger().info(
            f"Barycentre (init): X:{barycentre[0]:.3f} ; Y:{barycentre[1]:.3f}"
        )

    def compute_swarm_center(self):
        """
        Calcule le barycentre de l'essaim (ou l'estime si TF2 barycenter indisponible).
        Utilise uniquement la position du robot et de ses voisins.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"barycenter", rclpy.time.Time()
            )
            pos = trans.transform.translation
            return [pos.x, pos.y]
        except Exception:
            neighbors_names = self.robot_neighbors.get(self.robot_name, [])
            if not neighbors_names:
                neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
            all_positions = [self.my_position] + [self.other_robot_positions[n] for n in neighbors_names if n in self.other_robot_positions and self.other_robot_positions[n] is not None]
            if not all_positions:
                return [self.my_position['x'], self.my_position['y']]
            total_x = sum(pos['x'] for pos in all_positions)
            total_y = sum(pos['y'] for pos in all_positions)
            count = len(all_positions)
            return [total_x / count, total_y / count]

    #-----------------------------#
    #   Contrôle de consensus     #
    #-----------------------------#
    def transform_velocity(self, global_lin_x, global_lin_y):
        """
        Transforme une vitesse du repère global vers le repère du robot.
        """
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
            self.get_logger().error(f'Erreur de transformation: {e}')
            return global_lin_x, global_lin_y

    def apply_consensus_control(self):
        """
        Applique le contrôle de consensus pour maintenir la formation et suivre l'objectif.
        """
        if not self.goal_point_set:
            return
        global_goal = np.array(self.goal_point)
        pr = global_goal + self.initial_relative_vector
        pi = np.array([self.my_position['x'], self.my_position['y']])
        neighbors_names = self.robot_neighbors.get(self.robot_name, [])
        if not neighbors_names:
            neighbors_names = [r for r in ALL_ROBOT_NAMES if r != self.robot_name]
        pj_array = []
        dij_list = []
        distance_info = []
        for neighbor_name in neighbors_names:
            if neighbor_name in self.other_robot_positions and self.other_robot_positions[neighbor_name] is not None:
                other_pos = self.other_robot_positions[neighbor_name]
                pj = np.array([other_pos['x'], other_pos['y']])
                pj_array.append(pj)
                current_distance = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                dij = self.desired_distances.get(neighbor_name)
                if dij is None:
                    dij = current_distance
                dij_list.append(dij)
                distance_info.append(f"{neighbor_name}: actuelle={current_distance:.3f}m, désirée={dij:.3f}m")
        if distance_info:
            distances_str = ", ".join(distance_info)
            self.get_logger().info(f"Robot {self.robot_name} - Distances: {distances_str}")
        if not pj_array:
            # Si aucun voisin, aller simplement vers le point cible individuel
            control_vector = -(c1_gamma * (pi - pr))
            self.integral_term = None
            self.derivative_term = None
            ui_alpha = np.array([0.0, 0.0])
            ui_gamma = control_vector
            self.previous_gamma = ui_gamma  # Met à jour previous_gamma même dans ce cas
        else:
            # Contrôle avec prise en compte des voisins
            try:
                # Try new version (6 values)
                control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma, updated_gamma = control_with_components(
                    pj_array=pj_array,
                    pi=pi,
                    dij_list=dij_list,
                    pr=pr,
                    dt=self.dt,
                    integral_term=self.integral_term,
                    derivative_term=self.derivative_term,
                    is_rotating=False,
                    logger=self.get_logger(),
                    previous_gamma=self.previous_gamma
                )
                self.previous_gamma = updated_gamma
            except TypeError:
                # Fallback to old version (5 values)
                control_vector, updated_integral, updated_derivative, ui_alpha, ui_gamma = control_with_components(
                    pj_array=pj_array,
                    pi=pi,
                    dij_list=dij_list,
                    pr=pr,
                    dt=self.dt,
                    integral_term=self.integral_term,
                    derivative_term=self.derivative_term,
                    is_rotating=False,
                    logger=self.get_logger(),
                    previous_gamma=self.previous_gamma
                )
                self.previous_gamma = ui_gamma
            self.integral_term = updated_integral
            self.derivative_term = updated_derivative
        # Publier les composantes de contrôle pour debug
        control_msg = Float64MultiArray()
        control_msg.data = [float(ui_alpha[0]), float(ui_alpha[1]), float(ui_gamma[0]), float(ui_gamma[1])]
        self.control_component_publisher.publish(control_msg)
        # Calculer la commande de vitesse dans le repère du robot
        robot_lin_x, robot_lin_y = self.transform_velocity(
            control_vector[0], control_vector[1]
        )
        twist_msg = Twist()
        twist_msg.linear.x = float(robot_lin_x)
        twist_msg.linear.y = float(robot_lin_y)
        # Limiter la vitesse maximale
        max_speed = 0.14
        speed = math.sqrt(twist_msg.linear.x**2 + twist_msg.linear.y**2)
        if speed > max_speed:
            scaling = max_speed / speed
            twist_msg.linear.x *= scaling
            twist_msg.linear.y *= scaling
        # Vérifier si la cible individuelle est atteinte
        current_state = self.is_robot_target_reached(pi, pr)
        if current_state != self.is_target_reached_state:
            self.is_target_reached_state = current_state
            self.publish_target_status(1 if current_state else 0)
            if current_state:
                self.get_logger().info("Individual target reached!")
            else:
                self.get_logger().info("Individual target not reached")
        # Publier la commande de vitesse
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(
            f"Robot {self.robot_name} (voisins: {len(neighbors_names)}): Global:{control_vector[0]:.3f},{control_vector[1]:.3f} -> Robot:{twist_msg.linear.x:.3f},{twist_msg.linear.y:.3f}"
        )

    def is_robot_target_reached(self, robot_pos, target_pos):
        """
        Vérifie si le robot est suffisamment proche de son objectif individuel.
        """
        distance = math.sqrt((robot_pos[0] - target_pos[0])**2 + (robot_pos[1] - target_pos[1])**2)
        self.get_logger().info(f"distance to individual goal: {distance:.3f}")
        return distance <= self.target_tolerance

    def publish_target_status(self, status):
        """
        Publie le statut d'atteinte de la cible individuelle.
        """
        msg = Int32()
        msg.data = status
        self.target_status_publisher.publish(msg)

    def stop_robot(self):
        """
        Arrête le robot en publiant une commande de vitesse nulle.
        """
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        self.get_logger().info(f"Robot {self.robot_name} stopped")

def main(args=None):
    """
    Point d'entrée principal du noeud ROS2.
    """
    rclpy.init(args=args)
    node = DistributedSwarm2Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
