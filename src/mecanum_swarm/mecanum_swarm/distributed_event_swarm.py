#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32, String
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3Stamped
import tf2_geometry_msgs
from tf2_ros import TransformException
import socket
import numpy as np
# Import formules.py
from mecanum_swarm.formules import *

'''
Version distribuée du contrôleur d'essaim.
Ce programme doit s'exécuter sur chaque robot individuellement.
Chaque robot contrôle son propre mouvement tout en maintenant la formation avec les autres.
'''

#--------------------------------------------------------------------
# Variables globales
#--------------------------------------------------------------------
# Noms possibles des robots dans l'essaim
ALL_ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]  # Liste de tous les robots possibles
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
        
        # Pas de temps pour l'intégration
        self.dt = 0.1
        
        # Tolérance pour considérer que la cible est atteinte
        self.target_tolerance = 0.05  # aligné avec swarm.py
        
        # État d'atteinte de la cible
        self.is_target_reached_state = False

        # Timer pour le contrôle périodique
        self.create_timer(self.dt, self.timer_callback)

        # Souscription au topic '/formation' pour réinitialiser la formation à la demande
        self.create_subscription(
            Int32, "/formation", self.formation_callback, 10
        )

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

    def all_positions_available(self):
        """Vérifie si toutes les positions des robots sont connues (non None)"""
        # Vérifie la position du robot courant
        if self.my_position['x'] == 0.0 and self.my_position['y'] == 0.0:
            return False
        # Vérifie les positions des autres robots
        for name in ALL_ROBOT_NAMES:
            if name != self.robot_name:
                if name not in self.other_robot_positions or self.other_robot_positions[name] is None:
                    return False
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
        
        # Initialiser la formation si ce n'est pas déjà fait et toutes les positions sont connues
        if not self.formation_initialized and self.all_positions_available():
            self.initialize_formation()
            self.formation_initialized = True
            self.get_logger().info("Formation initialized")
            
        # Déterminer le goal point à utiliser
        if self.goal_point is not None:
            goal_point = self.goal_point
        else:
            barycentre = self.compute_swarm_center()
            goal_point = (barycentre[0], barycentre[1])

        # Appliquer le contrôle de consensus si actif
        if self.active and self.formation_initialized and self.goal_point_set:
            self.apply_consensus_control(goal_point)
            
            # Vérifier si la cible est atteinte
            if self.goal_point_set:
                swarm_center = self.compute_swarm_center()
                self.get_logger().info(
                    f"Barycentre : X:{swarm_center[0]:.3f} ; Y:{swarm_center[1]:.3f}"
                )
                current_state = self.is_target_reached(swarm_center, goal_point)
                
                # Si l'état a changé, publier le statut
                if current_state != self.is_target_reached_state:
                    self.is_target_reached_state = current_state
                    self.publish_target_status(1 if current_state else 0)
                    
                    if current_state:
                        self.get_logger().info("Target reached!")
                    else:
                        self.get_logger().info("Target not reached")

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
        for other_name in ALL_ROBOT_NAMES:
            if other_name != self.robot_name and other_name in self.other_robot_positions:
                other_pos = self.other_robot_positions[other_name]
                
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
        all_positions = [self.my_position] + list(self.other_robot_positions.values())
        if not all_positions:
            return [self.my_position['x'], self.my_position['y']]
            
        total_x = sum(pos['x'] for pos in all_positions if pos is not None)
        total_y = sum(pos['y'] for pos in all_positions if pos is not None)
        count = sum(1 for pos in all_positions if pos is not None)
        
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
        """Appliquer le contrôle de consensus pour ce robot"""
        # Point de référence (objectif)
        pr = np.array(goal_point)
        
        # Position de ce robot
        pi = np.array([self.my_position['x'], self.my_position['y']])
        
        # Positions des voisins et distances désirées
        pj_array = []
        dij_list = []
        
        # Pour chaque voisin
        for other_name, other_pos in self.other_robot_positions.items():
            if other_pos is not None:
                # Position du voisin
                pj = np.array([other_pos['x'], other_pos['y']])
                pj_array.append(pj)
                
                # Distance désirée
                dij = self.desired_distances.get(other_name, 2.0) 
                dij_list.append(dij)
        
        # S'il n'y a pas de voisins, juste aller vers l'objectif
        if not pj_array:
            # Vecteur simple vers l'objectif
            control_vector = -(c1_gamma * (pi - pr))
            self.integral_term = None
        else:
            # Appliquer la fonction de contrôle avec les voisins
            control_vector, updated_integral = control(
                pj_array=pj_array,
                pi=pi,
                dij_list=dij_list,
                pr=pr,
                dt=self.dt,
                integral_term=self.integral_term
            )
            
            # Mettre à jour le terme intégral
            self.integral_term = updated_integral
        
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
        self.get_logger().info(
            f"Robot {self.robot_name}: Global:{control_vector[0]:.3f},{control_vector[1]:.3f} -> Robot:{twist_msg.linear.x:.3f},{twist_msg.linear.y:.3f}"
        )
    
    def is_target_reached(self, swarm_center, goal):
        """Vérifier si le centre de l'essaim est proche de l'objectif"""
        distance = math.sqrt((swarm_center[0] - goal[0])**2 + (swarm_center[1] - goal[1])**2)
        self.get_logger().info(f"distance to goal: {distance:.3f}")
        return distance <= self.target_tolerance
    
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
