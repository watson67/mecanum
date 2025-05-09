#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Vector3, Vector3Stamped
import sys
import termios
import tty
import select
import threading
import math
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
import tf2_geometry_msgs
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# Noms des robots dans l'essaim
ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]

# TOPICS ROS pour les poses des robots
POSE_TOPICS = [f"/vrpn_mocap/{name}/pose" for name in ROBOT_NAMES]

# TOPICS ROS pour piloter les robots
CMD_VEL_TOPICS = [f"/{name}/cmd_vel" for name in ROBOT_NAMES]

# Configuration des touches pour un clavier AZERTY avec commandes dans le repère global
key_mapping = {
    'z': (0.14, 0.0, 0.0, 0.0),         # Avancer
    's': (-0.14, 0.0, 0.0, 0.0),        # Reculer
    'a': (0.099, 0.099, 0.0, 0.0),      # Diagonale avant-gauche
    'e': (0.099, -0.099, 0.0, 0.0),     # Diagonale avant-droit
    'w': (-0.099, 0.099, 0.0, 0.0),     # Diagonale arrière-gauche
    'x': (-0.099, -0.099, 0.0, 0.0),    # Diagonale arrière-droit
    'r': (0.0, 0.0, 0.0, 1.0),          # Rotation gauche sur place
    't': (0.0, 0.0, 0.0, -1.0),         # Rotation droite sur place
    'f': (0.07, 0.0, 0.0, 1.0),         # Virage avant-gauche
    'g': (0.07, 0.0, 0.0, -1.0),        # Virage avant-droit
    'c': (-0.07, 0.0, 0.0, 1.0),        # Virage arrière-gauche
    'v': (-0.07, 0.0, 0.0, -1.0),       # Virage arrière-droit
    'q': (0.0, 0.14, 0.0, 0.0),         # Translation gauche
    'd': (0.0, -0.14, 0.0, 0.0),        # Translation droite
    ' ': (0.0, 0.0, 0.0, 0.0),          # Stop
}

# QoS compatible avec VRPN
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

def get_key():
    """Lit une touche clavier sans bloquer et restaure le terminal"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(fd)
        dr, _, _ = select.select([sys.stdin], [], [], 0.01)
        if dr:
            key = sys.stdin.read(1)
        else:
            key = ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    return key

def euler_from_quaternion(x, y, z, w):
    """Convertit un quaternion en angles d'Euler (roll, pitch, yaw)."""
    # Roll (rotation autour de l'axe x)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation autour de l'axe y)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  
    else:
        pitch = math.asin(sinp)

    # Yaw (rotation autour de l'axe z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class MultiRobotTeleop(Node):
    def __init__(self):
        super().__init__('multi_robot_teleop')
        
        # Initialize all instance attributes explicitly
        # Définir les frames globales et locales
        self.global_frame_id = "mocap"
        self.publishers = {}  # Dictionary for publishers
        self.poses = {}       # Dictionary for poses
        self.robot_frames = {} # Dictionary for robot frames
        self.running = True   # Flag for thread control
        
        # Déclaration des paramètres pour permettre une configuration dynamique
        self.declare_parameter('global_frame_id', self.global_frame_id)
        self.global_frame_id = self.get_parameter('global_frame_id').get_parameter_value().string_value
        
        # Initialisation de TF2 pour gérer les transformations entre frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Ajout d'un broadcaster TF pour publier les transformations
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Mise en place des publishers et subscribers pour chaque robot
        for i, robot_name in enumerate(ROBOT_NAMES):
            # Frames pour chaque robot
            robot_frame_id = f"{robot_name}/base_link"
            self.robot_frames[robot_name] = robot_frame_id
            
            # Création du publisher pour envoyer les commandes de vitesse
            self.publishers[robot_name] = self.create_publisher(Twist, CMD_VEL_TOPICS[i], 1)
            
            # Abonnement au topic de pose du robot
            self.create_subscription(
                PoseStamped, 
                POSE_TOPICS[i], 
                lambda msg, rname=robot_name: self.pose_callback(msg, rname), 
                qos_profile
            )
            
            self.get_logger().info(f'Configuration pour {robot_name}: frame={robot_frame_id}, cmd_vel={CMD_VEL_TOPICS[i]}, pose={POSE_TOPICS[i]}')
        
        self.get_logger().info('Téléopération multi-robots prête')
        
    def pose_callback(self, msg, robot_name):
        """
        Callback pour les messages de pose des robots.
        Publie la transformation entre le repère global et le repère de chaque robot.
        """
        self.poses[robot_name] = msg
        
        # Créer et publier la transformation TF
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.global_frame_id
        transform.child_frame_id = self.robot_frames[robot_name]
        
        # Copier les données de position
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        
        # Copier les données d'orientation (quaternion)
        transform.transform.rotation.x = msg.pose.orientation.x
        transform.transform.rotation.y = msg.pose.orientation.y
        transform.transform.rotation.z = msg.pose.orientation.z
        transform.transform.rotation.w = msg.pose.orientation.w
        
        # Publier la transformation
        self.tf_broadcaster.sendTransform(transform)
        
        # Log pour confirmer la publication de la transformation (en debug pour éviter trop de messages)
        self.get_logger().debug(f'Transformation publiée pour {robot_name}: {self.global_frame_id} -> {self.robot_frames[robot_name]}')

    def transform_velocity_for_robot(self, robot_name, global_lin_x, global_lin_y, global_ang_z):
        """
        Transforme les vitesses du repère global au repère d'un robot spécifique.
        """
        try:
            if robot_name not in self.robot_frames:
                self.get_logger().error(f'Robot {robot_name} non configuré')
                return global_lin_x, global_lin_y, global_ang_z
                
            # Créer un vecteur estampillé pour représenter la vitesse globale
            global_vel = Vector3Stamped()
            global_vel.header.frame_id = self.global_frame_id
            global_vel.header.stamp = self.get_clock().now().to_msg()
            global_vel.vector.x = float(global_lin_x)
            global_vel.vector.y = float(global_lin_y)
            global_vel.vector.z = 0.0 
            
            # Récupérer la transformation entre les frames
            try:
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frames[robot_name],  # Frame cible (robot)
                    self.global_frame_id,  # Frame source (globale)
                    now,  # Temps de la transformation
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Appliquer la transformation au vecteur de vitesse
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                
                # Retourner les vitesses transformées
                return robot_vel.vector.x, robot_vel.vector.y, global_ang_z
                
            except TransformException as ex:
                # Si TF2 échoue, loguer une erreur
                self.get_logger().warning(f'Échec de transformation TF2 pour {robot_name}: {ex}')
                return global_lin_x, global_lin_y, global_ang_z
            
        except Exception as e:
            self.get_logger().error(f'Erreur dans transform_velocity pour {robot_name}: {e}')
            return global_lin_x, global_lin_y, global_ang_z

    def send_command_to_all_robots(self, global_dx, global_dy, global_dth):
        """
        Transforme et envoie les commandes de vitesse à tous les robots.
        """
        for robot_name in ROBOT_NAMES:
            twist = Twist()
            
            # Transformer les vitesses du repère global au repère du robot
            robot_dx, robot_dy, robot_dth = self.transform_velocity_for_robot(
                robot_name, global_dx, global_dy, global_dth
            )
            
            # Créer le message Twist avec les vitesses dans le repère du robot
            twist.linear.x = robot_dx
            twist.linear.y = robot_dy
            twist.angular.z = robot_dth
            
            # Publier la commande
            if robot_name in self.publishers:
                self.publishers[robot_name].publish(twist)
                self.get_logger().debug(f'{robot_name}: Vitesse robot: x={robot_dx:.3f}, y={robot_dy:.3f}, th={robot_dth:.3f}')

    def spin_thread(self):
        """Thread pour exécuter rclpy.spin."""
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.01)

    def run(self):
        spin_thread = threading.Thread(target=self.spin_thread)
        spin_thread.start()

        try:
            while rclpy.ok():
                key = get_key()

                if key == '\x03':  # CTRL+C pour quitter
                    break

                if key in key_mapping:
                    global_dx, global_dy, _, global_dth = key_mapping[key]
                    
                    # Afficher les commandes globales
                    self.get_logger().info(f'Commande globale: x={global_dx:.3f}, y={global_dy:.3f}, th={global_dth:.3f} (touche={key})')
                    
                    # Envoyer les commandes à tous les robots
                    self.send_command_to_all_robots(global_dx, global_dy, global_dth)
                elif key:  # Si une touche non définie est pressée
                    # Envoyer commande d'arrêt à tous les robots
                    self.get_logger().info(f'Touche non reconnue: {key} - Arrêt des robots')
                    self.send_command_to_all_robots(0.0, 0.0, 0.0)
                # Si aucune touche n'est pressée, ne rien envoyer
        except Exception as e:
            self.get_logger().error(f'Erreur dans la boucle principale: {e}')
        finally:
            self.running = False
            spin_thread.join()

def main(args=None):
    rclpy.init(args=args)
    
    # Créer un seul nœud qui gère tous les robots
    multi_teleop = MultiRobotTeleop()
    
    try:
        multi_teleop.run()
    except Exception as e:
        multi_teleop.get_logger().error(f'Erreur dans le nœud multi-robot: {e}')
    finally:
        # Cleanup
        multi_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
