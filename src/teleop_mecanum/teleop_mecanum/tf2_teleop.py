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

class VRPNTeleop(Node):
    def __init__(self, cmd_vel_topic, robot_name):
        super().__init__('global_frame_teleop')
        
        # Vérifier que le nom du robot est valide parmis les 3 robots disponibles
        if robot_name not in ROBOT_NAMES:
            self.get_logger().error(f"Le nom du robot doit être l'un des suivants : {', '.join(ROBOT_NAMES)}")
            raise ValueError(f"Nom de robot invalide : {robot_name}")
        
        self.robot_name = robot_name
        self.pose_topic = f"/vrpn_mocap/{robot_name}/pose"
        
        # Création du publisher pour envoyer les commandes de vitesse
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 1)
        
        # Abonnement au topic de pose du robot
        self.subscription = self.create_subscription(
            PoseStamped, 
            self.pose_topic, 
            self.pose_callback, 
            qos_profile  # QoS adapté pour VRPN
        )
        
        # Initialisation de TF2 pour gérer les transformations entre frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Ajout d'un broadcaster TF pour publier les transformations
        self.tf_broadcaster = TransformBroadcaster(self)

        # Définir les frames globales et locales
        self.global_frame_id = "mocap"
        self.robot_frame_id = f"{robot_name}/base_link"

        # Déclaration des paramètres pour permettre une configuration dynamique
        self.declare_parameter('global_frame_id', self.global_frame_id)
        self.declare_parameter('robot_frame_id', self.robot_frame_id)

        # Mise à jour des frames à partir des paramètres si disponibles
        self.global_frame_id = self.get_parameter('global_frame_id').get_parameter_value().string_value
        self.robot_frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value

        # Variables pour stocker les données de pose et d'orientation du robot
        self.pose = None
        self.yaw = 0.0
        self.last_pose_time = self.get_clock().now()

        # Journalisation pour indiquer que le nœud est prêt
        self.get_logger().info(f'Téléopération prête. Publie sur {cmd_vel_topic}')
        self.get_logger().info(f'Utilise {self.global_frame_id} comme frame globale et {self.robot_frame_id} comme frame du robot')

        # Indique si le nœud est en cours d'exécution
        self.running = True

    def pose_callback(self, msg):
        """
        Callback pour les messages de pose du robot.
        Publie la transformation entre le repère global et le repère du robot.
        """
        self.pose = msg
        self.last_pose_time = self.get_clock().now()
        
        # Extraire l'orientation (yaw) du quaternion pour les logs
        _, _, self.yaw = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        
        # Créer et publier la transformation TF
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.global_frame_id
        transform.child_frame_id = self.robot_frame_id
        
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
        
        # Log pour confirmer la publication de la transformation
        self.get_logger().debug(f'Transformation publiée: {self.global_frame_id} -> {self.robot_frame_id}')

    def transform_velocity(self, global_lin_x, global_lin_y, global_ang_z):
        """
        Transforme les vitesses du repère global au repère du robot en utilisant TF2.
        """
        try:
            # les entrées sont des floats
            global_lin_x = float(global_lin_x)
            global_lin_y = float(global_lin_y)
            global_ang_z = float(global_ang_z)
            
            # créer un vecteur estampillé (Vector3Stamped) pour représenter la vitesse globale
            global_vel = Vector3Stamped()
            global_vel.header.frame_id = self.global_frame_id
            global_vel.header.stamp = self.get_clock().now().to_msg()
            global_vel.vector.x = global_lin_x
            global_vel.vector.y = global_lin_y
            global_vel.vector.z = 0.0 
            
            # Récupérer la transformation entre les frames
            try:
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame_id,  # Frame cible (robot)
                    self.global_frame_id,  # Frame source (globale)
                    now,  # Temps de la transformation
                    timeout=rclpy.duration.Duration(seconds=0.1)  # Timeout augmenté
                )
                
                # Appliquer la transformation au vecteur de vitesse
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                
                # Retourner les vitesses transformées
                return robot_vel.vector.x, robot_vel.vector.y, global_ang_z
                
            except TransformException as ex:
                # Si TF2 échoue, loguer une erreur détaillée
                self.get_logger().error(f'Échec de la transformation TF2 : {ex}')
                self.get_logger().info(f'Verifiez que la transformation entre {self.global_frame_id} et {self.robot_frame_id} est disponible dans le tf_tree')
                
                # Si pas de transformation disponible, retourner les vitesses globales
                return global_lin_x, global_lin_y, global_ang_z
            
        except Exception as e:
            # Loguer toute autre erreur
            self.get_logger().error(f'Erreur dans transform_velocity : {e}')
            return global_lin_x, global_lin_y, global_ang_z

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
                twist = Twist()

                if key == '\x03':  # CTRL+C pour quitter
                    break

                if key in key_mapping:
                    global_dx, global_dy, _, global_dth = key_mapping[key]
                    
                    # Transformer les vitesses du repère global au repère du robot
                    robot_dx, robot_dy, robot_dth = self.transform_velocity(global_dx, global_dy, global_dth)
                    
                    # Créer le message Twist avec les vitesses dans le repère du robot
                    twist.linear.x = robot_dx
                    twist.linear.y = robot_dy
                    twist.angular.z = robot_dth * 1.0

                    # Ajouter un log pour afficher les vitesses globales et les vitesses du robot sur deux lignes
                    self.get_logger().info(
                        f'Global: linear.x={global_dx}, linear.y={global_dy}, angular.z={global_dth}'
                    )
                    self.get_logger().info(
                        f'Robot: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                    )
                    self.publisher.publish(twist)
                elif key:  # Si une touche non définie est pressée
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0

                    self.get_logger().info(
                        f'Publication Twist: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                    )
                    self.publisher.publish(twist)
                # Si aucune touche n'est pressée, ne rien envoyer
        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')
        finally:
            self.running = False
            spin_thread.join()

def main(args=None):
    rclpy.init(args=args)
    
    # Récupérer les paramètres de ligne de commande
    if len(sys.argv) < 2:
        print(f"Usage: ros2 run teleop_mecanum tf2_teleop [robot_name]")
        print(f"  où robot_name est l'un des suivants: {', '.join(ROBOT_NAMES)}")
        rclpy.shutdown()
        return

    robot_name = sys.argv[1]
    
    if robot_name not in ROBOT_NAMES:
        print(f"Erreur: Nom du robot invalide. Le nom doit être l'un des suivants: {', '.join(ROBOT_NAMES)}")
        rclpy.shutdown()
        return

    # Définir le topic de publication basé sur le nom du robot
    cmd_vel_topic = f"/{robot_name}/cmd_vel"
    
    try:
        node = VRPNTeleop(cmd_vel_topic, robot_name)
        node.run()
    except ValueError as e:
        print(str(e))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
