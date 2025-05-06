#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Vector3
import sys
import termios
import tty
import select
import threading
import math
from tf2_ros import Buffer, TransformListener, TransformException
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
        # Un "frame" est un repère de coordonnées utilisé pour définir la position et l'orientation d'un objet dans l'espace.
        # Les transformations entre frames permettent de convertir des positions ou des vitesses d'un repère à un autre.
        self.tf_buffer = Buffer()  # Stocke les transformations entre différents frames (repères de coordonnées)
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Écoute les transformations publiées sur le réseau ROS2

        # Définir les frames globales et locales
        # Le frame global (global_frame_id) est le repère de référence pour l'ensemble du système (par exemple, "mocap").
        # Le frame local (robot_frame_id) est le repère attaché au robot (par exemple, "base_link").
        self.global_frame_id = "mocap"  # Frame de référence globale (souvent utilisé pour la navigation)
        self.robot_frame_id = f"{robot_name}/base_link"  # Frame spécifique au robot (attaché au robot)

        # Déclaration des paramètres pour permettre une configuration dynamique
        # Ces paramètres permettent de modifier les frames globales et locales via des outils ROS2 (comme ros2 param).
        self.declare_parameter('global_frame_id', self.global_frame_id)  # Paramètre pour le frame global
        self.declare_parameter('robot_frame_id', self.robot_frame_id)  # Paramètre pour le frame du robot

        # Mise à jour des frames à partir des paramètres si disponibles
        # Si les paramètres sont définis dans un fichier de configuration ou via la ligne de commande, ils sont utilisés ici.
        self.global_frame_id = self.get_parameter('global_frame_id').get_parameter_value().string_value
        self.robot_frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value

        # Variables pour stocker les données de pose et d'orientation du robot
        self.pose = None  # Stocke la pose actuelle (position et orientation) du robot
        self.yaw = 0.0  # Stocke l'orientation (yaw) du robot en radians
        self.last_pose_time = self.get_clock().now()  # Temps de la dernière mise à jour de la pose

        # Journalisation pour indiquer que le nœud est prêt
        self.get_logger().info(f'Téléopération prête. Publie sur {cmd_vel_topic}')
        self.get_logger().info(f'Utilise {self.global_frame_id} comme frame globale et {self.robot_frame_id} comme frame du robot')

        # Indique si le nœud est en cours d'exécution
        self.running = True

        # Suivi de l'état de fonctionnement de TF2
        # Cette variable est utilisée pour détecter si TF2 fonctionne correctement ou si un fallback manuel est nécessaire.
        self.tf2_working = False

    def pose_callback(self, msg):
        """Callback pour les messages de pose du robot."""
        self.pose = msg
        self.last_pose_time = self.get_clock().now()
        
        # Extraire l'orientation (yaw) du quaternion
        _, _, self.yaw = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

    def manual_transform_velocity(self, global_lin_x, global_lin_y, global_ang_z):
        """
        Transforme manuellement les vitesses globales en vitesses dans le repère du robot
        en utilisant les données de pose VRPN lorsque TF2 échoue.
        Args:
            global_lin_x (float): Vitesse linéaire globale sur l'axe X.
            global_lin_y (float): Vitesse linéaire globale sur l'axe Y.
            global_ang_z (float): Vitesse angulaire globale autour de l'axe Z.
        Returns:
            tuple: Une tuple contenant:
                - robot_lin_x (float): Vitesse linéaire dans le repère du robot sur l'axe X.
                - robot_lin_y (float): Vitesse linéaire dans le repère du robot sur l'axe Y.
                - robot_ang_z (float): Vitesse angulaire dans le repère du robot autour de l'axe Z.
        Remarques:
            - Si les données de pose ne sont pas disponibles ou sont obsolètes (plus d'une seconde),
              les vitesses globales sont retournées sans transformation.
            - La transformation utilise une matrice de rotation 2D basée sur le yaw extrait des
              données de pose VRPN.
            - En cas d'erreur, les vitesses globales sont retournées et une erreur est journalisée.
        """
        try:
            # Vérifier si nous avons des données de pose
            if self.pose is None:
                return global_lin_x, global_lin_y, global_ang_z
            
            # Calculer le temps écoulé depuis la dernière mise à jour de la pose
            time_diff = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
            if time_diff > 1.0:  # Si la dernière pose date de plus d'une seconde
                self.get_logger().warning_throttle(5, f'Les données de pose sont obsolètes ({time_diff:.2f}s). Utilisation des vitesses globales.')
                return global_lin_x, global_lin_y, global_ang_z
            
            # Utiliser le yaw de la pose VRPN pour la transformation
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            
            # Transformer du repère global au repère du robot en utilisant une matrice de rotation 2D
            robot_lin_x = global_lin_x * cos_yaw + global_lin_y * sin_yaw
            robot_lin_y = -global_lin_x * sin_yaw + global_lin_y * cos_yaw
            
            # La vitesse angulaire reste inchangée
            robot_ang_z = global_ang_z
            
            return robot_lin_x, robot_lin_y, robot_ang_z
                
        except Exception as e:
            self.get_logger().error(f'Manual transformation error: {e}')
            return global_lin_x, global_lin_y, global_ang_z

    def transform_velocity(self, global_lin_x, global_lin_y, global_ang_z):
        """
        Transforme les vitesses du repère global au repère du robot.
        Utilise TF2 pour effectuer la transformation, avec un fallback manuel si TF2 échoue.
        """
        self.get_logger().info(
            f"Appel de transform_velocity avec global_lin_x={global_lin_x}, global_lin_y={global_lin_y}, global_ang_z={global_ang_z}"
        )
        try:
            # les entrées sont des floats
            global_lin_x = float(global_lin_x)
            global_lin_y = float(global_lin_y)
            global_ang_z = float(global_ang_z)
            
            # vecteur pour représenter la vitesse globale
            global_vel = Vector3()
            global_vel.x = global_lin_x
            global_vel.y = global_lin_y
            global_vel.z = 0.0  # Pas de mouvement en Z
            
            # TF2 pour transformer les vitesses
            try:
                # Récupérer la transformation entre les frames
                now = rclpy.time.Time()  # Temps actuel
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame_id,  # Frame cible (robot)
                    self.global_frame_id,  # Frame source (globale)
                    now,  # Temps de la transformation
                    timeout=rclpy.duration.Duration(seconds=0.05)  # Timeout pour la recherche
                )
                
                # Si TF2 fonctionne, loguer une confirmation
                if not self.tf2_working:
                    self.tf2_working = True
                    self.get_logger().info(f'Transformation TF2 réussie entre {self.global_frame_id} et {self.robot_frame_id}')
                
                # Appliquer la transformation au vecteur de vitesse
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                
                # Retourner les vitesses transformées
                return robot_vel.x, robot_vel.y, global_ang_z
                
            except TransformException as ex:
                # Si TF2 échoue, loguer une erreur et passer au fallback manuel
                if self.tf2_working:
                    self.tf2_working = False
                    self.get_logger().error(f'Échec de la transformation TF2 : {ex}')
                    self.get_logger().info('Utilisation de la transformation manuelle basée sur les données VRPN')
                
                # Fallback manuel
                return self.manual_transform_velocity(global_lin_x, global_lin_y, global_ang_z)
            
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
                        f'Global: linear.x={global_dx*0.5}, linear.y={global_dy*0.5}, angular.z={global_dth*1.0}'
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
