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
import queue
import time
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

# Noms des robots dans l'essaim
ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]

# TOPICS ROS pour les poses des robots
POSE_TOPICS = [f"/vrpn_mocap/{name}/pose" for name in ROBOT_NAMES]

# TOPICS ROS pour piloter les robots
CMD_VEL_TOPICS = [f"/{name}/cmd_vel" for name in ROBOT_NAMES]

# Configuration des touches pour un clavier AZERTY avec commandes dans le repère global
key_mapping = {
    'z': (0.14, 0.0, 0.0, 0.0),    # Avancer
    's': (-0.14, 0.0, 0.0, 0.0),   # Reculer
    'a': (0.099, 0.099, 0.0, 0.0),  # Diagonale avant-gauche
    'e': (0.099, -0.099, 0.0, 0.0),  # Diagonale avant-droit
    'w': (-0.099, 0.099, 0.0, 0.0),  # Diagonale arrière-gauche
    'x': (-0.099, -0.099, 0.0, 0.0),  # Diagonale arrière-droit
    'r': (0.0, 0.0, 0.0, 1.0),    # Rotation gauche sur place
    't': (0.0, 0.0, 0.0, -1.0),   # Rotation droite sur place
    'f': (0.07, 0.0, 0.0, 1.0),  # Virage avant-gauche
    'g': (0.07, 0.0, 0.0, -1.0), # Virage avant-droit
    'c': (-0.07, 0.0, 0.0, 1.0), # Virage arrière-gauche
    'v': (-0.07, 0.0, 0.0, -1.0),# Virage arrière-droit
    'q': (0.0, 0.14, 0.0, 0.0),    # Translation gauche
    'd': (0.0, -0.14, 0.0, 0.0),   # Translation droite
    ' ': (0.0, 0.0, 0.0, 0.0),    # Stop
}

# QoS compatible avec VRPN
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

qos_profile2 = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

def get_key():
    """Lit une touche clavier sans bloquer et restaure le terminal correctement."""
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
        pitch = math.copysign(math.pi / 2, sinp)  # Utilise 90 degrés si hors de portée
    else:
        pitch = math.asin(sinp)

    # Yaw (rotation autour de l'axe z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class RobotCommandThread(threading.Thread):
    """Thread dédié pour le traitement et la publication des commandes pour un robot spécifique."""
    
    def __init__(self, node, robot_name):
        super().__init__(name=f"{robot_name}_thread")
        self.node = node
        self.robot_name = robot_name
        self.command_queue = queue.Queue()
        self.running = True
        self.daemon = True  # Le thread s'arrêtera quand le thread principal s'arrête
    
    def run(self):
        """Boucle principale du thread de commande du robot."""
        while self.running:
            try:
                # Attendre une commande avec timeout pour permettre de vérifier self.running
                try:
                    command = self.command_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                global_dx, global_dy, global_dth = command
                
                # Transformer les vitesses du repère global au repère du robot
                robot_dx, robot_dy, robot_dth = self.node.transform_velocity(
                    global_dx, global_dy, global_dth, self.robot_name
                )
                
                # Créer le message Twist avec les vitesses dans le repère du robot
                twist = Twist()
                twist.linear.x = robot_dx 
                twist.linear.y = robot_dy 
                twist.angular.z = robot_dth * 1.0
                
                # Log pour ce robot spécifique
                self.node.get_logger().info(
                    f'{self.robot_name}: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                )
                
                # Publier la commande pour ce robot
                self.node.cmd_vel_publishers[self.robot_name].publish(twist)
                
                # Marquer la tâche comme terminée
                self.command_queue.task_done()
                
            except Exception as e:
                self.node.get_logger().error(f'{self.robot_name}: Erreur dans le thread de commande: {e}')
    
    def send_command(self, global_dx, global_dy, global_dth):
        """Envoie une commande au thread pour traitement."""
        self.command_queue.put((global_dx, global_dy, global_dth))
    
    def stop(self):
        """Arrête le thread de commande."""
        self.running = False
        # Ajouter une commande vide pour débloquer le thread s'il attend
        self.command_queue.put((0, 0, 0))

class VRPNTeleop(Node):
    def __init__(self):
        super().__init__('multi_robot_teleop')
        
        # Création des publishers pour envoyer les commandes de vitesse à tous les robots
        self.cmd_vel_publishers = {}
        for name, topic in zip(ROBOT_NAMES, CMD_VEL_TOPICS):
            self.cmd_vel_publishers[name] = self.create_publisher(
                Twist,
                topic,
                qos_profile2
            )
            self.get_logger().info(f"Créé publisher pour {name} sur {topic}")

        # Initialisation des données de pose pour chaque robot
        self.poses = {name: None for name in ROBOT_NAMES}
        self.yaws = {name: 0.0 for name in ROBOT_NAMES}
        self.last_pose_times = {name: self.get_clock().now() for name in ROBOT_NAMES}
        
        # Abonnements aux sujets des poses des robots
        for name, topic in zip(ROBOT_NAMES, POSE_TOPICS):
            self.create_subscription(
                PoseStamped,
                topic,
                self.make_pose_callback(name),
                qos_profile
            )
            self.get_logger().info(f"Créé subscription pour {name} sur {topic}")
        
        # Initialisation de TF2 pour gérer les transformations entre frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Configuration des frames
        self.global_frame_id = "odom"
        self.robot_frame_ids = {name: f"{name}/base_link" for name in ROBOT_NAMES}
        
        # Déclaration des paramètres
        self.declare_parameter('global_frame_id', self.global_frame_id)
        for name in ROBOT_NAMES:
            self.declare_parameter(f'{name}_frame_id', self.robot_frame_ids[name])
        
        # Mise à jour des frames à partir des paramètres
        self.global_frame_id = self.get_parameter('global_frame_id').get_parameter_value().string_value
        for name in ROBOT_NAMES:
            param_name = f'{name}_frame_id'
            if self.has_parameter(param_name):
                self.robot_frame_ids[name] = self.get_parameter(param_name).get_parameter_value().string_value
        
        self.running = True
        
        # Suivi de l'état de fonctionnement de TF2 pour chaque robot
        self.tf2_working = {name: False for name in ROBOT_NAMES}
        
        self.get_logger().info(f'Téléopération multi-robots prête. Publie sur {", ".join(CMD_VEL_TOPICS)}')

        # Initialisation des threads de commande pour chaque robot
        self.robot_threads = {}
        for name in ROBOT_NAMES:
            self.robot_threads[name] = RobotCommandThread(self, name)
            self.robot_threads[name].start()
            self.get_logger().info(f"Thread de commande démarré pour {name}")

    def make_pose_callback(self, robot_name):
        """
        Factory de callback pour les messages de pose des robots.
        Crée un callback spécifique pour chaque robot.
        """
        def callback(msg):
            self.poses[robot_name] = msg
            self.last_pose_times[robot_name] = self.get_clock().now()
            
            # Extraire l'orientation (yaw) du quaternion
            _, _, self.yaws[robot_name] = euler_from_quaternion(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
        
        return callback

    def manual_transform_velocity(self, global_lin_x, global_lin_y, global_ang_z, robot_name):
        """
        Transforme manuellement les vitesses globales en vitesses dans le repère du robot
        en utilisant les données de pose VRPN pour le robot spécifié.
        """
        try:
            # Vérifier si nous avons des données de pose pour ce robot
            if self.poses[robot_name] is None:
                return global_lin_x, global_lin_y, global_ang_z
            
            # Calculer le temps écoulé depuis la dernière mise à jour de la pose
            time_diff = (self.get_clock().now() - self.last_pose_times[robot_name]).nanoseconds / 1e9
            if time_diff > 1.0:  # Si la dernière pose date de plus d'une seconde
                self.get_logger().warning_throttle(5, f'{robot_name}: Les données de pose sont obsolètes ({time_diff:.2f}s). Utilisation des vitesses globales.')
                return global_lin_x, global_lin_y, global_ang_z
            
            # Utiliser le yaw de la pose VRPN pour la transformation
            yaw = self.yaws[robot_name]
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            # Transformer du repère global au repère du robot
            robot_lin_x = global_lin_x * cos_yaw + global_lin_y * sin_yaw
            robot_lin_y = -global_lin_x * sin_yaw + global_lin_y * cos_yaw
            
            # La vitesse angulaire reste inchangée
            robot_ang_z = global_ang_z
            
            return robot_lin_x, robot_lin_y, robot_ang_z
                
        except Exception as e:
            self.get_logger().error(f'{robot_name}: Manual transformation error: {e}')
            return global_lin_x, global_lin_y, global_ang_z

    def transform_velocity(self, global_lin_x, global_lin_y, global_ang_z, robot_name):
        """
        Transforme les vitesses du repère global au repère du robot spécifié.
        """
        try:
            # les entrées sont des floats
            global_lin_x = float(global_lin_x)
            global_lin_y = float(global_lin_y)
            global_ang_z = float(global_ang_z)
            
            # Création d'un vecteur pour représenter la vitesse globale
            global_vel = Vector3()
            global_vel.x = global_lin_x
            global_vel.y = global_lin_y
            global_vel.z = 0.0  # Pas de mouvement en Z
            
            # TF2 pour transformer les vitesses
            try:
                # Récupérer la transformation entre les frames
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame_ids[robot_name],  # Frame cible (robot)
                    self.global_frame_id,  # Frame source (globale)
                    now,
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                
                # Si TF2 fonctionne, loguer une confirmation
                if not self.tf2_working[robot_name]:
                    self.tf2_working[robot_name] = True
                    self.get_logger().info(f'{robot_name}: Transformation TF2 réussie entre {self.global_frame_id} et {self.robot_frame_ids[robot_name]}')
                
                # Appliquer la transformation au vecteur de vitesse
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                
                # Retourner les vitesses transformées
                return robot_vel.x, robot_vel.y, global_ang_z
                
            except TransformException as ex:
                # Si TF2 échoue, loguer une erreur et passer au fallback manuel
                if self.tf2_working[robot_name]:
                    self.tf2_working[robot_name] = False
                    self.get_logger().error(f'{robot_name}: Échec de la transformation TF2 : {ex}')
                    self.get_logger().info(f'{robot_name}: Utilisation de la transformation manuelle basée sur les données VRPN')
                
                # Fallback manuel
                return self.manual_transform_velocity(global_lin_x, global_lin_y, global_ang_z, robot_name)
            
        except Exception as e:
            # Loguer toute autre erreur
            self.get_logger().error(f'{robot_name}: Erreur dans transform_velocity : {e}')
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

                if key == '\x03':  # CTRL+C pour quitter
                    break

                if key in key_mapping:
                    global_dx, global_dy, _, global_dth = key_mapping[key]
                    
                    self.get_logger().info(
                        f'Commande globale: linear.x={global_dx*0.5}, linear.y={global_dy*0.5}, angular.z={global_dth*1.0}'
                    )
                    
                    # Dispatch des commandes aux threads individuels des robots
                    for robot_name in ROBOT_NAMES:
                        self.robot_threads[robot_name].send_command(global_dx, global_dy, global_dth)
                        
                elif key:  # Si une touche non définie est pressée
                    # Envoyer une commande d'arrêt à tous les robots
                    for robot_name in ROBOT_NAMES:
                        self.get_logger().info(f'{robot_name}: Arrêt')
                        self.robot_threads[robot_name].send_command(0, 0, 0)
                        
                # Si aucune touche n'est pressée, ne rien envoyer
        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')
        finally:
            # Envoyer une commande d'arrêt à tous les robots avant de quitter
            for robot_name in ROBOT_NAMES:
                self.robot_threads[robot_name].send_command(0, 0, 0)
                time.sleep(0.1)  # Laisser un peu de temps pour que la commande soit traitée
                self.robot_threads[robot_name].stop()
                
            # Attendre que tous les threads s'arrêtent
            for robot_name in ROBOT_NAMES:
                self.robot_threads[robot_name].join(timeout=1.0)
                
            self.running = False
            spin_thread.join()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VRPNTeleop()
        node.run()
    except Exception as e:
        print(f"Erreur lors de l'exécution: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
