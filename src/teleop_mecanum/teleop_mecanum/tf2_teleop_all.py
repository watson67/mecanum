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
import queue
import time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
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
        
        # Ajouter un timeout plus court pour les opérations TF
        self.tf_timeout = 0.05  # 50 ms instead of 100 ms
    
    def run(self):
        """Boucle principale du thread de commande du robot."""
        while self.running:
            try:
                # Attendre une commande avec timeout plus court
                try:
                    command = self.command_queue.get(timeout=0.05)  # Réduire à 50ms
                except queue.Empty:
                    continue
                
                global_dx, global_dy, global_dth = command
                
                # Transformer les vitesses du repère global au repère du robot
                # avec un timeout plus court
                robot_dx, robot_dy, robot_dth = self.node.transform_velocity(
                    global_dx, global_dy, global_dth, self.robot_name, self.tf_timeout
                )
                
                # Créer le message Twist avec les vitesses dans le repère du robot
                twist = Twist()
                twist.linear.x = robot_dx 
                twist.linear.y = robot_dy 
                twist.angular.z = robot_dth * 1.0
                
                # Log pour ce robot spécifique (à commenter pour réduire la charge)
                # self.node.get_logger().info(
                #     f'{self.robot_name}: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}'
                # )
                
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
        
        # Ajout d'un broadcaster TF pour publier les transformations
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Configuration des frames
        self.global_frame_id = "mocap"
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

        # Ajout d'un contrôle de rate pour les publications TF
        self.tf_publish_rate = 30.0  # Hz
        self.last_tf_publish = {name: self.get_clock().now() for name in ROBOT_NAMES}
        
        # Cache pour les transformations
        self.last_transform = {name: None for name in ROBOT_NAMES}

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
            
            # Limiter la fréquence de publication des transformations
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_tf_publish[robot_name]).nanoseconds / 1e9
            
            if time_diff >= (1.0 / self.tf_publish_rate):
                # Créer et publier la transformation TF
                transform = TransformStamped()
                transform.header.stamp = current_time.to_msg()
                transform.header.frame_id = self.global_frame_id
                transform.child_frame_id = self.robot_frame_ids[robot_name]
                
                # Copier les données de position
                transform.transform.translation.x = msg.pose.position.x
                transform.transform.translation.y = msg.pose.position.y
                transform.transform.translation.z = msg.pose.position.z
                
                # Copier les données d'orientation (quaternion)
                transform.transform.rotation.x = msg.pose.orientation.x
                transform.transform.rotation.y = msg.pose.orientation.y
                transform.transform.rotation.z = msg.pose.orientation.z
                transform.transform.rotation.w = msg.pose.orientation.w
                
                # Stocker la transformation et la publier
                self.last_transform[robot_name] = transform
                self.tf_broadcaster.sendTransform(transform)
                
                # Mettre à jour le temps de dernière publication
                self.last_tf_publish[robot_name] = current_time
        
        return callback

    def transform_velocity(self, global_lin_x, global_lin_y, global_ang_z, robot_name, timeout=0.1):
        """
        Transforme les vitesses du repère global au repère du robot spécifié en utilisant TF2.
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
            
            # TF2 pour transformer les vitesses
            try:
                # Récupérer la transformation entre les frames avec timeout spécifié
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame_ids[robot_name],  # Frame cible (robot)
                    self.global_frame_id,  # Frame source (globale)
                    now,
                    timeout=rclpy.duration.Duration(seconds=timeout)
                )
                
                # Si TF2 fonctionne, loguer une confirmation (uniquement la première fois)
                if not self.tf2_working[robot_name]:
                    self.tf2_working[robot_name] = True
                    self.get_logger().info(f'{robot_name}: Transformation TF2 réussie entre {self.global_frame_id} et {self.robot_frame_ids[robot_name]}')
                
                # Appliquer la transformation au vecteur de vitesse
                robot_vel = tf2_geometry_msgs.do_transform_vector3(global_vel, transform)
                
                # Retourner les vitesses transformées
                return robot_vel.vector.x, robot_vel.vector.y, global_ang_z
                
            except TransformException as ex:
                # Si TF2 échoue, loguer une erreur détaillée (mais pas trop souvent)
                if self.tf2_working[robot_name]:
                    self.tf2_working[robot_name] = False
                    self.get_logger().error(f'{robot_name}: Échec de la transformation TF2 : {ex}')
                
                # Si pas de transformation disponible, retourner les vitesses globales
                return global_lin_x, global_lin_y, global_ang_z
            
        except Exception as e:
            # Loguer toute autre erreur (mais pas trop souvent)
            self.get_logger().error(f'{robot_name}: Erreur dans transform_velocity : {e}')
            return global_lin_x, global_lin_y, global_ang_z

    def spin_thread(self):
        """Thread pour exécuter rclpy.spin avec délai réduit."""
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.005)  # Réduire à 5ms

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
                    
                    # Log réduit pour moins encombrer la console
                    self.get_logger().debug(
                        f'Commande globale: linear.x={global_dx}, linear.y={global_dy}, angular.z={global_dth}'
                    )
                    
                    # Dispatch des commandes aux threads individuels des robots
                    # Traiter Porthos en premier pour réduire sa latence
                    robot_order = ["Porthos", "Aramis", "Athos"] if "Porthos" in ROBOT_NAMES else ROBOT_NAMES
                    for robot_name in robot_order:
                        if robot_name in self.robot_threads:
                            self.robot_threads[robot_name].send_command(global_dx, global_dy, global_dth)
                        
                elif key:  # Si une touche non définie est pressée
                    # Envoyer une commande d'arrêt à tous les robots
                    for robot_name in ROBOT_NAMES:
                        self.robot_threads[robot_name].send_command(0, 0, 0)
                        
                # Si aucune touche n'est pressée, ne rien envoyer
                # Ajouter un court délai pour réduire l'utilisation CPU
                time.sleep(0.005)
        
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
