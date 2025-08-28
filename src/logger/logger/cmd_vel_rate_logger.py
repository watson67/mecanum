import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Int32, Float64MultiArray
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import csv
from datetime import datetime
import os
import sys
import glob

from rclpy.qos import QoSProfile, ReliabilityPolicy
from swarm_manager.config import ALL_ROBOT_NAMES

GLOBAL_FRAME = "mocap"

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

comm_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1
)

class CmdVelRateLogger(Node):
    def __init__(self, mode='classic'):
        super().__init__('cmd_vel_rate_logger')
        self.csv_dir = os.path.expanduser(f'~/mecanum/csv/{mode}')
        os.makedirs(self.csv_dir, exist_ok=True)
        
        # Nettoyer les anciens fichiers CSV
        self._cleanup_old_csvs()
        
        self.csv_paths = {name: os.path.join(self.csv_dir, f"{name}_cmd_vel_rate.csv") for name in ALL_ROBOT_NAMES}
        self.subs = []
        self.active = True
        self.start_time = None
        self.cmd_vel_counts = {name: 0 for name in ALL_ROBOT_NAMES}
        self.position_counts = {name: 0 for name in ALL_ROBOT_NAMES}
        self.published_pose_counts = {name: 0 for name in ALL_ROBOT_NAMES}
        
        # Initialize tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Stockage des données de contrôle pour chaque robot
        self.control_data = {name: {'ui_alpha_x': 0.0, 'ui_alpha_y': 0.0, 'ui_gamma_x': 0.0, 'ui_gamma_y': 0.0} for name in ALL_ROBOT_NAMES}
        
        self._init_csvs()

        for name in ALL_ROBOT_NAMES:
            topic = f'/{name}/cmd_vel'
            sub = self.create_subscription(
                Twist,
                topic,
                self._make_callback(name),
                10
            )
            self.subs.append(sub)
            
            # Ajouter subscription pour les données de contrôle
            control_topic = f'/{name}/control_components'
            control_sub = self.create_subscription(
                Float64MultiArray,
                control_topic,
                self._make_control_callback(name),
                10
            )
            self.subs.append(control_sub)

            # Ajouter subscription pour les données de position
            pos_topic = f"/vrpn_mocap/{name}/pose"
            pos_sub = self.create_subscription(
                PoseStamped,
                pos_topic,
                self._make_position_callback(name),
                qos_profile
            )
            self.subs.append(pos_sub)

            # Ajouter subscription pour les published_pose
            pub_pos_topic = f"/{name}/published_pose"
            pub_pos_sub = self.create_subscription(
                Point,
                pub_pos_topic,
                self._make_published_pose_callback(name),
                comm_qos
            )
            self.subs.append(pub_pos_sub)

        self.create_subscription(
            Int32,
            "/master",
            self.master_callback,
            10
        )

    def _cleanup_old_csvs(self):
        """Supprime tous les fichiers CSV existants se terminant par cmd_vel_rate.csv"""
        pattern = os.path.join(self.csv_dir, "*cmd_vel_rate.csv")
        csv_files = glob.glob(pattern)
        for csv_file in csv_files:
            try:
                os.remove(csv_file)
                self.get_logger().info(f"Fichier CSV supprimé : {csv_file}")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la suppression du fichier {csv_file} : {e}")

        duration_file = os.path.join(self.csv_dir, "statistics.txt")
        if os.path.exists(duration_file):
            try:
                os.remove(duration_file)
                self.get_logger().info(f"Fichier de statistiques supprimé : {duration_file}")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la suppression du fichier de statistiques {duration_file} : {e}")

    def _init_csvs(self):
        # Un fichier CSV par robot, avec colonnes pour timestamp, position, commandes et composantes de contrôle
        for name in ALL_ROBOT_NAMES:
            try:
                with open(self.csv_paths[name], 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['timestamp', 'x', 'y', 'linear_x', 'linear_y', 'angular_z', 'ui_alpha_x', 'ui_alpha_y', 'ui_gamma_x', 'ui_gamma_y'])
                self.get_logger().info(f"Fichier CSV initialisé : {self.csv_paths[name]}")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la création du CSV pour {name} : {e}")

    def master_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("Activation reçue sur /master, réinitialisation des fichiers CSV.")
            self._init_csvs()
            self.active = True
            self.start_time = datetime.now()
            self.cmd_vel_counts = {name: 0 for name in ALL_ROBOT_NAMES}
            self.position_counts = {name: 0 for name in ALL_ROBOT_NAMES}
            self.published_pose_counts = {name: 0 for name in ALL_ROBOT_NAMES}
        elif msg.data == 0:
            self.get_logger().info("Désactivation reçue sur /master, arrêt de l'écriture dans les CSV.")
            self.active = False
            if self.start_time:
                end_time = datetime.now()
                duration = end_time - self.start_time
                duration_path = os.path.join(self.csv_dir, 'statistics.txt')
                total_cmd_vel = sum(self.cmd_vel_counts.values())
                total_pos = sum(self.position_counts.values())
                total_pub_pos = sum(self.published_pose_counts.values())
                try:
                    with open(duration_path, 'w') as f:
                        f.write(f"Durée du parcours: {duration.total_seconds()} secondes\n\n")
                        f.write("Nombre de messages cmd_vel reçus:\n")
                        for name, count in self.cmd_vel_counts.items():
                            f.write(f"- {name}: {count}\n")
                        f.write(f"Total cmd_vel: {total_cmd_vel}\n\n")
                        f.write("Nombre de messages de position reçus:\n")
                        for name, count in self.position_counts.items():
                            f.write(f"- {name}: {count}\n")
                        f.write(f"Total positions: {total_pos}\n\n")
                        f.write("Nombre de messages de position publiée reçus:\n")
                        for name, count in self.published_pose_counts.items():
                            f.write(f"- {name}: {count}\n")
                        f.write(f"Total positions publiées: {total_pub_pos}\n")

                    self.get_logger().info(f"Statistiques enregistrées dans {duration_path}")
                except Exception as e:
                    self.get_logger().error(f"Erreur lors de l'écriture du fichier de statistiques : {e}")
                self.start_time = None

    def _make_control_callback(self, robot_name):
        def callback(msg):
            if not self.active:
                return
            # msg.data contient [ui_alpha_x, ui_alpha_y, ui_gamma_x, ui_gamma_y]
            if len(msg.data) >= 4:
                self.control_data[robot_name]['ui_alpha_x'] = msg.data[0]
                self.control_data[robot_name]['ui_alpha_y'] = msg.data[1]
                self.control_data[robot_name]['ui_gamma_x'] = msg.data[2]
                self.control_data[robot_name]['ui_gamma_y'] = msg.data[3]
        return callback

    def _make_position_callback(self, robot_name):
        def callback(msg):
            if not self.active:
                return
            self.position_counts[robot_name] += 1
        return callback

    def _make_published_pose_callback(self, robot_name):
        def callback(msg):
            if not self.active:
                return
            self.published_pose_counts[robot_name] += 1
        return callback

    def _make_callback(self, robot_name):
        def callback(msg):
            if not self.active:
                return
            
            self.cmd_vel_counts[robot_name] += 1
            now = datetime.now().isoformat()
            
            # Récupérer la position du robot via tf2
            try:
                trans = self.tf_buffer.lookup_transform(
                    GLOBAL_FRAME,
                    f"{robot_name}/base_link",
                    rclpy.time.Time()
                )
                x = trans.transform.translation.x
                y = trans.transform.translation.y
            except Exception as e:
                self.get_logger().warn(f"Impossible de récupérer la position de {robot_name}: {e}")
                x = y = 0.0
            
            # Extraire les données de commande
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z
            
            # Récupérer les données de contrôle
            control = self.control_data[robot_name]
            
            try:
                with open(self.csv_paths[robot_name], 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([now, x, y, linear_x, linear_y, angular_z, 
                                   control['ui_alpha_x'], control['ui_alpha_y'], 
                                   control['ui_gamma_x'], control['ui_gamma_y']])
            except Exception as e:
                self.get_logger().error(f"Erreur lors de l'écriture dans le CSV pour {robot_name} : {e}")
        return callback

def main(args=None):
    rclpy.init(args=args)
    # Récupération de l'argument mode (classic/event)
    mode = 'classic'
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    node = CmdVelRateLogger(mode)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
