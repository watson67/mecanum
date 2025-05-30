import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import csv
from datetime import datetime
import os
import sys

from mecanum_swarm.config import ALL_ROBOT_NAMES

GLOBAL_FRAME = "mocap"

class CmdVelRateLogger(Node):
    def __init__(self, mode='classic'):
        super().__init__('cmd_vel_rate_logger')
        self.csv_dir = os.path.expanduser(f'~/mecanum/csv/{mode}')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_paths = {name: os.path.join(self.csv_dir, f"{name}_cmd_vel_rate.csv") for name in ALL_ROBOT_NAMES}
        self.subs = []
        self.active = True
        
        # Initialize tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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

        self.create_subscription(
            Int32,
            "/master",
            self.master_callback,
            10
        )

    def _init_csvs(self):
        # Un fichier CSV par robot, avec colonnes pour timestamp, position et commandes
        for name in ALL_ROBOT_NAMES:
            try:
                with open(self.csv_paths[name], 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['timestamp', 'x', 'y', 'linear_x', 'linear_y', 'angular_z'])
                self.get_logger().info(f"Fichier CSV initialisé : {self.csv_paths[name]}")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la création du CSV pour {name} : {e}")

    def master_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("Activation reçue sur /master, réinitialisation des fichiers CSV.")
            self._init_csvs()
            self.active = True
        elif msg.data == 0:
            self.get_logger().info("Désactivation reçue sur /master, arrêt de l'écriture dans les CSV.")
            self.active = False

    def _make_callback(self, robot_name):
        def callback(msg):
            if not self.active:
                return
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
            
            try:
                with open(self.csv_paths[robot_name], 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([now, x, y, linear_x, linear_y, angular_z])
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
