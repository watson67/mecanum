import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import csv
from datetime import datetime
import os
import sys

from mecanum_swarm.config import ALL_ROBOT_NAMES

class CmdVelRateLogger(Node):
    def __init__(self, mode='classic'):
        super().__init__('cmd_vel_rate_logger')
        self.csv_dir = os.path.expanduser(f'~/mecanum/csv/{mode}')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_paths = {name: os.path.join(self.csv_dir, f"{name}_cmd_vel_rate.csv") for name in ALL_ROBOT_NAMES}
        self.subs = []
        self.active = True
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
        # Un fichier CSV par robot, avec une seule colonne "timestamp"
        for name in ALL_ROBOT_NAMES:
            try:
                with open(self.csv_paths[name], 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['timestamp'])
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
            try:
                with open(self.csv_paths[robot_name], 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([now])
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
