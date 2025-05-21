import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import csv
from datetime import datetime
import os

ALL_ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]  # Liste de tous les robots possibles

class CmdVelRateLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_rate_logger')
        self.declare_parameter('csv_filename', '')
        csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        if not csv_filename:
            csv_filename = 'cmd_vel_rate.csv'
        self.csv_dir = os.path.expanduser('~/mecanum/csv')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_path = os.path.join(self.csv_dir, csv_filename)
        self.cmd_counts = {name: 0 for name in ALL_ROBOT_NAMES}
        self.subs = []
        self._init_csv()

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

    def _init_csv(self):
        # Entête : une colonne par robot
        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(ALL_ROBOT_NAMES)
            self.get_logger().info(f"Fichier CSV initialisé : {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la création du CSV : {e}")

    def master_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("Activation reçue sur /master, réinitialisation du fichier CSV.")
            self.cmd_counts = {name: 0 for name in ALL_ROBOT_NAMES}
            self._init_csv()

    def _make_callback(self, robot_name):
        def callback(msg):
            now = datetime.now().isoformat()
            # Une ligne avec l'heure dans la colonne du robot, vide ailleurs
            row = ['' for _ in ALL_ROBOT_NAMES]
            idx = ALL_ROBOT_NAMES.index(robot_name)
            row[idx] = now
            try:
                with open(self.csv_path, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(row)
            except Exception as e:
                self.get_logger().error(f"Erreur lors de l'écriture dans le CSV : {e}")
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRateLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
