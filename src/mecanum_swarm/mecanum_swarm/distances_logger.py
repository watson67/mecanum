import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import csv
from datetime import datetime
import os
import math
import sys
from mecanum_swarm.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

GLOBAL_FRAME = "mocap"

class DistancesLogger(Node):
    def __init__(self, mode='classic'):
        super().__init__('distances_logger')
        self.declare_parameter('csv_filename', '')
        csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        if not csv_filename:
            csv_filename = 'distances_logger.csv'
        self.csv_dir = os.path.expanduser(f'~/mecanum/csv/{mode}')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_path = os.path.join(self.csv_dir, csv_filename)
        self.pairs = self._generate_neighbor_pairs()
        self._init_csv()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.create_subscription(
            Int32,
            "/master",
            self.master_callback,
            10
        )
        self.active = True

    def _generate_neighbor_pairs(self):
        """Generate unique pairs from robot neighbor relationships"""
        pairs = set()
        for robot, neighbors in ROBOT_NEIGHBORS.items():
            for neighbor in neighbors:
                # Create sorted tuple to avoid duplicates (A,B) and (B,A)
                pair = tuple(sorted([robot, neighbor]))
                pairs.add(pair)
        return list(pairs)

    def _init_csv(self):
        header = ['timestamp']
        for a, b in self.pairs:
            header.append(f"{a}_{b}_distance")
        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(header)
            self.get_logger().info(f"Fichier CSV initialisé : {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la création du CSV : {e}")

    def master_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("Activation reçue sur /master, réinitialisation du fichier CSV.")
            self._init_csv()
            self.active = True
        elif msg.data == 0:
            self.get_logger().info("Désactivation reçue sur /master, arrêt de l'écriture dans le CSV.")
            self.active = False

    def timer_callback(self):
        if not self.active:
            return
        positions = {}
        try:
            for name in ALL_ROBOT_NAMES:
                trans = self.tf_buffer.lookup_transform(
                    GLOBAL_FRAME,
                    f"{name}/base_link",
                    rclpy.time.Time()
                )
                pos = trans.transform.translation
                positions[name] = (pos.x, pos.y, pos.z)
        except Exception:
            # Si une position manque, on saute cet enregistrement
            return

        now = datetime.now().isoformat()
        row = [now]
        for a, b in self.pairs:
            ax, ay, az = positions[a]
            bx, by, bz = positions[b]
            dist = math.sqrt((ax-bx)**2 + (ay-by)**2 )
            row.append(dist)
        try:
            with open(self.csv_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'écriture dans le CSV : {e}")

def main(args=None):
    rclpy.init(args=args)
    mode = 'classic'
    import sys
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    node = DistancesLogger(mode)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
