import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import csv
from datetime import datetime
import os

BARYCENTER_FRAME = "barycenter"
GLOBAL_FRAME = "mocap"

class BarycenterLogger(Node):
    def __init__(self):
        super().__init__('barycenter_logger')
        self.declare_parameter('csv_filename', '')
        csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        if not csv_filename:
            csv_filename = 'barycenter_logger.csv'
        self.csv_dir = os.path.expanduser('~/mecanum/csv')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_path = os.path.join(self.csv_dir, csv_filename)
        self.trajectory_type = "unknown"
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
        self.create_subscription(
            String,
            "/trajectory_type",
            self.trajectory_type_callback,
            10
        )

    def trajectory_type_callback(self, msg):
        self.trajectory_type = msg.data

    def _init_csv(self):
        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Première ligne : type de trajectoire
                writer.writerow([f"trajectory_type: {self.trajectory_type}"])
                # Deuxième ligne : entêtes
                writer.writerow(['timestamp', 'x', 'y', 'z'])
            self.get_logger().info(f"Fichier CSV initialisé : {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la création du CSV : {e}")

    def master_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("Activation reçue sur /master, réinitialisation du fichier CSV.")
            self._init_csv()

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME,
                BARYCENTER_FRAME,
                rclpy.time.Time()
            )
            now = datetime.now().isoformat()
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            with open(self.csv_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([now, x, y, z])
        except Exception:
            # Pas de transform dispo, on ignore
            pass

def main(args=None):
    rclpy.init(args=args)
    node = BarycenterLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
