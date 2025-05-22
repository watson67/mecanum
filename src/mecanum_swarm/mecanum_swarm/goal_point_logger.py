import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import csv
from datetime import datetime
import os
import sys

class GoalPointLogger(Node):
    def __init__(self, mode='classic'):
        super().__init__('goal_point_logger')
        self.declare_parameter('csv_filename', '')
        csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        if not csv_filename:
            csv_filename = 'goal_point_logger.csv'
        self.csv_dir = os.path.expanduser(f'~/mecanum/csv/{mode}')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_path = os.path.join(self.csv_dir, csv_filename)
        self._init_csv()
        self.active = True
        self.create_subscription(Point, '/goal_point', self.goal_point_callback, 10)
        self.create_subscription(Int32, "/master", self.master_callback, 10)

    def _init_csv(self):
        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'x', 'y', 'z'])
            self.get_logger().info(f"Fichier CSV goal_point_logger initialisé : {self.csv_path}")
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

    def goal_point_callback(self, msg):
        if not self.active:
            return
        now = datetime.now().isoformat()
        try:
            with open(self.csv_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([now, msg.x, msg.y, msg.z])
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'écriture dans le CSV : {e}")

def main(args=None):
    rclpy.init(args=args)
    mode = 'classic'
    import sys
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    node = GoalPointLogger(mode)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
