import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import csv
from datetime import datetime
import os

class GoalPointLogger(Node):
    def __init__(self):
        super().__init__('goal_point_logger')
        self.csv_dir = os.path.expanduser('~/mecanum/csv')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_path = os.path.join(self.csv_dir, 'goal_point_logger.csv')
        self._init_csv()
        self.create_subscription(Point, '/goal_point', self.goal_point_callback, 10)

    def _init_csv(self):
        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'x', 'y', 'z'])
            self.get_logger().info(f"Fichier CSV goal_point_logger initialisé : {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la création du CSV : {e}")

    def goal_point_callback(self, msg):
        now = datetime.now().isoformat()
        try:
            with open(self.csv_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([now, msg.x, msg.y, msg.z])
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'écriture dans le CSV : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPointLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
