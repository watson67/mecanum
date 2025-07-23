#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import psutil

class CpuRamUsagePublisher(Node):
    """
    Noeud ROS2 qui publie le pourcentage d'utilisation CPU et RAM sur /{robot_name}/cpu_usage et /{robot_name}/ram_usage.
    """
    def __init__(self):
        super().__init__('cpu_ram_usage_publisher')
        # Déterminer le nom du robot à partir du hostname
        hostname = socket.gethostname().lower()
        if hostname.endswith('-desktop'):
            hostname = hostname[:-8]
        self.robot_name = hostname.capitalize()
        self.cpu_publisher = self.create_publisher(
            Float32, f"/{self.robot_name}/cpu_usage", 10
        )
        self.ram_publisher = self.create_publisher(
            Float32, f"/{self.robot_name}/ram_usage", 10
        )
        # Timer pour publier toutes les 0.5 secondes
        self.create_timer(0.5, self.publish_usage)

    def publish_usage(self):
        # Mesure du % CPU global (tous coeurs)
        cpu_percent = psutil.cpu_percent(interval=None)
        # Mesure du % RAM utilisée
        ram_percent = psutil.virtual_memory().percent
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        ram_msg = Float32()
        ram_msg.data = float(ram_percent)
        self.cpu_publisher.publish(cpu_msg)
        self.ram_publisher.publish(ram_msg)
        self.get_logger().debug(f"CPU usage: {cpu_percent:.2f}% | RAM usage: {ram_percent:.2f}%")

def main(args=None):
    rclpy.init(args=args)
    node = CpuRamUsagePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
