import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os
import csv
from datetime import datetime
import glob
import sys

from swarm_manager.config import ALL_ROBOT_NAMES

class CpuRamLogger(Node):
    def __init__(self, mode='classic'):
        super().__init__('logger/cpu_ram_logger')
        # Dossier pour les CSV (avec mode)
        self.csv_dir = os.path.expanduser(f'~/mecanum/csv/{mode}/cpu_ram')
        os.makedirs(self.csv_dir, exist_ok=True)
        self._cleanup_old_csvs()

        # Chemins des fichiers CSV pour chaque robot
        self.cpu_csv_paths = {name: os.path.join(self.csv_dir, f"{name}_cpu_usage.csv") for name in ALL_ROBOT_NAMES}
        self.ram_csv_paths = {name: os.path.join(self.csv_dir, f"{name}_ram_usage.csv") for name in ALL_ROBOT_NAMES}
        self.ram_used_csv_paths = {name: os.path.join(self.csv_dir, f"{name}_ram_used.csv") for name in ALL_ROBOT_NAMES}

        # Initialiser les CSV
        self._init_csvs()

        # Souscriptions pour chaque robot
        self.subs = []
        for name in ALL_ROBOT_NAMES:
            cpu_topic = f"/{name}/cpu_usage"
            ram_topic = f"/{name}/ram_usage"
            ram_used_topic = f"/{name}/ram_used"
            self.subs.append(
                self.create_subscription(Float32, cpu_topic, self._make_callback(name, "cpu"), 10)
            )
            self.subs.append(
                self.create_subscription(Float32, ram_topic, self._make_callback(name, "ram"), 10)
            )
            self.subs.append(
                self.create_subscription(Float32, ram_used_topic, self._make_callback(name, "ram_used"), 10)
            )

    def _cleanup_old_csvs(self):
        """Supprime tous les fichiers CSV existants dans le dossier cpu_ram."""
        for pattern in ["*_cpu_usage.csv", "*_ram_usage.csv", "*_ram_used.csv"]:
            for csv_file in glob.glob(os.path.join(self.csv_dir, pattern)):
                try:
                    os.remove(csv_file)
                    self.get_logger().info(f"Fichier CSV supprimé : {csv_file}")
                except Exception as e:
                    self.get_logger().error(f"Erreur lors de la suppression du fichier {csv_file} : {e}")

    def _init_csvs(self):
        """Crée les fichiers CSV avec en-tête timestamp, value."""
        for name in ALL_ROBOT_NAMES:
            try:
                with open(self.cpu_csv_paths[name], 'w', newline='') as cpu_csv:
                    writer = csv.writer(cpu_csv)
                    writer.writerow(['timestamp', 'cpu_percent'])
                with open(self.ram_csv_paths[name], 'w', newline='') as ram_csv:
                    writer = csv.writer(ram_csv)
                    writer.writerow(['timestamp', 'ram_percent'])
                with open(self.ram_used_csv_paths[name], 'w', newline='') as ram_used_csv:
                    writer = csv.writer(ram_used_csv)
                    writer.writerow(['timestamp', 'ram_used_mb'])
                self.get_logger().info(f"Fichiers CSV initialisés pour {name}")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la création des CSV pour {name} : {e}")

    def _make_callback(self, robot_name, usage_type):
        """
        usage_type: "cpu", "ram" ou "ram_used"
        """
        def callback(msg):
            now = datetime.now().isoformat()
            value = float(msg.data)
            if usage_type == "cpu":
                path = self.cpu_csv_paths[robot_name]
            elif usage_type == "ram":
                path = self.ram_csv_paths[robot_name]
            elif usage_type == "ram_used":
                path = self.ram_used_csv_paths[robot_name]
            else:
                return
            try:
                with open(path, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([now, value])
            except Exception as e:
                self.get_logger().error(f"Erreur lors de l'écriture dans {path} : {e}")
        return callback

def main(args=None):
    rclpy.init(args=args)
    # Récupération de l'argument mode (classic/event)
    mode = 'classic'
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    node = CpuRamLogger(mode)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
