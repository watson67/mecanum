import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
import socket
from mecanum_swarm.config import ALL_ROBOT_NAMES

GLOBAL_FRAME = "mocap"

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

class DistributedTF2Manager(Node):
    def __init__(self):
        # Déterminer le nom du robot à partir du hostname
        hostname = socket.gethostname().lower()
        # Supprimer le suffixe '-desktop' si présent
        if hostname.endswith('-desktop'):
            hostname = hostname[:-8]
        self.robot_name = hostname.capitalize()  # Première lettre en majuscule
        
        # Vérifier si le nom est dans la liste des robots connus
        if self.robot_name not in ALL_ROBOT_NAMES:
            print(f"Warning: Robot name '{self.robot_name}' not in known robot list {ALL_ROBOT_NAMES}")
            self.robot_name = "Unknown"  # Fallback au cas où
        
        # Pas de namespace pour le TF2 manager - il doit publier globalement
        super().__init__(f'distributed_tf2_manager_{self.robot_name.lower()}')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Souscrire uniquement au topic VRPN de ce robot
        topic = f"/vrpn_mocap/{self.robot_name}/pose"
        self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            qos_profile
        )
        
        self.get_logger().info(f"Distributed TF2 Manager started for robot: {self.robot_name}")
        self.get_logger().info(f"Subscribed to: {topic}")
        self.get_logger().info(f"Publishing TF2 frame: {self.robot_name}/base_link")

    def pose_callback(self, msg):
        """Callback pour la position de ce robot"""
        # Créer et publier la transformation TF2 pour ce robot
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = GLOBAL_FRAME
        transform.child_frame_id = f"{self.robot_name}/base_link"
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation.x = msg.pose.orientation.x
        transform.transform.rotation.y = msg.pose.orientation.y
        transform.transform.rotation.z = msg.pose.orientation.z
        transform.transform.rotation.w = msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedTF2Manager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
