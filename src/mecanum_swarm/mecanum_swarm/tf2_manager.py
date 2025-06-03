import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
from mecanum_swarm.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

GLOBAL_FRAME = "mocap"

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

def euler_from_quaternion(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class TF2Manager(Node):
    def __init__(self):
        super().__init__('tf2_manager')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_data = {}
        self.barycenter_orientation = [0.0, 0.0, 0.0, 1.0]
        
        # Flag pour suivre l'état de l'obstacle
        self.obstacle_available = False
        
        # Add formation initializer reference (optional integration)
        self.formation_initializer = None

        for name in ALL_ROBOT_NAMES:
            topic = f"/vrpn_mocap/{name}/pose"
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, robot=name: self.pose_callback(msg, robot),
                qos_profile
            )
            self.pose_data[name] = None

        # Souscrire au topic de l'obstacle
        self.create_subscription(
            PoseStamped,
            "/vrpn_mocap/Obstacle/pose",
            self.obstacle_pose_callback,
            qos_profile
        )

        self.get_logger().info("TF2 Manager prêt. Souscrit aux topics VRPN et publie les transformations TF2.")

        self.create_timer(0.05, self.publish_barycenter_tf)  # 20 Hz

    def set_formation_initializer(self, formation_initializer):
        """Définir la référence à l'initialisateur de formation pour l'intégration"""
        self.formation_initializer = formation_initializer

    def get_robot_positions_dict(self):
        """Obtenir les positions actuelles des robots sous forme de dictionnaire"""
        positions = {}
        for name, pose_msg in self.pose_data.items():
            if pose_msg is not None:
                positions[name] = (pose_msg.pose.position.x, pose_msg.pose.position.y)
        return positions

    def pose_callback(self, msg, robot_name):
        self.pose_data[robot_name] = msg

        transform = TransformStamped()
        # Use the current time for the transform header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = GLOBAL_FRAME
        transform.child_frame_id = f"{robot_name}/base_link"
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation.x = msg.pose.orientation.x
        transform.transform.rotation.y = msg.pose.orientation.y
        transform.transform.rotation.z = msg.pose.orientation.z
        transform.transform.rotation.w = msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug(f"Published TF2: {GLOBAL_FRAME} -> {robot_name}/base_link at ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})")

    def publish_barycenter_tf(self):
        if not all(self.pose_data[name] is not None for name in ALL_ROBOT_NAMES):
            return

        x = sum(self.pose_data[name].pose.position.x for name in ALL_ROBOT_NAMES) / len(ALL_ROBOT_NAMES)
        y = sum(self.pose_data[name].pose.position.y for name in ALL_ROBOT_NAMES) / len(ALL_ROBOT_NAMES)
        z = sum(self.pose_data[name].pose.position.z for name in ALL_ROBOT_NAMES) / len(ALL_ROBOT_NAMES)

        transform = TransformStamped()
        # Use the current time for the transform header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = GLOBAL_FRAME
        transform.child_frame_id = "barycenter"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = self.barycenter_orientation[0]
        transform.transform.rotation.y = self.barycenter_orientation[1]
        transform.transform.rotation.z = self.barycenter_orientation[2]
        transform.transform.rotation.w = self.barycenter_orientation[3]

        self.tf_broadcaster.sendTransform(transform)

    def set_barycenter_orientation(self, x, y, z, w):
        self.barycenter_orientation = [x, y, z, w]

    def obstacle_pose_callback(self, msg):
        """Callback pour la position de l'obstacle"""
        if not self.obstacle_available:
            self.get_logger().info("Obstacle detected, publishing obstacle/base_link frame")
            self.obstacle_available = True

        # Créer et publier la transformation TF2 pour l'obstacle
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = GLOBAL_FRAME
        transform.child_frame_id = "Obstacle/base_link"
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation.x = msg.pose.orientation.x
        transform.transform.rotation.y = msg.pose.orientation.y
        transform.transform.rotation.z = msg.pose.orientation.z
        transform.transform.rotation.w = msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug(f"Published TF2: {GLOBAL_FRAME} -> obstacle/base_link at ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})")

def main(args=None):
    rclpy.init(args=args)
    node = TF2Manager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
