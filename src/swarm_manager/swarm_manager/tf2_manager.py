import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
from swarm_manager.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

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

        # Publisher pour le barycentre au format VRPN (triche pour les robots)
        self.barycenter_vrpn_publisher = self.create_publisher(
            PoseStamped,
            "/vrpn_mocap/Barycenter/pose",
            qos_profile
        )

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

        self.get_logger().info("TF2 Manager prêt. Souscrit aux topics VRPN et publie les transformations TF2 + barycentre VRPN.")

        self.create_timer(0.05, self.publish_all_tf)  # 20 Hz

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

    def publish_all_tf(self):
        # Regroupe tous les TF2 pour chaque robot à 20 Hz dans un seul message
        transforms = []
        for name, pose_msg in self.pose_data.items():
            if pose_msg is not None:
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = GLOBAL_FRAME
                transform.child_frame_id = f"{name}/base_link"
                transform.transform.translation.x = pose_msg.pose.position.x
                transform.transform.translation.y = pose_msg.pose.position.y
                transform.transform.translation.z = pose_msg.pose.position.z
                transform.transform.rotation.x = pose_msg.pose.orientation.x
                transform.transform.rotation.y = pose_msg.pose.orientation.y
                transform.transform.rotation.z = pose_msg.pose.orientation.z
                transform.transform.rotation.w = pose_msg.pose.orientation.w
                transforms.append(transform)
        # Calcul et publication du barycentre
        if all(self.pose_data[name] is not None for name in ALL_ROBOT_NAMES):
            x = sum(self.pose_data[name].pose.position.x for name in ALL_ROBOT_NAMES) / len(ALL_ROBOT_NAMES)
            y = sum(self.pose_data[name].pose.position.y for name in ALL_ROBOT_NAMES) / len(ALL_ROBOT_NAMES)
            z = sum(self.pose_data[name].pose.position.z for name in ALL_ROBOT_NAMES) / len(ALL_ROBOT_NAMES)
            
            # Publication TF2 du barycentre (pour visualisation)
            bary_transform = TransformStamped()
            bary_transform.header.stamp = self.get_clock().now().to_msg()
            bary_transform.header.frame_id = GLOBAL_FRAME
            bary_transform.child_frame_id = "barycenter"
            bary_transform.transform.translation.x = x
            bary_transform.transform.translation.y = y
            bary_transform.transform.translation.z = z
            bary_transform.transform.rotation.x = self.barycenter_orientation[0]
            bary_transform.transform.rotation.y = self.barycenter_orientation[1]
            bary_transform.transform.rotation.z = self.barycenter_orientation[2]
            bary_transform.transform.rotation.w = self.barycenter_orientation[3]
            transforms.append(bary_transform)
            
            # Publication VRPN du barycentre (triche pour les robots)
            barycenter_pose_msg = PoseStamped()
            barycenter_pose_msg.header.stamp = self.get_clock().now().to_msg()
            barycenter_pose_msg.header.frame_id = GLOBAL_FRAME
            barycenter_pose_msg.pose.position.x = x
            barycenter_pose_msg.pose.position.y = y
            barycenter_pose_msg.pose.position.z = z
            barycenter_pose_msg.pose.orientation.x = self.barycenter_orientation[0]
            barycenter_pose_msg.pose.orientation.y = self.barycenter_orientation[1]
            barycenter_pose_msg.pose.orientation.z = self.barycenter_orientation[2]
            barycenter_pose_msg.pose.orientation.w = self.barycenter_orientation[3]
            
            self.barycenter_vrpn_publisher.publish(barycenter_pose_msg)
            self.get_logger().debug(f"Published barycenter VRPN: X:{x:.3f}, Y:{y:.3f}")
        
        # Publie tous les TF2 d'un coup
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

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
