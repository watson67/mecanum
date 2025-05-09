import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]
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

        for name in ROBOT_NAMES:
            topic = f"/vrpn_mocap/{name}/pose"
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, robot=name: self.pose_callback(msg, robot),
                qos_profile
            )
            self.pose_data[name] = None

        self.get_logger().info("TF2 Manager prêt. Souscrit aux topics VRPN et publie les transformations TF2.")

    def pose_callback(self, msg, robot_name):
        self.pose_data[robot_name] = msg

        transform = TransformStamped()
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
