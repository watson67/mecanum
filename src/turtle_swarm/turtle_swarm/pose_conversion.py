import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TurtlesimPose
from geometry_msgs.msg import Pose as GeometryPose

class PoseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter')

        # Subscribers
        self.athos_pose_sub = self.create_subscription(
            TurtlesimPose,
            '/athos/pose',
            self.athos_pose_callback,
            10
        )
        self.orthos_pose_sub = self.create_subscription(
            TurtlesimPose,
            '/orthos/pose',
            self.orthos_pose_callback,
            10
        )

        # Publishers
        self.athos_pose_pub = self.create_publisher(
            GeometryPose,
            '/athos/pose_converted',
            10
        )
        self.orthos_pose_pub = self.create_publisher(
            GeometryPose,
            '/orthos/pose_converted',
            10
        )

    def athos_pose_callback(self, msg):
        converted_msg = self.convert_pose(msg)
        self.athos_pose_pub.publish(converted_msg)

    def orthos_pose_callback(self, msg):
        converted_msg = self.convert_pose(msg)
        self.orthos_pose_pub.publish(converted_msg)

    def convert_pose(self, turtlesim_pose):
        # Convert turtlesim/msg/Pose to geometry_msgs/msg/Pose
        geometry_pose = GeometryPose()
        geometry_pose.position.x = turtlesim_pose.x
        geometry_pose.position.y = turtlesim_pose.y
        geometry_pose.position.z = 0.0  # Turtlesim is 2D, so z is 0
        geometry_pose.orientation.z = turtlesim_pose.theta
        geometry_pose.orientation.x = 0.0
        geometry_pose.orientation.y = 0.0
        geometry_pose.orientation.w = 1.0  # Assuming no roll/pitch
        return geometry_pose

def main(args=None):
    rclpy.init(args=args)
    node = PoseConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()