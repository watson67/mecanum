import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TurtlesimPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np

class PoseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter')

        # Noms des robots dans l'essaim
        self.ROBOT_NAMES = ["Athos", "Porthos", "Aramis"]
        
        # Dictionary to store publishers for each robot
        self.pose_publishers = {}
        
        # Create subscribers and publishers dynamically
        for name in self.ROBOT_NAMES:
            # Create subscriber for turtle pose
            self.create_subscription(
                TurtlesimPose,
                f'/{name}/pose',
                self.make_pose_callback(name),
                10
            )
            
            # Create publisher for converted pose
            publisher = self.create_publisher(
                PoseStamped,
                f'/vrpn_mocap/{name}/pose',
                10
            )
            self.pose_publishers[name] = publisher
            
            self.get_logger().info(f"Created subscription and publisher for {name}")
        
        self.get_logger().info("Pose converter initialized - Converting to PoseStamped")
    
    def make_pose_callback(self, robot_name):
        """Factory method that creates callback functions for each robot"""
        def pose_callback(msg):
            converted_msg = self.convert_pose(msg)
            self.pose_publishers[robot_name].publish(converted_msg)
            # Optional debug logging
            # self.get_logger().debug(f"Published converted pose for {robot_name}")
        return pose_callback

    def convert_pose(self, turtlesim_pose):
        # Convert turtlesim/msg/Pose to geometry_msgs/msg/PoseStamped
        pose_stamped = PoseStamped()
        
        # Add header with timestamp
        pose_stamped.header = Header()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "world"
        
        # Set position and orientation
        pose_stamped.pose.position.x = turtlesim_pose.x
        pose_stamped.pose.position.y = turtlesim_pose.y
        pose_stamped.pose.position.z = 0.0  # Turtlesim is 2D, so z is 0
        pose_stamped.pose.orientation.z = np.sin(turtlesim_pose.theta / 2.0)  # Correct quaternion conversion
        pose_stamped.pose.orientation.w = np.cos(turtlesim_pose.theta / 2.0)
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        
        return pose_stamped

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