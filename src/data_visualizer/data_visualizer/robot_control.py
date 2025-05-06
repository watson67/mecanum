#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Bool
import math
from transforms3d.euler import quat2euler, euler2quat
import numpy as np

class RobotController(Node):
    """
    Controls robot movement by sending velocity commands
    Uses a simple proportional controller to move to a target position
    """
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_controller')
        
        # Robot name
        self.robot_name = robot_name
        
        # Current position and orientation
        self.current_position = None
        self.current_yaw = None
        
        # Target position and status
        self.target_position = None
        self.target_yaw = 0.0
        self.is_moving = False
        self.arrived = False
        
        # Controller parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.05  # meters
        self.angle_tolerance = 0.1  # radians
        
        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f"/vrpn_mocap/{robot_name}/pose",
            self.pose_callback,
            10)
        
        # Subscribe to target pose topic
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            f"/{robot_name}/target_pose",
            self.target_pose_callback,
            10)
            
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f"/{robot_name}/cmd_vel",
            10)
            
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'{robot_name} controller initialized')
    
    def pose_callback(self, msg):
        """Update current position and orientation"""
        self.current_position = (msg.pose.position.x, msg.pose.position.y)
        
        # Extract yaw from quaternion
        q = [msg.pose.orientation.x, msg.pose.orientation.y, 
             msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, self.current_yaw = quat2euler(q)
    
    def target_pose_callback(self, msg):
        """Process new target pose"""
        # Extract position
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        
        # Extract yaw from quaternion
        q = [msg.pose.orientation.x, msg.pose.orientation.y, 
             msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, target_yaw = quat2euler(q)
        
        # Set new target
        self.target_position = (target_x, target_y)
        self.target_yaw = target_yaw
        self.is_moving = True
        self.arrived = False
        
        self.get_logger().info(f'New target set: ({target_x:.2f}, {target_y:.2f}, {math.degrees(target_yaw):.2f}°)')
    
    def set_target(self, x, y, yaw=0.0):
        """Set a new target position directly"""
        self.target_position = (x, y)
        self.target_yaw = yaw
        self.is_moving = True
        self.arrived = False
        self.get_logger().info(f'New target set: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.2f}°)')
    
    def control_loop(self):
        """Main control loop for robot movement"""
        if not self.is_moving or not self.target_position or not self.current_position:
            return
        
        # Calculate distance and angle to target
        x, y = self.current_position
        target_x, target_y = self.target_position
        
        dx = target_x - x
        dy = target_y - y
        
        # Distance to target
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.position_tolerance:
            # We've arrived at the target position, now adjust orientation if needed
            angle_diff = self.target_yaw - self.current_yaw
            while angle_diff > math.pi:
                angle_diff -= 2*math.pi
            while angle_diff < -math.pi:
                angle_diff += 2*math.pi
                
            if abs(angle_diff) < self.angle_tolerance:
                # We've arrived at the target position and orientation
                self.stop_robot()
                self.is_moving = False
                self.arrived = True
                self.get_logger().info(f'Arrived at target position and orientation')
                return
            else:
                # Just rotate to target orientation
                cmd = Twist()
                cmd.angular.z = self.angular_speed * np.sign(angle_diff)
                self.cmd_vel_pub.publish(cmd)
                return
        
        # Calculate target heading
        target_heading = math.atan2(dy, dx)
        
        # Calculate smallest angle difference
        angle_diff = target_heading - self.current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        while angle_diff < -math.pi:
            angle_diff += 2*math.pi
        
        # Create velocity command
        cmd = Twist()
        
        # If we're not facing the right direction, rotate first
        if abs(angle_diff) > self.angle_tolerance:
            # Rotate to face target
            cmd.angular.z = self.angular_speed * np.sign(angle_diff)
        else:
            # Move forward and make small adjustments to stay on course
            cmd.linear.x = min(self.linear_speed, distance)
            cmd.angular.z = self.angular_speed * angle_diff / math.pi  # Proportional control
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Send zero velocity command to stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Stopping robot')


def main(args=None):
    rclpy.init(args=args)
    
    # Get robot name from command line arguments
    import sys
    robot_name = "Aramis"  # Default
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    
    # Create and run controller
    controller = RobotController(robot_name)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
