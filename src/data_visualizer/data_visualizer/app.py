#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import threading
from .gui_manager import GUIManager
from .data_manager import DataManager
from .config_loader import ConfigLoader

class RobotVisualizerApp(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        
        # Load configuration
        self.config = ConfigLoader()
        self.config.load_robot_config()
        
        # Initialize data manager
        self.data_manager = DataManager(self.config)
        
        # Initialize GUI
        self.gui_manager = GUIManager(self.config, self.data_manager)
        
        # QoS profile for MOCAP
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Subscribe to pose topics
        for i, topic in enumerate(self.config.pose_topics):
            self.create_subscription(
                PoseStamped,
                topic,
                self.pose_callback_factory(self.config.robot_names[i]),
                qos_profile=self.qos_profile
            )
        
        # Subscribe to cmd_vel topics
        for i, topic in enumerate(self.config.cmd_vel_topics):
            self.create_subscription(
                Twist,
                topic,
                self.cmd_vel_callback_factory(self.config.robot_names[i]),
                10
            )
        
        # Subscribe to estimated position topics
        for robot in self.config.robot_names:
            if robot in self.config.estimated_position_topics:
                for neighbor, topic in self.config.estimated_position_topics[robot].items():
                    self.create_subscription(
                        Point,
                        topic,
                        self.estimated_position_callback_factory(robot, neighbor),
                        10
                    )
        
        # Subscribe to goal_point topic
        self.create_subscription(
            Point,
            '/goal_point',
            self.goal_point_callback,
            10
        )
        
        # Subscribe to master topic for distance plot reset
        self.create_subscription(
            Int32,
            '/master',
            self.master_callback,
            10
        )
        
        # Timer for updating the plot
        self.timer = self.create_timer(0.1, self.gui_manager.update_plot)
        
        self.get_logger().info('Robot visualizer started')
    
    def pose_callback_factory(self, name):
        def callback(msg):
            self.data_manager.update_pose(name, msg)
            self.gui_manager.mark_text_update_needed()
        return callback
    
    def cmd_vel_callback_factory(self, name):
        def callback(msg):
            self.data_manager.update_velocity(name, msg)
        return callback
    
    def estimated_position_callback_factory(self, robot, neighbor):
        """Factory for estimated position callbacks"""
        def callback(msg):
            self.data_manager.update_estimated_position(robot, neighbor, msg)
        return callback
    
    def goal_point_callback(self, msg):
        """Callback for goal point messages"""
        self.data_manager.update_goal_point(msg)
        self.get_logger().info(f'Goal point updated: ({msg.x:.2f}, {msg.y:.2f})')
    
    def master_callback(self, msg):
        """Callback for master topic - reset distance plot when 1 is received"""
        if msg.data == 1:
            self.data_manager.reset_distance_history()
            self.get_logger().info('Distance plot reset triggered by master topic')
    
    def start(self):
        """Start the application"""
        # Create a thread for ROS spinning
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start the GUI
        self.gui_manager.start()
    
    def ros_spin(self):
        """Spin ROS in a separate thread"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    app = RobotVisualizerApp()
    
    try:
        app.start()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        app.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
