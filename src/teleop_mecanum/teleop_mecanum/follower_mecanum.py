#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Follower(Node):
    def __init__(self):
        super().__init__('follower_node')

        # QoS compatible avec VRPN
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Paramètres PID
        self.Kp = 1
        self.Ki = 0.02
        self.Kd = 0.3
        self.dt = 0.1
        self.target_dist = 0.5

        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.pose_aramis = None
        self.pose_athos = None

        # Subscriptions
        self.sub_aramis = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/Aramis/pose',
            self.aramis_callback,
            qos_profile
        )

        self.sub_athos = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/Athos/pose',
            self.athos_callback,
            qos_profile
        )

        # Publisher
        self.publisher_cmd = self.create_publisher(
            Twist,
            '/Athos/cmd_vel',
            10
        )

        # Timer
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def aramis_callback(self, msg):
        self.pose_aramis = msg

    def athos_callback(self, msg):
        self.pose_athos = msg

    def timer_callback(self):
        if self.pose_aramis is None or self.pose_athos is None:
            return

        x1 = self.pose_aramis.pose.position.x
        y1 = self.pose_aramis.pose.position.y
        x2 = self.pose_athos.pose.position.x
        y2 = self.pose_athos.pose.position.y

        dx = x1 - x2
        dy = y1 - y2
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return

        dir_x = dx / dist
        dir_y = dy / dist
        error = dist - self.target_dist
        error_x = dir_x * error
        error_y = dir_y * error

        self.integral_x += error_x * self.dt
        self.integral_y += error_y * self.dt
        derivative_x = (error_x - self.prev_error_x) / self.dt
        derivative_y = (error_y - self.prev_error_y) / self.dt

        vx = self.Kp * error_x + self.Ki * self.integral_x + self.Kd * derivative_x
        vy = self.Kp * error_y + self.Ki * self.integral_y + self.Kd * derivative_y

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # Clamp
        max_speed = 0.5
        vx = max(min(vx, max_speed), -max_speed)
        vy = max(min(vy, max_speed), -max_speed)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = 0.0
        self.publisher_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
