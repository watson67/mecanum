#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from inputs import get_gamepad
import sys
import threading
import time  # Add this import for sleep

class TeleopDualShock4(Node):
    def __init__(self, topic):
        super().__init__('teleop_dualshock4')

        # Parameters
        self.declare_parameter('max_speed', 1.0)  # Max linear speed
        self.declare_parameter('max_turn', 0.5)   # Max angular speed
        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn = self.get_parameter('max_turn').value

        self.pub = self.create_publisher(Twist, topic, 10)
        self.running = True  # Flag to control the input loop

        self.last_values = {
            'ABS_X': 128,  # Left stick X
            'ABS_Y': 128,  # Left stick Y
            'ABS_RX': 128, # Right stick X
            'ABS_RY': 128, # Right stick Y
            'ABS_Z': 0,    # Trigger L2
            'ABS_RZ': 0    # Trigger R2
        }
        self.deadzone = 0.2  # Deadzone to filter noise
        self.get_logger().info(f"Listening for DualShock 4 inputs on topic '{topic}'...")

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return value

    def run(self):
        try:
            while self.running:
                rclpy.spin_once(self, timeout_sec=0.01)  # Handle ROS events

                twist = Twist()
                try:
                    events = get_gamepad()  # Read gamepad events
                except Exception as e:
                    self.get_logger().warn(f"Error reading gamepad: {e}")
                    continue

                for event in events:
                    if event.ev_type == 'Absolute' and event.code in self.last_values:
                        self.last_values[event.code] = event.state
                    elif event.ev_type == 'Key':
                        action = "pressed" if event.state == 1 else "released"
                        self.get_logger().info(f"Button {event.code} {action}")
                        if event.code == 'BTN_EAST' and event.state == 1:  # Stop button
                            self.stop_robot()

                # Normalize stick values (0-255 -> -1 to 1)
                lx = self.apply_deadzone(-(self.last_values['ABS_X'] - 128) / 127.0, self.deadzone)  # Inverted X-axis
                ly = self.apply_deadzone(-(self.last_values['ABS_Y'] - 128) / 127.0, self.deadzone)
                rx = self.apply_deadzone((self.last_values['ABS_RX'] - 128) / 127.0, self.deadzone)
                l2 = self.last_values['ABS_Z'] / 255.0
                r2 = self.last_values['ABS_RZ'] / 255.0
                vz = r2 - l2

                # Apply values to the Twist message
                twist.linear.x = float(ly * self.max_speed)
                twist.linear.y = float(lx * self.max_speed)
                twist.linear.z = float(vz * self.max_speed)
                twist.angular.z = float(rx * self.max_turn)

                self.pub.publish(twist)
                self.get_logger().info(
                    f"cmd_vel: linear=({twist.linear.x}, {twist.linear.y}, {twist.linear.z}), angular=({twist.angular.z})"
                )

                time.sleep(0.05)  # Add a small delay to reduce the publishing rate
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
        finally:
            self.running = False

    def stop_robot(self):
        twist = Twist()
        self.pub.publish(twist)
        self.get_logger().info("Robot stopped")

    def destroy_node(self):
        self.running = False  # Ensure the thread stops when the node is destroyed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    topic = sys.argv[1] if len(sys.argv) > 1 else '/cmd_vel'
    node = TeleopDualShock4(topic)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
