import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# Configuration des touches pour un clavier AZERTY
key_mapping = {
    'z': (1, 0, 0, 0),    # Avancer
    's': (-1, 0, 0, 0),   # Reculer
    'q': (0, 1, 0, 0),    # Translation gauche
    'd': (0, -1, 0, 0),   # Translation droite
    'a': (0, 0, 0, 1),    # Rotation gauche
    'e': (0, 0, 0, -1),   # Rotation droite
    ' ': (0, 0, 0, 0)     # Stop
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopMecanum(Node):
    def __init__(self, topic_name):
        super().__init__('teleop_mecanum')
        self.publisher = self.create_publisher(Twist, topic_name, 10)
        self.get_logger().info(f'Teleop Mecanum prêt. Publie sur {topic_name}')

    def run(self):
        try:
            while rclpy.ok():
                key = get_key()
                if key == '\x03':  # CTRL+C pour quitter
                    break
                
                twist = Twist()
                if key in key_mapping:
                    x, y, z, th = key_mapping[key]
                    twist.linear.x = x * 0.5
                    twist.linear.y = y * 0.5
                    twist.angular.z = th * 1.0
                
                self.publisher.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    topic = sys.argv[1] if len(sys.argv) > 1 else '/cmd_vel'
    node = TeleopMecanum(topic)
    node.run()
    node.destroy_node()
    rclpy.shutdown()
