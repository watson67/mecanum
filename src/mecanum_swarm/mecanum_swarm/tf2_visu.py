import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from mecanum_swarm.config import ALL_ROBOT_NAMES, ROBOT_NEIGHBORS

GLOBAL_FRAME = "mocap"

class Swarm(Node):
    def __init__(self):
        super().__init__('swarm')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        for robot_name in ALL_ROBOT_NAMES:
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    GLOBAL_FRAME, f"{robot_name}/base_link", rclpy.time.Time()
                )
                position = transform.transform.translation
                self.get_logger().info(
                    f"Position de {robot_name:<10} dans {GLOBAL_FRAME}: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"Impossible de récupérer la position de {robot_name:<10}: {e}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = Swarm()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()