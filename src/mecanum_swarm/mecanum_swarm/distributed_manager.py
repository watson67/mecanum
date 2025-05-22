import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

ALL_ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]

class DistributedManager(Node):
    def __init__(self):
        super().__init__('distributed_manager')
        self.target_status = {name: 0 for name in ALL_ROBOT_NAMES}
        self.target_reached_publisher = self.create_publisher(Int32, "/target_reached", 10)
        self.last_published = 0

        for name in ALL_ROBOT_NAMES:
            self.create_subscription(
                Int32,
                f"/{name}/target_status",
                lambda msg, robot=name: self.target_status_callback(msg, robot),
                10
            )

    def target_status_callback(self, msg, robot_name):
        self.target_status[robot_name] = msg.data
        # Tous à 1 -> publier 1, sinon publier 0 si déjà publié 1
        if all(status == 1 for status in self.target_status.values()):
            if self.last_published != 1:
                out = Int32()
                out.data = 1
                self.target_reached_publisher.publish(out)
                self.get_logger().info("Tous les robots ont atteint leur cible. /target_reached = 1")
                self.last_published = 1
        else:
            if self.last_published != 0:
                out = Int32()
                out.data = 0
                self.target_reached_publisher.publish(out)
                self.get_logger().info("/target_reached = 0 (au moins un robot n'a pas atteint la cible)")
                self.last_published = 0

def main(args=None):
    rclpy.init(args=args)
    node = DistributedManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
