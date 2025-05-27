import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Point
from mecanum_swarm.trajectory import RECTANGLE_POINTS, CIRCLE_POINTS, EIGHT_POINTS, TEST_POINTS

TRAJECTORIES = {
    "rectangle": RECTANGLE_POINTS,
    "circle": CIRCLE_POINTS,
    "eight": EIGHT_POINTS,
    "test": TEST_POINTS,
}

class GoalPointSender(Node):
    def __init__(self, trajectory_name):
        super().__init__('goal_point_sender')
        self.trajectory_name = trajectory_name
        self.points = TRAJECTORIES.get(trajectory_name)
        if self.points is None:
            self.get_logger().error(f"Trajectoire inconnue : {trajectory_name}")
            rclpy.shutdown()
            sys.exit(1)
        self.current_index = 0
        self.last_point_sent = False

        # Publishers
        self.goal_publisher = self.create_publisher(Point, '/goal_point', 10)
        self.trajectory_type_pub = self.create_publisher(String, '/trajectory_type', 10)
        self.master_pub = self.create_publisher(Int32, '/master', 10)  # Ajout du publisher /master

        # Publier le type de trajectoire au démarrage
        msg = String()
        msg.data = self.trajectory_name
        self.trajectory_type_pub.publish(msg)
        self.trajectory_type_published = False
        self.create_timer(0.5, self.publish_trajectory_type_once)
        
        # S'abonner à /target_reached
        self.create_subscription(Int32, '/target_reached', self.target_reached_callback, 10)
        # Optionnel : s'abonner à /master pour réinitialiser la trajectoire si besoin
        self.create_subscription(Int32, '/master', self.master_callback, 10)

        self.get_logger().info(f'Noeud GoalPointSender initialisé pour la trajectoire : {self.trajectory_name}')
        self.get_logger().info(f'Nombre de points : {len(self.points)}')
        # Ne pas publier le premier point immédiatement, attendre /master si besoin

    def publish_trajectory_type_once(self):
        if not self.trajectory_type_published:
            msg = String()
            msg.data = self.trajectory_name
            self.trajectory_type_pub.publish(msg)
            self.get_logger().info(f"Type de trajectoire republié : {self.trajectory_name}")
            self.trajectory_type_published = True

    def master_callback(self, msg):
        if msg.data == 1:
            # Réinitialiser et démarrer la trajectoire
            self.current_index = 0
            self.last_point_sent = False
            self.publish_next_point()

    def target_reached_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info('Cible atteinte, passage au point suivant')
            if self.last_point_sent:
                self.get_logger().info("Tous les points ont été envoyés, trajectoire terminée.")
                self.last_point_sent = False
                # Publier 1 sur /master
                master_msg = Int32()
                master_msg.data = 0
                self.master_pub.publish(master_msg)
            else:
                self.publish_next_point()

    def publish_next_point(self):
        if self.current_index < len(self.points):
            x, y = self.points[self.current_index]
            point_msg = Point()
            point_msg.x = float(x)
            point_msg.y = float(y)
            point_msg.z = 0.0
            self.goal_publisher.publish(point_msg)
            self.get_logger().info(f'Point d\'objectif publié {self.current_index+1}/{len(self.points)} : x={x:.3f}, y={y:.3f}')
            self.current_index += 1
            if self.current_index == len(self.points):
                self.last_point_sent = True

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Utilisation : ros2 run mecanum_swarm goal_point_sender <rectangle|circle|eight>")
        sys.exit(1)
    trajectory_name = sys.argv[1].lower()
    node = GoalPointSender(trajectory_name)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
