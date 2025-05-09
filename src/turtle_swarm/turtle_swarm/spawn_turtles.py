#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.get_logger().info('Launching turtlesim')
        
        # Créer le client pour le service spawn et kill
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')
        
        # Tuer la tortue par défaut (nommée 'turtle1')
        self.get_logger().info('Attempting to kill turtle1...')
        self.kill_turtle('turtle1')

        # Spawn des tortues athos et porthos
        self.spawn_turtle('athos', 2.0, 2.0, 0.0)
        self.spawn_turtle('orthos', 3.0, 3.0, 0.0)

    def kill_turtle(self, name):
        if not self.kill_client.service_is_ready():
            self.get_logger().error(f'Service /kill not available')
            return

        request = Kill.Request()
        request.name = name

        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Turtle {name} killed successfully!')
        else:
            self.get_logger().error(f'Failed to kill {name}.')

    def spawn_turtle(self, name, x, y, theta):
        request = Spawn.Request()
        request.name = name
        request.x = x
        request.y = y
        request.theta = theta

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Turtle {name} spawned successfully!')
        else:
            self.get_logger().error(f'Failed to spawn {name}.')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

