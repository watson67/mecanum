from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtle_swarm',
            executable='spawn_3turtles',
            name='spawn_turtles'
        ),
        Node(
            package='turtle_swarm',
            executable='pose_conversion2',
            name='pose_converter'
        ),

        Node(
            package='turtle_swarm',
            executable='consensus',
            name='consensus',
        ),
    ])