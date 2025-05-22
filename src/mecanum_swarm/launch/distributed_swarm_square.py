from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_swarm',
            executable='tf2_manager',
            name='tf2_manager'
        ),
        
        Node(
            package='mecanum_swarm',
            executable='rectangle',
            name='rectangle'
        ),
        Node(
            package='mecanum_swarm',
            executable='cmd_vel_rate',
            name='cmd_vel_rate',
            parameters=[{'csv_filename': 'cmd_vel_rate.csv'}]
        ),
        Node(
            package='mecanum_swarm',
            executable='barycenter_logger',
            name='barycenter_logger',
            parameters=[{'csv_filename': 'distributed_barycenter_logger.csv'}]
        ),
        Node(
            package='mecanum_swarm',
            executable='distances_logger',
            name='distances_logger',
            parameters=[{'csv_filename': 'distributed_distance_logger.csv'}]
        ),
        Node(
            package='mecanum_swarm',
            executable='goal_point_logger',
            name='goal_point_logger',
            parameters=[{'csv_filename': 'distributed_goal_point_logger.csv'}]
        ),
    ])
