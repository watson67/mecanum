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
            executable='distributed_manager',
            name='distributed_manager'
        ),
        
        Node(
            package='mecanum_swarm',
            executable='rectangle',
            name='rectangle'
        ),
        Node(
            package='mecanum_swarm',
            executable='cmd_vel_rate_logger',
            name='/logger/cmd_vel_rate_logger'
        ),
        Node(
            package='mecanum_swarm',
            executable='barycenter_logger',
            name='/logger/barycenter_logger',
            parameters=[{'csv_filename': 'distributed_barycenter_logger.csv'}]
        ),
        Node(
            package='mecanum_swarm',
            executable='distances_logger',
            name='/logger/distances_logger',
            parameters=[{'csv_filename': 'distributed_distance_logger.csv'}]
        ),
        Node(
            package='mecanum_swarm',
            executable='goal_point_logger',
            name='/logger/goal_point_logger',
            parameters=[{'csv_filename': 'distributed_goal_point_logger.csv'}]
        ),
    ])
