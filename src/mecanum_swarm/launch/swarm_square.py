from launch import LaunchDescription
from launch_ros.actions import Node
import os

# Récupère dynamiquement le chemin du fichier .rviz comme dans la commande shell
rviz_config_path = '/home/eswarm/mecanum/src/mecanum_swarm/config/rviz.rviz'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_swarm',
            executable='tf2_manager',
            name='tf2_manager'
        ),
        Node(
            package='mecanum_swarm',
            executable='swarm',
            name='swarm_controller'
        ),
        Node(
            package='mecanum_swarm',
            executable='rectangle',
            name='rectangle'
        ),
        Node(
            package='mecanum_swarm',
            executable='cmd_vel_rate_logger',
            namespace='logger',
            name='cmd_vel_rate_logger'
        ),
        Node(
            package='mecanum_swarm',
            executable='barycenter_logger',
            namespace='logger',
            name='barycenter_logger',
            parameters=[{'csv_filename': 'barycenter_logger.csv'}]
        ),
        Node(
            package='mecanum_swarm',
            executable='distances_logger',
            namespace='logger',
            name='distances_logger',
            parameters=[{'csv_filename': 'distance_logger.csv'}]
        ),
        Node(
            package='mecanum_swarm',
            executable='goal_point_logger',
            namespace='logger',
            name='goal_point_logger',
            parameters=[{'csv_filename': 'goal_point_logger.csv'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                rviz_config_path
            ],
            output='screen'
        )
    ])
