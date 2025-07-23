from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_manager',
            executable='tf2_manager',
            name='tf2_manager'
        ),
        Node(
            package='swarm_manager',
            executable='distributed_manager',
            name='distributed_manager'
        ),
        Node(
            package='swarm_manager',
            executable='goal_point_sender',
            name='goal_point_sender',
            arguments=['rectangle']  # 'rectangle' 'circle' ou 'eight' 
        ),
        Node(
            package='logger',
            executable='cmd_vel_rate_logger',
            namespace='logger',
            name='cmd_vel_rate_logger',
            arguments=['predictive']
        ),
        Node(
            package='logger',
            executable='barycenter_logger',
            namespace='logger',
            name='barycenter_logger',
            parameters=[{'csv_filename': 'distributed_barycenter_logger.csv'}],
            arguments=['predictive']
        ),
        Node(
            package='logger',
            executable='distances_logger',
            namespace='logger',
            name='distances_logger',
            parameters=[{'csv_filename': 'distributed_distance_logger.csv'}],
            arguments=['predictive']
        ),
        Node(
            package='logger',
            executable='goal_point_logger',
            namespace='logger',
            name='goal_point_logger',
            parameters=[{'csv_filename': 'distributed_goal_point_logger.csv'}],
            arguments=['predictive']
        ),
        Node(
            package='logger',
            executable='predictive_logger',
            namespace='logger',
            name='predictive_logger',
            arguments=['predictive']
        ),
    ])