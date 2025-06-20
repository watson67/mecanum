from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch argument for enabling logging
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable or disable logging nodes'
    )
    
    # Get the launch configuration
    enable_logging = LaunchConfiguration('enable_logging')
    
    return LaunchDescription([
        enable_logging_arg,
        
        Node(
            package='mecanum_swarm',
            executable='tf2_manager',
            name='tf2_manager'
        ),
        #Node(
        #    package='mecanum_swarm',
        #    executable='formation_init',
        #    name='formation_init'
        #),
        Node(
            package='mecanum_swarm',
            executable='swarm',
            name='swarm_controller'
        ),
        Node(
            package='mecanum_swarm',
            executable='goal_point_sender',
            name='goal_point_sender',
            arguments=['rectangle']  # 'rectangle' 'circle' ou 'eight' 
        ),
        
        Node(
            package='mecanum_swarm',
            executable='cmd_vel_rate_logger',
            namespace='logger',
            name='cmd_vel_rate_logger',
            arguments=['classic'],
            condition=IfCondition(enable_logging)
        ),
        Node(
            package='mecanum_swarm',
            executable='barycenter_logger',
            namespace='logger',
            name='barycenter_logger',
            parameters=[{'csv_filename': 'distributed_barycenter_logger.csv'}],
            arguments=['classic'],
            condition=IfCondition(enable_logging)
        ),
        Node(
            package='mecanum_swarm',
            executable='distances_logger',
            namespace='logger',
            name='distances_logger',
            parameters=[{'csv_filename': 'distributed_distance_logger.csv'}],
            arguments=['classic'],
            condition=IfCondition(enable_logging)
        ),
        Node(
            package='mecanum_swarm',
            executable='goal_point_logger',
            namespace='logger',
            name='goal_point_logger',
            parameters=[{'csv_filename': 'distributed_goal_point_logger.csv'}],
            arguments=['classic'],
            condition=IfCondition(enable_logging)
        ),
    ])
