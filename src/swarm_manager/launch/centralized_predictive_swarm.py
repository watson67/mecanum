from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

type='predictive'  # 'event' or 'classic'
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
            package='centralized_swarm',
            executable='predictive_swarm',
            name='swarm_controller'
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
            arguments=[type],
            condition=IfCondition(enable_logging)
        ),
        Node(
            package='logger',
            executable='barycenter_logger',
            namespace='logger',
            name='barycenter_logger',
            parameters=[{'csv_filename': 'distributed_barycenter_logger.csv'}],
            arguments=[type],
            condition=IfCondition(enable_logging)
        ),
        Node(
            package='logger',
            executable='distances_logger',
            namespace='logger',
            name='distances_logger',
            parameters=[{'csv_filename': 'distributed_distance_logger.csv'}],
            arguments=[type],
            condition=IfCondition(enable_logging)
        ),
        Node(
            package='logger',
            executable='goal_point_logger',
            namespace='logger',
            name='goal_point_logger',
            parameters=[{'csv_filename': 'distributed_goal_point_logger.csv'}],
            arguments=[type],
            condition=IfCondition(enable_logging)
        ),
    ])