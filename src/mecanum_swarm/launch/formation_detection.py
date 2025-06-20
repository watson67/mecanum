from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

  
    return LaunchDescription([
        
        Node(
            package='mecanum_swarm',
            executable='tf2_manager',
            name='tf2_manager'
        ),
        Node(
            package='mecanum_swarm',
            executable='formation_init_spat',
            name='formation_init_spat',
            parameters=[{'enable_plotting': True}]
        ),

    ])
