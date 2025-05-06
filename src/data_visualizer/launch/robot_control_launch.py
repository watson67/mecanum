from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_visualizer',
            executable='robot_control',
            name='aramis_controller',
            output='screen',
            arguments=['Aramis']
        ),
        Node(
            package='data_visualizer',
            executable='robot_control',
            name='athos_controller',
            output='screen',
            arguments=['Athos']
        ),
        Node(
            package='data_visualizer',
            executable='robot_control',
            name='porthos_controller',
            output='screen',
            arguments=['Porthos']
        )
    ])
