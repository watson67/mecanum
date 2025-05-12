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
        #Node(
        #    package='mecanum_swarm',
        #    executable='tf2_visu',
        #    name='tf2_visu'
        #),
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
