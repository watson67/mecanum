from launch import LaunchDescription
from launch_ros.actions import Node
import socket

#Ce launch file est à lancer sur chaque robot du swarm.
def generate_launch_description():
    hostname = socket.gethostname().lower()
    # Optionnel : nettoyer le hostname si besoin (ex: enlever '-desktop')
    if hostname.endswith('-desktop'):
        hostname = hostname[:-8]
    robot_name = hostname.capitalize()

    return LaunchDescription([
        # Publie les repères de transformation TF2 pour chaque robot
        Node(
            package='mecanum_swarm',
            executable='distributed_tf2_obstacle_manager',
            name=f'distributed_tf2_manager_{robot_name.lower()}',
            output='screen'
        ),
        
        # Responsable du contrôle du swarm distribué
        Node(
            package='mecanum_swarm',
            executable='distributed_swarm',
            name=f'distributed_swarm_controller_{robot_name.lower()}',
            output='screen'
        ),
    ])
