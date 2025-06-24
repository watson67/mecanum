from launch import LaunchDescription
from launch_ros.actions import Node

# Ce launch file est à lancer sur chaque robot du swarm.
# Les noms des nœuds seront automatiquement uniques car ils incluent le hostname du robot.
def generate_launch_description():
    return LaunchDescription([
        # Publie les repères de transformation TF2 pour chaque robot
        # Nom du nœud: distributed_tf2_manager_{robot_name}
        Node(
            package='mecanum_swarm',
            executable='distributed_tf2_manager',
            name='distributed_tf2_manager',
            output='screen'
        ),
        
        # Responsable du contrôle du swarm distribué  
        # Nom du nœud: distributed_swarm_controller_{robot_name}
        Node(
            package='mecanum_swarm',
            executable='distributed_swarm',
            name='distributed_swarm_controller',
            output='screen'
        ),
    ])
