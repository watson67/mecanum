from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Lance le nœud master et le nœud consensus_v2 en même temps.
    Le nœud master permet de contrôler l'activation/désactivation
    de l'algorithme de consensus via l'appui sur la touche espace.
    """
    master_node = Node(
        package='consensus',
        executable='master',
        name='master_node',
        output='screen',
        emulate_tty=True,  # Permet l'interaction avec le terminal
    )
    
    consensus_node = Node(
        package='consensus',
        executable='consensus_v2',
        name='consensus_swarm',
        output='screen',
        emulate_tty=True,  # Nécessaire pour afficher correctement les logs
    )
    
    return LaunchDescription([
        master_node,
        consensus_node,
    ])
