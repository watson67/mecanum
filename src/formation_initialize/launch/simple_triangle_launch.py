from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    triangle_node = Node(
        package='formation_initialize',
        executable='simple_triangle',
        name='simple_triangle',
        output='screen',
        emulate_tty=True,  # Permet l'interaction avec le terminal
    )
    
    visu_node = Node(
        package='formation_initialize',
        executable='trajectory_visualizer',
        name='trajectory_visualizer',
        output='screen',
        emulate_tty=True,  # Nécessaire pour afficher correctement les logs
    )
    
    return LaunchDescription([
        triangle_node,
        visu_node,
    ])