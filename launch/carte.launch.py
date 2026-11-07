"""
carte.launch.py : affiche la carte du parking dans RViz.

Lance 3 nœuds :
    map_server        -> lit parking.yaml + parking.pgm et publie la carte sur /map
    lifecycle_manager -> "allume" map_server (les nœuds Nav2 démarrent éteints)
    rviz2             -> affiche la carte

Utilisation :
    ros2 launch robot_parking carte.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dossier = get_package_share_directory('robot_parking')
    fichier_carte = os.path.join(dossier, 'maps', 'parking.yaml')
    config_rviz = os.path.join(dossier, 'rviz', 'carte.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': fichier_carte}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_carte',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', config_rviz],
            output='screen',
        ),
    ])
