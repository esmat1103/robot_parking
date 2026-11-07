"""
simulation.launch.py : démarre TOUTE la simulation en une commande.

    1. Gazebo avec le parking 3D          (worlds/parking.world)
    2. robot_state_publisher              (lit robot.urdf, publie la forme du robot)
    3. spawn_entity                       (pose le robot à l'entrée du parking)
    4. map_server + lifecycle_manager     (publie la carte sur /map)
    5. static_transform_publisher         (relie la carte "map" au repère "odom")
    6. RViz                               (affiche carte + robot + capteur)
    7. planificateur                      (A* : calcule le chemin vers le but cliqué)

Utilisation :
    ros2 launch robot_parking simulation.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from robot_parking.carte import ENTREE


def generate_launch_description():
    dossier = get_package_share_directory('robot_parking')
    monde = os.path.join(dossier, 'worlds', 'parking.world')
    fichier_carte = os.path.join(dossier, 'maps', 'parking.yaml')
    config_rviz = os.path.join(dossier, 'rviz', 'simulation.rviz')
    with open(os.path.join(dossier, 'urdf', 'robot.urdf')) as f:
        description_robot = f.read()

    temps_simu = {'use_sim_time': True}   # tous les nœuds suivent l'horloge de Gazebo

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': monde}.items(),
    )

    # 2. Forme du robot (publie /robot_description et les repères TF du robot)
    etat_robot = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description_robot}, temps_simu],
        output='screen',
    )

    # 3. Pose le robot dans Gazebo, à l'entrée, tourné vers la droite (+x)
    poser_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot',
                   '-x', str(ENTREE[0]), '-y', str(ENTREE[1]), '-z', '0.005', '-Y', '0.0'],
        output='screen',
    )

    # 4. La carte
    serveur_carte = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'yaml_filename': fichier_carte}, temps_simu], output='screen',
    )
    allumage_carte = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_carte',
        parameters=[{'autostart': True, 'node_names': ['map_server']}, temps_simu],
        output='screen',
    )

    # 5. map -> odom : identiques, car la carte et le monde Gazebo ont la même origine
    lien_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[temps_simu],
    )

    # 6. RViz
    rviz = Node(
        package='rviz2', executable='rviz2', arguments=['-d', config_rviz],
        parameters=[temps_simu], output='screen',
    )

    # 7. Planificateur A* (étape 4) : /map + /odom + /goal_pose -> /plan
    planificateur = Node(
        package='robot_parking', executable='planificateur',
        parameters=[{'rayon_securite': 0.15}, temps_simu], output='screen',
    )

    return LaunchDescription([gazebo, etat_robot, poser_robot,
                              serveur_carte, allumage_carte, lien_map_odom, rviz,
                              planificateur])
