# setup.py : la "notice de rangement" du package (lue par colcon build)
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_parking'


def dossier_complet(dossier):
    """Installe un dossier AVEC ses sous-dossiers (glob seul ne descend pas dedans).
    Utile pour models/ : chaque voiture a ses sous-dossiers meshes/ et materials/."""
    liste = []
    for racine, _, fichiers in os.walk(dossier):
        if fichiers:
            liste.append((os.path.join('share', package_name, racine),
                          [os.path.join(racine, f) for f in fichiers]))
    return liste


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ] + dossier_complet('models'),                     # NOUVEAU (vision B) : voitures 3D + plaques
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esmat',
    maintainer_email='esmateddinekhamassi@gmail.com',
    description='Robot de guidage dans un parking',
    license='MIT',
    tests_require=['pytest'],
    # Les commandes "ros2 run robot_parking <nom>"
    entry_points={
        'console_scripts': [
            'generer_carte = robot_parking.generer_carte:main',
            'generer_monde = robot_parking.generer_monde:main',
            'planificateur = robot_parking.planificateur:main',
            'suiveur = robot_parking.suiveur:main',
            'securite = robot_parking.securite:main',
            'mission = robot_parking.mission:main',
            'collecteur = robot_parking.collecteur:main',         # vision V1
            'tester_lecture = robot_parking.tester_lecture:main', # vision V2
            'tester_yolo = robot_parking.tester_yolo:main',       # vision V4
            'pieton = robot_parking.pieton:main',                 # D1 : piéton
            'suiveur_dwa = robot_parking.suiveur_dwa:main',       # D3 : DWA
            'suiveur_ppo = robot_parking.suiveur_ppo:main',       # NOUVEAU (D4 : PPO)
        ],
    },
)
