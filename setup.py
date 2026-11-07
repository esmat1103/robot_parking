# setup.py : explique à colcon comment INSTALLER le package robot_parking

import os                     # pour construire des chemins de dossiers
from glob import glob         # pour lister des fichiers (ex : tous les .yaml)
from setuptools import find_packages, setup   # l'outil standard d'installation Python

package_name = 'robot_parking'

setup(
    # --- Identité (comme package.xml) ---
    name=package_name,
    version='0.1.0',

    # --- Le code Python ---
    # cherche automatiquement le dossier robot_parking/ (celui avec __init__.py)
    packages=find_packages(exclude=['test']),

    # --- Les autres fichiers à installer ---
    # format : (dossier de destination, [liste des fichiers à copier])
    data_files=[
        # marqueur pour que "ros2 pkg list" trouve le package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # la carte d'identité
        ('share/' + package_name, ['package.xml']),
        # nos 4 tiroirs (ajoutés par nous) :
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # démarrage
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),       # réglages
        (os.path.join('share', package_name, 'maps'),   glob('maps/*')),              # carte
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),            # monde Gazebo
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    # --- Infos auteur ---
    maintainer='esmat',
    maintainer_email='esmat@todo.todo',
    description='Robot de guidage dans un parking',
    license='MIT',
    tests_require=['pytest'],

    # --- Les programmes lançables avec "ros2 run" ---
    # vide pour l'instant ; plus tard on ajoutera par exemple :
    # 'planificateur = robot_parking.planificateur:main'
    entry_points={
        'console_scripts': [
        ],
    },
)
