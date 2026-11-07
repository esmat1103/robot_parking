# setup.py : la "notice de rangement" du package (lue par colcon build)
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_parking'

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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),   # NOUVEAU (étape 3)
    ],
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
            'generer_monde = robot_parking.generer_monde:main',   # NOUVEAU (étape 3)
        ],
    },
)
