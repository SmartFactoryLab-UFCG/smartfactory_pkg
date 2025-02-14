from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'smartfactory_behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smartfactory-lab',
    maintainer_email='marina.batista@ee.ufcg.edu.br',
    description='Pacote de controle baseado em árvores de comportamento usando ROS 2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_kinect_node = smartfactory_behavior_tree.behavior_kinect_node:main',
            'behavior_basler_node = smartfactory_behavior_tree.behavior_basler_node:main',
            'behavior_ur10_node = smartfactory_behavior_tree.behavior_ur10_node:main',
            'behavior_aruco_node = smartfactory_behavior_tree.behavior_aruco_node:main',
            'behavior_kinect_detect = smartfactory_behavior_tree.behavior_kinect_detect:main',
            'behavior_basler_detect = smartfactory_behavior_tree.behavior_basler_detect:main',
            'behavior_pick_and_place = smartfactory_behavior_tree.behavior_pick_and_place:main',
        ],
    },
)


