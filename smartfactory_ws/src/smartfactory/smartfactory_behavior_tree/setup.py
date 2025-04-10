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
        (os.path.join('share', package_name, 'ur10'), glob('ur10/*.py')),
        (os.path.join('share', package_name, 'algoritms_perception'), glob('algoritms_perception/*.py')),
        (os.path.join('share', package_name, 'supervisores'), glob('supervisores/*.py')),
        (os.path.join('share', package_name, 'utils'), glob('utils/*.py')),
        (os.path.join('share', package_name, 'teste'), glob('teste/*.py')),
        (os.path.join('share', package_name, 'perception'), glob('perception/*.py')),  
        
    ],
    install_requires=['setuptools', 'smartfactory_ur_utils'],
    zip_safe=True,
    maintainer='smartfactory-lab',
    maintainer_email='marina.batista@ee.ufcg.edu.br',
    description='Pacote de controle baseado em árvores de comportamento usando ROS 2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_kinect_node = smartfactory_behavior_tree.perception.behavior_kinect_node:main',
            'behavior_basler_node = smartfactory_behavior_tree.teste.behavior_basler_node:main',
            'behavior_ur10_node = smartfactory_behavior_tree.teste.behavior_ur10_node:main',
            'behavior_aruco_node = smartfactory_behavior_tree.teste.behavior_aruco_node:main',
            'behavior_kinect_detect = smartfactory_behavior_tree.teste.behavior_kinect_detect:main',
            'behavior_basler_detect = smartfactory_behavior_tree.teste.behavior_basler_detect:main',
            'behavior_pick_and_place = smartfactory_behavior_tree.teste.behavior_pick_and_place:main',

            # 🔹 Adicionando os módulos do UR10 como nós executáveis
            'ur10_motion = smartfactory_behavior_tree.ur10.ur10_motion:main',
            'ur10_gripper = smartfactory_behavior_tree.ur10.ur10_gripper:main',
            'ur10_sensors = smartfactory_behavior_tree.ur10.ur10_sensors:main',
            # 🔹 Adicionando os módulos da percepção como nós executáveis
            'aruco_detector = smartfactory_behavior_tree.algoritms_perception.aruco_detector:main',

            'pick_and_place = smartfactory_behavior_tree.supervisores.pick_and_place:main',

        ],
    },
)


