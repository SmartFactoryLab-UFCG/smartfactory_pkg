from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smart_factory_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        py_modules=[
        'smart_factory_bringup.kinect_aruco_pose_transformer', 
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'map'),   glob('map/*')),
        (os.path.join('share', package_name, 'models/turtlebot3_world'),   glob('turtlebot3_world/*')),
        (os.path.join('share', package_name, 'param'),   glob('param/*')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'),   glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smartfactory-lab',
    maintainer_email='marina.oiveira@virtus-cc.ufcg.edu.br',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_aruco_pose_transformer = smart_factory_bringup.kinect_aruco_pose_transformer:main',
        ],
    },
)
