from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot2i_slam_evaluation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotxr-humble',
    maintainer_email='felipe.nascimento@virtus.ufcg.edu.br',
    description='Slam evaluation package for RobotXR',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_error = turtlebot2i_slam_evaluation.odometry_error_node:main',
            'plot_bag = turtlebot2i_slam_evaluation.plot_bag_data:main',
        ],
    },
)
