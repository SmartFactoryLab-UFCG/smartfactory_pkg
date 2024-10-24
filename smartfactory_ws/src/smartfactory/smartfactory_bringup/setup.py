from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smartfactory_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        py_modules=[
        'smartfactory_bringup.kinect_aruco_pose_transformer', 
        'smartfactory_bringup.basler_aruco_pose_transformer'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smartfactory-lab',
    maintainer_email='marina.batista@ee.ufcg.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_aruco_pose_transformer = smartfactory_bringup.kinect_aruco_pose_transformer:main',
            'basler_aruco_pose_transformer = smartfactory_bringup.basler_aruco_pose_transformer:main',
        ],
    },
)
