from setuptools import find_packages, setup

package_name = 'kinect_behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smartfactory-lab',
    maintainer_email='marina.batista@ee.ufcg.edu.br',
    description='Pacote de controle baseado em visão para a fábrica inteligente usando ROS 2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_behavior_node = kinect_behavior_tree.kinect_behavior_node:main',
        ],
    },
)
