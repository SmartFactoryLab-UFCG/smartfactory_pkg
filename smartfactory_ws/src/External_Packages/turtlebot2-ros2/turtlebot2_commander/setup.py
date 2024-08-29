from setuptools import setup

package_name = 'turtlebot2_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igor',
    maintainer_email='252910@student.pwr.edu.pl',
    description='Package for nav2_simple_commander scripts for TurtleBot2 in ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
