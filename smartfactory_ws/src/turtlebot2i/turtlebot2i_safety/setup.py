from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot2i_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        package_name + '.zone_definition_node',
        package_name + '.speed_return_node',
        package_name + '.speed_scaling_simple',
        package_name + '.action_complete_safety',
        package_name + '.assessment_rules_demo'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/param', ['param/vel_mux.yaml']),
        ('share/' + package_name + '/rules', ['rules/ra_demo.data']),
        ('share/' + package_name + '/rules', ['rules/ra_full.data']),
        ('share/' + package_name + '/rules', ['rules/ra_full_newly_generated.data']),
    ],
    install_requires=[
        'setuptools',
        'scikit-fuzzy',
    ],
    zip_safe=True,
    maintainer='Larissa Silva',
    maintainer_email='larissa.silva@virtus.ufcg.edu.br',
    description='Safety nodes for Turtlebot2i',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zone_definition_node = ' + package_name + '.zone_definition_node:main',
            'speed_return_node = ' + package_name + '.speed_return_node:main',
            'speed_scaling_simple = ' + package_name + '.speed_scaling_simple:main',
            'action_complete_safety = ' + package_name + '.action_complete_safety:main',
            'assessment_rules_demo = ' + package_name + '.assessment_rules_demo:main',
            'mitigation_rules_demo = ' + package_name + '.mitigation_rules_demo:main',
        ],
    },
)
