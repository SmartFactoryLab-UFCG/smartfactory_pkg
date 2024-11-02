from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smartfactory_simulation'
def package_files(directory):
    """Recursively gather all files under a directory."""
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            filepath = os.path.join(path, filename)
            # Exclude directories and include files only
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [filepath]))
    return paths

data_files_to_include = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.*'), recursive=True)),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
]

data_files_to_include += package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'smartfactory_simulation.kinematics',
        'smartfactory_simulation.SM_Transformations',  # Certifique-se de incluir o Transformations
        'smartfactory_simulation.kinematics_teste1',
        'smartfactory_simulation.kinematics_teste2',
        'smartfactory_simulation.aruco_metrics',
        'smartfactory_simulation.filtered_pose',
        'smartfactory_simulation.plot',
        'smartfactory_simulation.calculate_kinematics',
        'smartfactory_simulation.send_angles',
        'smartfactory_simulation.ventosa_on',
        'smartfactory_simulation.ventosa_off',
        'smartfactory_simulation.send_conveyor'
    ],
    data_files=data_files_to_include,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smartfactory-lab',
    maintainer_email='marina.batista@ee.ufcg.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics = smartfactory_simulation.kinematics:main',
            'SM_Transformations = smartfactory_simulation.SM_Transformations:main',
            'kinematics_teste1= smartfactory_simulation.kinematics_teste1:main',
            'kinematics_teste2= smartfactory_simulation.kinematics_teste2:main',
            'aruco_metrics= smartfactory_simulation.aruco_metrics:main',
            'filtered_pose= smartfactory_simulation.filtered_pose:main',
            'plot= smartfactory_simulation.plot:main',
            'calculate_kinematics= smartfactory_simulation.calculate_kinematics:main',
            'send_angles= smartfactory_simulation.send_angles:main',
            'ventosa_on=smartfactory_simulation.ventosa_on:main',
            'ventosa_off=smartfactory_simulation.ventosa_off:main',
            'send_conveyor=smartfactory_simulation.send_conveyor:main'
        ],
        
    },
)
