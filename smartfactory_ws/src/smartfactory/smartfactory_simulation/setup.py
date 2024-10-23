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
        'smartfactory_simulation.Transformations',  # Certifique-se de incluir o Transformations
        'smartfactory_simulation.kinematics_teste1',
        'smartfactory_simulation.kinematics_teste2',
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
            'Transformations = smartfactory_simulation.Transformations:main',
            'kinematics_teste1= smartfactory_simulation.kinematics_teste1:main',
            'kinematics_teste2= smartfactory_simulation.kinematics_teste2:main',
        ],
        
    },
)
