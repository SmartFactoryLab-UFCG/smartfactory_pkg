from setuptools import find_packages, setup

package_name = 'smartfactory_ur_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'smartfactory_ur_utils.SM_Transformations',  
        'smartfactory_ur_utils.calculate_kinematics',
        'smartfactory_ur_utils.send_angles',
        'smartfactory_ur_utils.ventosa_on',
        'smartfactory_ur_utils.ventosa_off',
        'smartfactory_ur_utils.send_conveyor',
        'smartfactory_ur_utils.vacuum_grip_detect'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'SM_Transformations = smartfactory_ur_utils.SM_Transformations:main',
            'calculate_kinematics= smartfactory_ur_utils.calculate_kinematics:main',
            'send_angles= smartfactory_ur_utils.send_angles:main',
            'ventosa_on=smartfactory_ur_utils.ventosa_on:main',
            'ventosa_off=smartfactory_ur_utils.ventosa_off:main',
            'send_conveyor=smartfactory_ur_utils.send_conveyor:main',
            'vacuum_grip_detect=smartfactory_ur_utils.vacuum_grip_detect:main',
        ],
    },
)
