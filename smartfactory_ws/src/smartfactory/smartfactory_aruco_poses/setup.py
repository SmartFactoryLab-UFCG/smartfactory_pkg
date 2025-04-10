from setuptools import find_packages, setup

package_name = 'smartfactory_aruco_poses'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'smartfactory_aruco_poses.aruco_metrics',
        'smartfactory_aruco_poses.filtered_pose',
        'smartfactory_aruco_poses.plot',     
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
            'aruco_metrics= smartfactory_aruco_poses.aruco_metrics:main',
            'filtered_pose= smartfactory_aruco_poses.filtered_pose:main',
            'plot= smartfactory_aruco_poses.plot:main',
        ],
    },
)
