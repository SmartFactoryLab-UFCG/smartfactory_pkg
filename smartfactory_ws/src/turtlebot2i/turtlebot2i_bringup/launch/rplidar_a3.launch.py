# This script configures and launches the RPLidar A3 for the TurtleBot2i, with an option
# to include RViz visualization.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Execute RViz simulation.')

    # Define package paths
    rviz_path = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'rviz', 'real_turtlebot2i.rviz')
    turtlebot2i = FindPackageShare(package="turtlebot2i_simulation").find("turtlebot2i_simulation")

    # Path to LiDAR parameters file
    lidar_config = os.path.join(
        get_package_share_directory('turtlebot2i_bringup'),
        'config',
        'rplidar_a3_params.yaml'
    )
    
    # Node for the RPLidar
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_node',
        parameters=[lidar_config],
        remappings = [
                ('/scan', '/lidar_scan')
            ],
        output='screen'
    )

    # Initialize RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot2i, 'launch','single_launch','rviz.launch.py')),
        condition=IfCondition(LaunchConfiguration("rviz")),
        launch_arguments={'rviz_scene': rviz_path}.items()
    )
    return LaunchDescription([
        rplidar_node,
        rviz_arg,
        rviz_launch
    ])
