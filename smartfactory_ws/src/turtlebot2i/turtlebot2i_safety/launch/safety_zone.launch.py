# This launch file initializes and launches multiple nodes required for the TurtleBot2i robot's
# safety zone management. It includes nodes for bringing up the robot, starting the kobuki base,
# monitoring the battery status, and defining safety zones along with speed adjustment nodes.

import os
import ament_index_python.packages
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the packages for bringing up the robot and monitoring
    turtlebot2i_bringup = FindPackageShare(package="turtlebot2i_bringup").find("turtlebot2i_bringup")
    turtlebot2i_monitoring = FindPackageShare(package="turtlebot2i_monitoring").find("turtlebot2i_monitoring")
    
    # Include the launch file for bringing up the robot
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(turtlebot2i_bringup, 'launch', 'base_camera_lidar.launch.py')]),
        launch_arguments={'Start_base': 'false'}.items()
    )

    # Initialize kobuki base
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    # Create node for kobuki base
    kobuki_ros_node = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        output='both',
        parameters=[params],
        remappings=[
            ('/commands/velocity', '/safety/velocity'),
        ]
    )
    
    # Include the launch file for monitoring battery status
    status_battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(turtlebot2i_monitoring, 'launch', 'robot_monitoring.launch.py')])
    )

    # Node for defining safety zones
    zone_definition_node = Node(
        package='turtlebot2i_safety',
        executable='zone_definition_node',
        name='zone_definition_node',
    )

    # Node for adjusting speed based on safety zones
    speed_return_node = Node(
        package= 'turtlebot2i_safety',
        executable='speed_return_node',
        name='speed_return_node',
    )

    return LaunchDescription([
        kobuki_ros_node,
        status_battery_launch,
        robot_bringup,
        zone_definition_node,
        speed_return_node
    ])
