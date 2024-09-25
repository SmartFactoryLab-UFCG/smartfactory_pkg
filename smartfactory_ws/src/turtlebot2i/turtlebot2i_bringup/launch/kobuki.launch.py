# This script launches the Kobuki base and additional components such as teleoperation,
# RViz visualization, and battery monitoring for the TurtleBot2i.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    # Declare launch arguments
    teleop_arg = DeclareLaunchArgument('teleop', default_value='false', description='Execute teleop control.')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Execute simulation.')
    
    turtlebot2i_monitoring = FindPackageShare(package="turtlebot2i_monitoring").find("turtlebot2i_monitoring")
    
    declare_safety = DeclareLaunchArgument('safety', default_value='false', description='Enable safety remapping.')


    # Executes the Kobuki Node, with the option to publish or not the TF Data 
    # for robot Odometry.
    share_dir = get_package_share_directory('turtlebot2i_bringup')
    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    params_file = os.path.join(share_dir, 'config', 'kobuki_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    
    # Define remappings based on the 'safety' argument
    kobuki_ros_node = GroupAction([
                Node(
                    package='kobuki_node',
                    executable='kobuki_ros_node',
                    output='both',
                    parameters=[params],
                    remappings = [('/commands/velocity', '/safety/velocity')],
                    condition=IfCondition(LaunchConfiguration('safety'))
                ),
                Node(
                    package='kobuki_node',
                    executable='kobuki_ros_node',
                    output='both',
                    parameters=[params],
                    condition=UnlessCondition(LaunchConfiguration('safety'))
                )
    ])
                

    # Static transform publisher from map to odom for RViz
    tf = Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
              condition=IfCondition(LaunchConfiguration("rviz")),
              arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    # Battery monitoring 
    status_battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(turtlebot2i_monitoring, 'launch', 'robot_monitoring.launch.py')])
    )

    return LaunchDescription([
        teleop_arg,
        rviz_arg,
        kobuki_ros_node,
        declare_safety,
        tf,
        status_battery_launch
    ])
