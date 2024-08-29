# This script launches the Kobuki base and additional components such as teleoperation,
# RViz visualization, and battery monitoring for the TurtleBot2i.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    teleop_arg = DeclareLaunchArgument('teleop', default_value='false', description='Execute teleop control.')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Execute RViz simulation.')
    
    # Define package paths
    kobuki_node = FindPackageShare(package="kobuki_node").find("kobuki_node")
    turtlebot2i_monitoring = FindPackageShare(package="turtlebot2i_monitoring").find("turtlebot2i_monitoring")
    
    # Initialize teleoperation process
    teleop_launch = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("teleop")),
        cmd=[
            'xterm', '-e', 'bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=commands/velocity"'
        ],
        output='screen'
    )

    # Include kobuki_node launch file 
    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(kobuki_node, 'launch', 'kobuki_node-launch.py')])
    )

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
        kobuki_launch,
        teleop_launch,
        tf,
        status_battery_launch
    ])
