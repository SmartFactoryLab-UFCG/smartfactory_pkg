import os
from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.actions import SetParameter
from typing import Text
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
        
def generate_launch_description():

    turtlebot2i_navigation_dir = get_package_share_directory('turtlebot2i_navigation'),
    bringup_dir = get_package_share_directory('nav2_bringup'),
    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    # map_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'maps', 'turtlebot3_world.yaml')
    params_dir = os.path.join(get_package_share_directory('turtlebot2i'), 'params', 'robotxr_nav2_params.yaml')
    # params_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')

    return LaunchDescription([
        
        # Declare launch arguments
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='odom'),
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('laser_topic', default_value='scan'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_namespace', default_value='false', description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument('slam', default_value='False', description='Whether run a SLAM'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('use_composition', default_value='True', description='Whether to use composed bringup'),
        DeclareLaunchArgument('use_respawn', default_value='False', description='Whether to respawn if a node crashes. Applied when composition is disabled.'),

        # Define paths to config files
        # DeclareLaunchArgument('custom_param_file', default_value=os.path.join(turtlebot2i_navigation_dir, 'param', 'dummy.yaml')),
        # DeclareLaunchArgument('map', default_value=map_dir, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=params_dir, description='Full path to the ROS2 parameters file to use for all launched nodes'),

        GroupAction(
            actions=[
                SetRemap(src='mobile_base/cmd_vel', dst='/navigation_velocity_smoother/raw_cmd_vel'),
                IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_dir),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'use_namespace': LaunchConfiguration('use_namespace'),
                    'slam': LaunchConfiguration('slam'),
                    # 'map': LaunchConfiguration('map'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('params_file'),
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': LaunchConfiguration('use_composition'),
                    'use_respawn': LaunchConfiguration('use_respawn'),
                }.items(),
                )
            ]
        ),
    ])