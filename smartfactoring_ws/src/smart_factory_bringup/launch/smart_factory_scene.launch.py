#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur = FindPackageShare(package="ur_robot_driver").find("ur_robot_driver")
    rviz_cfg = os.path.join(FindPackageShare("smart_factory_bringup").find("smart_factory_bringup"),
                            "rviz","smart_factory_scene.rviz")
    arm_pkg = FindPackageShare(
        "smart_factory_arm_description").find("smart_factory_arm_description")
    urdf_path = os.path.join(arm_pkg, "urdf", "smart_factory_arm.urdf")
    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_xml = f.read()

    # Declarando os argumentos de lançamento
    declare_launch_rviz = DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz")
    declare_launch_kinect = DeclareLaunchArgument("kinect", default_value="true", description="Launch Kinect")
    declare_launch_astra = DeclareLaunchArgument("astra", default_value="true", description="Launch Astra")
    declare_launch_ur10 = DeclareLaunchArgument("ur10", default_value="true", description="Launch UR10")
    
    spawn_ur = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(ur, "launch", "ur10.launch.py")]),
            launch_arguments={
                "robot_ip":"192.168.0.104",
                "launch_rviz":"false",
            }.items(),
            condition=IfCondition(LaunchConfiguration('ur10'))
    )

    astra_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(FindPackageShare("smart_factory_bringup").find("smart_factory_bringup"), "launch", "astra_pro_plus.launch.py")]),
            condition=IfCondition(LaunchConfiguration('astra'))
    )

    rsp_custom = Node(
        condition=IfCondition(LaunchConfiguration('ur10')),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="custom_ur10",
        parameters=[{
            "robot_description": urdf_xml,
            "use_sim_time": False
        }],
        output="screen",
    )

    kinect_node = Node(
        condition=IfCondition(LaunchConfiguration('kinect')),
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
        output="screen",
        namespace="/camera/color",
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=['-d', rviz_cfg],
    )

    web_video_server_node = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
    )

    return LaunchDescription([
        declare_launch_rviz,
        declare_launch_kinect,
        declare_launch_astra,
        declare_launch_ur10,
        spawn_ur,
        astra_node,
        kinect_node,
        rsp_custom,
        rviz,
        web_video_server_node
    ])
