#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, 
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(_):

    launch_rviz = LaunchConfiguration("launch_rviz")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    rviz_cfg = os.path.join(FindPackageShare("smart_factory_bringup").find("smart_factory_bringup"),
                            "rviz","smart_factory_scene.rviz")

    robot_driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
                )),
        launch_arguments={
            "robot_ip": robot_ip,
            "ur_type": "ur10",
            "use_mock_hardware": use_mock_hardware,
            "launch_rviz": "false"
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=['-d', rviz_cfg],
    )

    return [robot_driver, rviz]

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", description="To launch rviz or not",
            default_value="true"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached.",
            default_value="192.168.0.104"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware", description="Whether to use or not real hardware.",
            default_value="false"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("smart_factory_arm_description"),
                 "urdf", "smart_factory_arm.urdf.xacro"]
            ),
            description="URDF/XACRO description file with the robot.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
