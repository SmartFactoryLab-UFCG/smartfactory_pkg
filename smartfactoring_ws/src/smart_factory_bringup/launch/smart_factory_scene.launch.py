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

    astra_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(FindPackageShare("smart_factory_bringup").find("smart_factory_bringup"), "launch", "astra_pro_plus.launch.py")]),
            condition=IfCondition(LaunchConfiguration('astra'))
    )

    kinect_node = Node(
        condition=IfCondition(LaunchConfiguration('kinect')),
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
        output="screen",
        namespace="/camera/color",
    )

    kinect_aruco_node = Node(
        condition = IfCondition(LaunchConfiguration('start_kinect')),
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        name='kinect_aruco',
        parameters=[{
            "marker_size": LaunchConfiguration('kinect_marker_size'),
            "aruco_dictionary_id": LaunchConfiguration('kinect_aruco_dictionary_id'),
            "image_topic": LaunchConfiguration('kinect_image_topic'),
            "use_depth_input": LaunchConfiguration('kinect_use_depth_input'),
            "depth_image_topic": LaunchConfiguration('kinect_depth_image_topic'),
            "camera_info_topic": LaunchConfiguration('kinect_camera_info_topic'),
            "camera_frame": LaunchConfiguration('kinect_camera_frame'),
            "detected_markers_topic": LaunchConfiguration('kinect_detected_markers_topic'),
            "markers_visualization_topic": LaunchConfiguration('kinect_markers_visualization_topic'),
            "output_image_topic": LaunchConfiguration('kinect_output_image_topic'),
        }],
        output='screen',
        emulate_tty=True
    )

    tf_aruco_kinect = Node(
        condition = IfCondition(LaunchConfiguration('start_kinect')),
        package='smartfactory_bringup', 
        name="tf_aruco_kinect",
        executable='kinect_aruco_pose_transformer'
        )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=['-d', rviz_cfg],
    )

    tf_map = Node(
        package='tf2_ros', 
        name="tf_map_world",
        executable='static_transform_publisher', 
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
        )

    aruco_filtered = Node(
        package='smartfactory_aruco_poses',
        executable='filtered_pose',
        name='aruco_filtered',
        output='screen',
    )

    kinematics = Node(
        #package='smartfactory_simulation',
        package='smartfactory_ur_utils',
        executable='calculate_kinematics',
        name='kinematics',
        output='screen',
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
        kinect_aruco_node,
        tf_aruco_kinect,
        tf_map,
        aruco_filtered,
        kinematics
    ])
