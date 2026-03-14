#!/usr/bin/env python3
import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur = FindPackageShare(package="ur_robot_driver").find("ur_robot_driver")
    rviz_cfg = os.path.join(FindPackageShare("smart_factory_bringup").find("smart_factory_bringup"),
                            "rviz","smart_factory_scene.rviz")
    arm_pkg = FindPackageShare(
        "smart_factory_arm_description").find("smart_factory_arm_description")
    urdf_path = os.path.join(arm_pkg, "urdf", "smart_factory_arm.urdf")

    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_xml = f.read()

    smartfactory_description = FindPackageShare('smartfactory_description').find('smartfactory_description')
    urdf = (xacro.process_file(os.path.join(smartfactory_description,
                                            "urdf/smart_spawn.urdf.xacro"),
                              ))
    pretty_urdf = urdf.toprettyxml(indent='   ')

    # Declarando os argumentos de lançamento
    declare_launch_rviz = DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz")
    declare_launch_kinect = DeclareLaunchArgument("kinect", default_value="true", description="Launch Kinect")
    declare_launch_astra = DeclareLaunchArgument("astra", default_value="true", description="Launch Astra")
    declare_launch_ur10 = DeclareLaunchArgument("ur10", default_value="true", description="Launch UR10")
    
    aruco_params_file = os.path.join(get_package_share_directory('smart_factory_bringup'),'param','aruco_params.yaml')

    with open(aruco_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config_kinect = config["/aruco_node_kinect"]["ros__parameters"]
    config_astra = config["/aruco_node_astra"]["ros__parameters"]


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

    astra_aruco_node = Node(
        condition = IfCondition(LaunchConfiguration('astra')),
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        name='astra_aruco',
        parameters=[config_astra],
        output='screen',
        emulate_tty=True
    )

    description = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="cameras",
            parameters=[{
                "robot_description": pretty_urdf,
                "use_sim_time": False,
                "publish_frequency": 30.0
                }]
        )

    kinect_node = Node(
        condition=IfCondition(LaunchConfiguration('kinect')),
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
        output="screen",
        namespace="/kinect",
    )

    kinect_aruco_node = Node(
        condition = IfCondition(LaunchConfiguration('kinect')),
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        name='kinect_aruco',
        parameters=[config_kinect],
        output='screen',
        emulate_tty=True
    )

    tf_aruco_kinect = Node(
        condition = IfCondition(LaunchConfiguration('kinect')),
        package='smart_factory_bringup', 
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

    vacuum_grip_node = Node(
        package='smartfactory_ur_utils',
        executable='vacuum_grip_detect',
        name='vacuum_gripper_node',
        output='screen'
    )

    return LaunchDescription([
        declare_launch_rviz,
        declare_launch_kinect,
        declare_launch_astra,
        declare_launch_ur10,
        spawn_ur,
        description,
        astra_node,
        kinect_node,
        rsp_custom,
        rviz,
        # kinect_aruco_node,
        astra_aruco_node,
        tf_aruco_kinect,
        tf_map,
        aruco_filtered,
        kinematics,
        vacuum_grip_node
    ])
