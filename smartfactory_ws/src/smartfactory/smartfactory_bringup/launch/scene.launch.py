
import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the turtlebot2i_simulation package
    ur = FindPackageShare(package="ur_bringup").find("ur_bringup")
    turtlebot2i_bringup = FindPackageShare(package="turtlebot2i_bringup").find("turtlebot2i_bringup")
    aruco = FindPackageShare(package="aruco_pose_estimation").find("aruco_pose_estimation")

    smartfactory_description = FindPackageShare('smartfactory_description').find('smartfactory_description')
    urdf = (xacro.process_file(os.path.join(smartfactory_description,
                                            "urdf/smart_spawn.urdf.xacro"),
                              ))
    pretty_urdf = urdf.toprettyxml(indent='   ')
    
    # Include the launch file to spawn the Kinect on Gazebo
    spawn_ur = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur, "launch", "ur10.launch.py")),
            launch_arguments={
            "robot_ip": "192.168.0.100",
            "launch_rviz": "false",
            }.items(),
        )
    
    # Include the launch file to spawn the Kinect on Gazebo
    spawn_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aruco, "launch", "aruco_pose_estimation.launch.py")),
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
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
        namespace="/camera/color"
        )
    
    kinect_view = Node(
        package="image_tools",
        executable="showimage",
        name="rgb_showimage",
        parameters=[{"window_name": "RGB"}],
        remappings=[("image", "/camera/color/image_raw")],
        )

    tf_map = Node(
        package='tf2_ros', 
        name="tf_map_world",
        executable='static_transform_publisher', 
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
        )

    tf = Node(package='tf2_ros', 
              name="tf_world_odom",
              executable='static_transform_publisher', 
              output='screen',
              arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
              )
    
    tf_aruco = Node(package='smartfactory_simulation', 
              name="tf_aruco",
              executable='aruco_pose_transformer'
              )
        
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('smartfactory_description'), 'rviz', 'smartfactory.rviz')],
    )

    return LaunchDescription([
        tf_map,
        tf,
        description,
        kinect_node,
        # kinect_view,
        spawn_aruco,
        tf_aruco,
        spawn_ur,
        rviz,
    ])