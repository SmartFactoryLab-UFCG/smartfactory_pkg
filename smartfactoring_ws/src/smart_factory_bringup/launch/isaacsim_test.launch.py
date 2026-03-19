import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the turtlebot2i_simulation package
   
    aruco = FindPackageShare(package="aruco_pose_estimation").find("aruco_pose_estimation")
        
    aruco_params_file = os.path.join(get_package_share_directory('smart_factory_bringup'),'param','aruco_params.yaml')

    with open(aruco_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config_kinect = config["/aruco_node_kinect"]["ros__parameters"]

    # Declare arguments for choosing which components to launch
    start_kinect_arg = DeclareLaunchArgument(name='start_kinect', default_value='true', description='Whether to start the Kinect camera')

    # declare configuration parameters' pasta config , para o start devices
    kinect_marker_size_arg = DeclareLaunchArgument(name='kinect_marker_size', default_value=str(config_kinect['marker_size']), description='Size of the aruco marker in meters',)
    kinect_aruco_dictionary_id_arg = DeclareLaunchArgument(name='kinect_aruco_dictionary_id', default_value=config_kinect['aruco_dictionary_id'], description='ID of the aruco dictionary to use',)
    kinect_image_topic_arg = DeclareLaunchArgument(name='kinect_image_topic', default_value=config_kinect['image_topic'], description='Name of the image RGB topic to subscribe to',)
    kinect_use_depth_input_arg = DeclareLaunchArgument(name='kinect_use_depth_input', default_value=str(config_kinect['use_depth_input']), description='Use depth input for pose estimation', choices=['true', 'false', 'True', 'False'])
    kinect_depth_image_topic_arg = DeclareLaunchArgument(name='kinect_depth_image_topic', default_value=config_kinect['depth_image_topic'], description='Name of the depth image topic to subscribe to',)
    kinect_camera_info_topic_arg = DeclareLaunchArgument(name='kinect_camera_info_topic', default_value=config_kinect['camera_info_topic'], description='Name of the camera info topic to subscribe to',)
    kinect_camera_frame_arg = DeclareLaunchArgument(name='kinect_camera_frame', default_value=config_kinect['camera_frame'], description='Name of the camera frame where the estimated pose will be',)
    kinect_detected_markers_topic_arg = DeclareLaunchArgument(name='kinect_detected_markers_topic', default_value=config_kinect['detected_markers_topic'], description='Name of the topic to publish the detected markers messages',)
    kinect_markers_visualization_topic_arg = DeclareLaunchArgument(name='kinect_markers_visualization_topic', default_value=config_kinect['markers_visualization_topic'], description='Name of the topic to publish the pose array for visualization of the markers',)
    kinect_output_image_topic_arg = DeclareLaunchArgument(name='kinect_output_image_topic', default_value=config_kinect['output_image_topic'], description='Name of the topic to publish the image with the detected markers', )

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

    kinect_node = Node(
        condition = IfCondition(LaunchConfiguration('start_kinect')),
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
        namespace="/camera/color"
        )

    tf_map = Node(
        package='tf2_ros', 
        name="tf_map_world",
        executable='static_transform_publisher', 
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
        )

    
    tf_aruco_kinect = Node(
        condition = IfCondition(LaunchConfiguration('start_kinect')),
        package='smart_factory_bringup', 
        name="tf_aruco_kinect",
        executable='kinect_aruco_pose_transformer'
        )
    
        
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('smart_factory_arm_description'), 'rviz', 'smartfactory.rviz')],
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
        start_kinect_arg,

        kinect_marker_size_arg,
        kinect_aruco_dictionary_id_arg,
        kinect_image_topic_arg,
        kinect_use_depth_input_arg,
        kinect_depth_image_topic_arg,
        kinect_camera_info_topic_arg,
        kinect_camera_frame_arg,
        kinect_detected_markers_topic_arg,
        kinect_markers_visualization_topic_arg,
        kinect_output_image_topic_arg,
    
        tf_map,
        #kinect_node,
        #kinect_aruco_node,
        tf_aruco_kinect,
        rviz,
        aruco_filtered,
        kinematics,
        #vacuum_grip_node
    ])