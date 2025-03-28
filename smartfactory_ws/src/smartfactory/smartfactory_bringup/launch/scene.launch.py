
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
    ur = FindPackageShare(package="ur_bringup").find("ur_bringup")
    turtlebot2i_bringup = FindPackageShare(package="turtlebot2i_bringup").find("turtlebot2i_bringup")
    aruco = FindPackageShare(package="aruco_pose_estimation").find("aruco_pose_estimation")

    smartfactory_description = FindPackageShare('smartfactory_description').find('smartfactory_description')
    urdf = (xacro.process_file(os.path.join(smartfactory_description,
                                            "urdf/smart_spawn.urdf.xacro"),
                              ))
    pretty_urdf = urdf.toprettyxml(indent='   ')
        
    aruco_params_file = os.path.join(get_package_share_directory('smartfactory_bringup'),'config','aruco_parameters.yaml')

    with open(aruco_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config_kinect = config["/aruco_node_kinect"]["ros__parameters"]
    config_basler = config["/aruco_node_basler"]["ros__parameters"]

    # Declare arguments for choosing which components to launch
    start_basler_arg = DeclareLaunchArgument(name='start_basler', default_value='true', description='Whether to start the Basler camera')
    start_kinect_arg = DeclareLaunchArgument(name='start_kinect', default_value='true', description='Whether to start the Kinect camera')
    start_ur10_arg = DeclareLaunchArgument(name='start_ur10', default_value='true', description='Whether to start the UR10 robot')

    # declare configuration parameters' pasta config , para o start devices
    kinect_marker_size_arg = DeclareLaunchArgument(name='kinect_marker_size', default_value=str(config_kinect['marker_size']), description='Size of the aruco marker in meters',)
    basler_marker_size_arg = DeclareLaunchArgument(name='basler_marker_size', default_value=str(config_basler['marker_size']), description='Size of the aruco marker in meters',)
    kinect_aruco_dictionary_id_arg = DeclareLaunchArgument(name='kinect_aruco_dictionary_id', default_value=config_kinect['aruco_dictionary_id'], description='ID of the aruco dictionary to use',)
    basler_aruco_dictionary_id_arg = DeclareLaunchArgument(name='basler_aruco_dictionary_id', default_value=config_basler['aruco_dictionary_id'], description='ID of the aruco dictionary to use',)
    kinect_image_topic_arg = DeclareLaunchArgument(name='kinect_image_topic', default_value=config_kinect['image_topic'], description='Name of the image RGB topic to subscribe to',)
    basler_image_topic_arg = DeclareLaunchArgument(name='basler_image_topic', default_value=config_basler['image_topic'], description='Name of the image RGB topic to subscribe to',)
    kinect_use_depth_input_arg = DeclareLaunchArgument(name='kinect_use_depth_input', default_value=str(config_kinect['use_depth_input']), description='Use depth input for pose estimation', choices=['true', 'false', 'True', 'False'])
    basler_use_depth_input_arg = DeclareLaunchArgument(name='basler_use_depth_input', default_value=str(config_basler['use_depth_input']), description='Use depth input for pose estimation', choices=['true', 'false', 'True', 'False'])
    kinect_depth_image_topic_arg = DeclareLaunchArgument(name='kinect_depth_image_topic', default_value=config_kinect['depth_image_topic'], description='Name of the depth image topic to subscribe to',)
    basler_depth_image_topic_arg = DeclareLaunchArgument(name='basler_depth_image_topic', default_value=config_basler['depth_image_topic'], description='Name of the depth image topic to subscribe to',)
    kinect_camera_info_topic_arg = DeclareLaunchArgument(name='kinect_camera_info_topic', default_value=config_kinect['camera_info_topic'], description='Name of the camera info topic to subscribe to',)
    basler_camera_info_topic_arg = DeclareLaunchArgument(name='basler_camera_info_topic', default_value=config_basler['camera_info_topic'], description='Name of the camera info topic to subscribe to',)
    kinect_camera_frame_arg = DeclareLaunchArgument(name='kinect_camera_frame', default_value=config_kinect['camera_frame'], description='Name of the camera frame where the estimated pose will be',)
    basler_camera_frame_arg = DeclareLaunchArgument(name='basler_camera_frame', default_value=config_basler['camera_frame'], description='Name of the camera frame where the estimated pose will be',)
    kinect_detected_markers_topic_arg = DeclareLaunchArgument(name='kinect_detected_markers_topic', default_value=config_kinect['detected_markers_topic'], description='Name of the topic to publish the detected markers messages',)
    basler_detected_markers_topic_arg = DeclareLaunchArgument(name='basler_detected_markers_topic', default_value=config_basler['detected_markers_topic'], description='Name of the topic to publish the detected markers messages',)
    kinect_markers_visualization_topic_arg = DeclareLaunchArgument(name='kinect_markers_visualization_topic', default_value=config_kinect['markers_visualization_topic'], description='Name of the topic to publish the pose array for visualization of the markers',)
    basler_markers_visualization_topic_arg = DeclareLaunchArgument(name='basler_markers_visualization_topic', default_value=config_basler['markers_visualization_topic'], description='Name of the topic to publish the pose array for visualization of the markers',)
    kinect_output_image_topic_arg = DeclareLaunchArgument(name='kinect_output_image_topic', default_value=config_kinect['output_image_topic'], description='Name of the topic to publish the image with the detected markers', )
    basler_output_image_topic_arg = DeclareLaunchArgument(name='basler_output_image_topic', default_value=config_basler['output_image_topic'], description='Name of the topic to publish the image with the detected markers',)

    # Include the launch file to spawn the Kinect on Gazebo
    spawn_ur = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur, "launch", "ur10.launch.py")),
            launch_arguments={
            "robot_ip": "192.168.0.104",
            "launch_rviz": "false",
            }.items(),
            condition = IfCondition(LaunchConfiguration('start_ur10')),
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

    # basler_aruco_node = Node(
    #     condition = IfCondition(LaunchConfiguration('start_basler')),
    #     package='aruco_pose_estimation',
    #     executable='aruco_node.py',
    #     name='basler_aruco',
    #     parameters=[{
    #         "marker_size": LaunchConfiguration('basler_marker_size'),
    #         "aruco_dictionary_id": LaunchConfiguration('basler_aruco_dictionary_id'),
    #         "image_topic": LaunchConfiguration('basler_image_topic'),
    #         "use_depth_input": LaunchConfiguration('basler_use_depth_input'),
    #         "depth_image_topic": LaunchConfiguration('basler_depth_image_topic'),
    #         "camera_info_topic": LaunchConfiguration('basler_camera_info_topic'),
    #         "camera_frame": LaunchConfiguration('basler_camera_frame'),
    #         "detected_markers_topic": LaunchConfiguration('basler_detected_markers_topic'),
    #         "markers_visualization_topic": LaunchConfiguration('basler_markers_visualization_topic'),
    #         "output_image_topic": LaunchConfiguration('basler_output_image_topic'),
    #     }],
    #     output='screen',
    #     emulate_tty=True
    # )

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
        condition = IfCondition(LaunchConfiguration('start_kinect')),
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
        namespace="/camera/color"
        )

    
    # basler_node = Node(
    #     condition = IfCondition(LaunchConfiguration('start_basler')),
    #     package='pylon_ros2_camera_wrapper',
    #     executable='pylon_ros2_camera_wrapper',
    #     name='basler',
    #     output='screen',
    #     respawn=False,
    #     emulate_tty=True,
    #     parameters=[
    #         os.path.join(get_package_share_directory('smartfactory_bringup'), 'config', 'basler_parameters.yaml'),
    #         {
    #             'gige/mtu_size': 1500,
    #             'startup_user_set': 'CurrentSetting',
    #             'enable_status_publisher': True,
    #             'enable_current_params_publisher': True
    #         }
    #     ]
    # )

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
    
    tf_aruco_kinect = Node(
        condition = IfCondition(LaunchConfiguration('start_kinect')),
        package='smartfactory_bringup', 
        name="tf_aruco_kinect",
        executable='kinect_aruco_pose_transformer'
        )
    
    # tf_aruco_basler = Node(
    #     condition = IfCondition(LaunchConfiguration('start_basler')),
    #     package='smartfactory_bringup', 
    #     name="tf_aruco_basler",
    #     executable='basler_aruco_pose_transformer'
    #     )
        
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('smartfactory_description'), 'rviz', 'smartfactory.rviz')],
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
        #start_basler_arg,
        start_kinect_arg,
        start_ur10_arg,

        kinect_marker_size_arg,
        basler_marker_size_arg,
        kinect_aruco_dictionary_id_arg,
        basler_aruco_dictionary_id_arg,
        kinect_image_topic_arg,
        basler_image_topic_arg,
        kinect_use_depth_input_arg,
        basler_use_depth_input_arg,
        kinect_depth_image_topic_arg,
        basler_depth_image_topic_arg,
        kinect_camera_info_topic_arg,
        basler_camera_info_topic_arg,
        kinect_camera_frame_arg,
        basler_camera_frame_arg,
        kinect_detected_markers_topic_arg,
        basler_detected_markers_topic_arg,
        kinect_markers_visualization_topic_arg,
        basler_markers_visualization_topic_arg,
        kinect_output_image_topic_arg,
        basler_output_image_topic_arg,
    
        tf_map,
        tf,
        description,
        kinect_node,
        #basler_node,
        kinect_aruco_node,
        #basler_aruco_node,
        tf_aruco_kinect,
        #tf_aruco_basler,
        spawn_ur,
        rviz,
        aruco_filtered,
        kinematics,
        vacuum_grip_node
    ])