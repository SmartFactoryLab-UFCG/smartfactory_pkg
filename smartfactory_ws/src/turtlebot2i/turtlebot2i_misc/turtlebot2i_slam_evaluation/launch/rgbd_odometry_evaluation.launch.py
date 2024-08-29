# This launch file sets up and runs the RGB-D odometry evaluation for the TurtleBot2i.
# It launches nodes for odometry computation and error evaluation, and optionally RViz for visualization.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):  

    # Path to the RTAB-Map odometry configuration file
    rtabmap_odom_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_odom_params.yaml')

    # Configure the use of simulation time
    sim_time = LaunchConfiguration('simtime')
    return [
        # Node for RGB-D odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{
                'Reg/Force3DoF': 'False',  # Set odometry to 3 DoF
                'use_sim_time': sim_time,  # Use simulation time
                'approx_sync': True,  # Enable approximate synchronization
                'approx_sync_max_interval': 0.0,  # Maximum interval for approximate synchronization
                'frame_id': 'kinect',  # Frame ID
                'ground_truth_base_frame_id': 'kinect',  # Ground truth base frame ID
                'ground_truth_frame_id': 'world',  # Ground truth frame ID
                'wait_for_transform': 0.1,  # Wait time for transform
                'Odom/Strategy': "0",  # Odometry strategy
                'Odom/ResetCountdown': "0",  # Reset countdown for odometry
                'Odom/GuessSmoothingDelay': "0",  # Guess smoothing delay for odometry
                'qos': 0,  # Quality of Service setting
                'qos_camera_info': 2  # Quality of Service setting for camera info
            }],
            remappings=[
                ('/rgb/image', '/camera/rgb/image_color'),
                ('/rgb/camera_info', '/camera/rgb/camera_info'),
                ('/depth/image', '/camera/depth/image'),
                ('/odom', '/odom_camera')
            ],
        ),

         # Node for odometry error evaluation
        Node(
            package='turtlebot2i_slam_evaluation', executable='odometry_error', output='screen',
            parameters=[{
                'use_sim_time': sim_time,  # Use simulation time
                'odometry_topic': '/odom_camera',  # Odometry topic
                'ground_truth_frame_parent': 'world',  # Ground truth frame parent
                'ground_truth_frame_child': 'kinect',  # Ground truth frame child
                '3DoF': False,  # Set to 3 DoF
            }],
        ),

        # Node for visualizing data in RViz
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            parameters=[{
                'use_sim_time': sim_time
            }],
            arguments=['-d', LaunchConfiguration("rviz_cfg")],
        ),

    ]

# Function to setup the recording based on the condition
def setup_recording(context, *args, **kwargs):
    if context.launch_configurations['record_data'] == 'true':
        return [
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '/odom_camera', '/odom_euclidian_error', '/tf'],
                output='screen'
            )
        ]
    return []

# Function to setup the playback based on the condition
def setup_playback(context, *args, **kwargs):
    if context.launch_configurations['play_bag'] == 'true' and context.launch_configurations['bag_file']:
        return [
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', context.launch_configurations['bag_file'], '--clock'],
                output='screen'
            )
        ]
    return []

def generate_launch_description():
    # Define path to RViz scene file
    rviz_path = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'rviz', 'real_turtlebot2i.rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('rviz_cfg', default_value=rviz_path, 
                              description='Configuration path of rviz2.'),

        DeclareLaunchArgument('rviz', default_value='false', 
                              description='Launch RVIZ (optional).'),

        DeclareLaunchArgument('simtime', default_value='true', 
                              description='Use Simtime.'),
                              
        DeclareLaunchArgument('record_data', default_value='false', 
                              description='Save a rosbag with results.'),

        DeclareLaunchArgument('play_bag', default_value='false', 
                              description='Condition to play bag file.'),

        DeclareLaunchArgument('bag_file', default_value='', 
                              description='Path of the bag file.'),
        
        # Conditionally record the data
        OpaqueFunction(function=setup_recording),
        
        # Conditionally play the bag file
        OpaqueFunction(function=setup_playback),

        OpaqueFunction(function=launch_setup)      
    ])