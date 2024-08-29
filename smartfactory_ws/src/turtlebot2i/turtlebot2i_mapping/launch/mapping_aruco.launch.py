# This launch file sets up ROS 2 nodes specifically for using Aruco markers 
# with a TurtleBot2i. It configures nodes for image synchronization, Aruco 
# marker detection, SLAM, point cloud processing, visualization, and point 
# cloud compression. The nodes are configured with parameters from YAML files 
# and include optional components based on launch arguments.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):  
    # Define paths to configuration files 
    rtabmap_slam_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_slam_params.yaml')   
    rtabmap_util_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_util_params.yaml')   
    rtabmap_viz_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_viz_params.yaml')  
    aruco_params = os.path.join(get_package_share_directory('ros2_aruco'), 'config', 'aruco_parameters.yaml')

    return [
        # Node for RGB-D image synchronization
        Node(
            package='rtabmap_sync',  
            executable='rgbd_sync', 
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.01,
                'queue_size': 40,
                'qos': 2
            }],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/depth/image', '/depth/image_raw'),
                ('/rgb/camera_info', '/color/camera_info'),
            ]
        ),

        # Node for Aruco marker detection
        Node(
            package='ros2_aruco', executable='aruco_node', output='screen',
            parameters=[aruco_params],
            remappings=[
                ('rgb/image', '/color/image_raw'),
                ('rgb/camera_info', '/color/camera_info')
            ]
        ),

        # Node for SLAM using RTAB-Map
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_slam_config],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/rgb/camera_info', '/color/camera_info'),
                ('/depth/image', '/depth/image_raw'),
                ('/scan', '/lidar_scan'),
            ],
            arguments=['-d']
            ), 

        # Node for processing RGB-D point clouds
        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', output='screen',
            parameters=[rtabmap_util_config],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/depth/image', '/depth/image_raw'),
                ('/rgb/camera_info', '/color/camera_info'),
            ]
        ),

        # Node for visualizing RTAB-Map data
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[rtabmap_viz_config],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/rgb/camera_info', '/color/camera_info'),
                ('/depth/image', '/depth/image_raw'),
                ('/scan', '/lidar_scan'),
            ],
            arguments=['-d', os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rgbd_gui.ini')]
        ),
        
        # Node for visualizing data in RViz
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration("rviz_cfg")],
        ),
        
        # Node for compressing point clouds
        Node(
            package='point_cloud_transport',
            executable='republish',
            name='pointcloud_republisher',
            arguments=[
                'raw', 'in:=/cloud_map',
                'compressed', 'out:=/cloud_map/compressed'
            ],
        ),
    ]
def generate_launch_description():
    # Define path to RViz scene file
    rviz_path = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'rviz', 'real_turtlebot2i.rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('rviz_cfg', default_value=rviz_path, 
                              description='Configuration path of rviz2.'),

        DeclareLaunchArgument('rtabmap_viz', default_value='false',  
                              description='Launch RTAB-Map UI (optional).'),

        DeclareLaunchArgument('rviz', default_value='false', 
                              description='Launch RVIZ (optional).'),

        DeclareLaunchArgument('rgbd_odom', default_value='true', 
                              description='Use camera to infer Odometry.'),
        # Call the function to setup nodes
        OpaqueFunction(function=launch_setup)
    ])