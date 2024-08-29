# This launch file sets up various ROS 2 nodes for a TurtleBot2i mapping setup.
# It includes nodes for synchronizing RGB-D images, processing point clouds, 
# performing SLAM, and visualizing the results in RViz. The nodes are configured
# based on parameters defined in YAML files, and various launch arguments are 
# used to control the inclusion of optional components.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):  
    # Define paths to configuration files
    rtabmap_odom_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_odom_params.yaml')
    rtabmap_slam_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_slam_params.yaml')
    rtabmap_sync_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_sync_params.yaml')
    rtabmap_util_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_util_params.yaml')   
    rtabmap_viz_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_viz_params.yaml')
    aruco_params = os.path.join(get_package_share_directory('ros2_aruco'), 'config', 'aruco_parameters.yaml')

    # Define whether to use simulation time
    sim_time = LaunchConfiguration('rosbag')

    return [
        # Node for RGB-D image synchronization
        Node(
            package='rtabmap_sync', 
            executable='rgbd_sync',  
            name='rgbd_sync',
            output='screen',
            parameters=[rtabmap_sync_config,
                {
                    'use_sim_time': sim_time
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
            condition=IfCondition(LaunchConfiguration("aruco")),
            parameters=[aruco_params,
                {
                    'use_sim_time': sim_time
                }],
            remappings=[
                ('rgb/image', '/color/image_raw'),
                ('rgb/camera_info', '/color/camera_info')
            ]
        ),

        # Node for assembling point clouds
        Node(
            package='rtabmap_util',
            executable='point_cloud_assembler',
            name='point_cloud_assembler',
            output='screen',
            parameters=[rtabmap_util_config,
                {
                    'use_sim_time': sim_time
                }]
        ),

        # Node for odometry estimation using RGB-D data
        Node(
            condition=IfCondition(LaunchConfiguration('rgbd_odom')),
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[rtabmap_odom_config],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/rgb/camera_info', '/color/camera_info'),
                ('/depth/image', '/depth/image_raw'),
                ('/scan', '/lidar_scan'),
                ('/odom', '/odom_camera')
            ],
        ),

        # Node for SLAM using RTAB-Map
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[
                rtabmap_slam_config,
                {
                    'use_sim_time': sim_time
                }],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/rgb/camera_info', '/color/camera_info'),
                ('/depth/image', '/depth/image_raw'),
                ('/scan', '/lidar_scan')
            ],
            arguments=['-d']
        ), 

        # Node for processing RGB-D point clouds
        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', output='screen',
            parameters=[rtabmap_util_config,
                {
                    'use_sim_time': sim_time
                }],
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
            parameters=[rtabmap_viz_config,
                {
                    'use_sim_time': sim_time
                }],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/rgb/camera_info', '/color/camera_info'),
                ('/depth/image', '/depth/image_raw'),
                ('/scan', '/lidar_scan'),
                ('/odom', '/odom_camera')
            ],
            arguments=['-d', os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rgbd_gui.ini')]
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

        # Node for compressing point clouds
        Node(
            package='point_cloud_transport',
            executable='republish',
            name='pointcloud_republisher',
            arguments=[
                'raw', 'in:=/cloud_map',
                'compressed', 'out:=/cloud_map/compressed'
            ],
        )

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

        DeclareLaunchArgument('aruco', default_value='true', 
                              description='Launch Aruco.'),

        DeclareLaunchArgument('rosbag', default_value='false', 
                              description='Launch Rosbag.'),

        DeclareLaunchArgument('rgbd_odom', default_value='false', 
                              description='Use camera to infer Odometry.'),
        # Call the function to setup nodes
        OpaqueFunction(function=launch_setup)
    ])

