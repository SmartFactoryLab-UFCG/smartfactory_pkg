# This launch file sets up nodes for a TurtleBot2i robot that uses LIDAR and RGB-D sensors.
# It configures the nodes for RTAB-Map SLAM, odometry, visualization, and point cloud processing.
# The nodes include RTAB-Map for SLAM and point cloud processing, RViz for visualization, 
# and a point cloud transport node to compress the point cloud data.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):  
    # Define configuration file paths
    rtabmap_odom_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_odom_params.yaml')
    rtabmap_slam_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_slam_params.yaml')   
    rtabmap_util_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_util_params.yaml')   
    rtabmap_viz_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_viz_params.yaml')  

    # Remapping topics for consistency
    remappings=[
        ('rgb/image', '/color/image_raw'),
        ('rgb/camera_info', '/color/camera_info'),
        ('depth/image', '/depth/image_raw'),
        ('scan', '/lidar_scan'),
        ('odom', '/odom_camera')]

    return [
        # Node for RGB-D odometry
        Node(
            condition=IfCondition(LaunchConfiguration('rgbd_odom')),
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[rtabmap_odom_config],
            remappings=remappings,
        ),

        # Node for RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_slam_config],
            remappings=remappings,
            arguments=['-d']
            ), 

        # Node for processing and assembling point clouds
        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', output='screen',
            parameters=[rtabmap_util_config],
            remappings=remappings),

        # Node for RTAB-Map visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[rtabmap_viz_config],
            remappings=remappings,
            arguments=['-d', os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rgbd_gui.ini')]
        ),
        
        # Node for running RViz visualization
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration("rviz_cfg")],
        ),
        
        # Node for compressing point cloud data
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

