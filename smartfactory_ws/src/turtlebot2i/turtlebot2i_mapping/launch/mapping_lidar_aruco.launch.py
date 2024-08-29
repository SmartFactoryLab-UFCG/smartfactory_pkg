# This launch file sets up a system for integrating LiDAR data with ArUco markers
# and performing SLAM with RTAB-Map. It includes nodes for publishing ArUco markers,
# performing odometry and SLAM, and visualizing the results in RViz. Additionally,
# it sets up static transformations and compresses point cloud data.
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
        # Node for publishing ArUco markers
        Node(
            package='aruco_ros',
            executable='marker_publisher', 
            name='aruco_marker_publisher',
            parameters=[{
                'marker_size': 0.1,  # Size of the marker in meters
                'dictionary_id': 'DICT_4X4_100' # Dictionary ID for marker detection
            }],
            remappings=[('/image', '/color/image_raw'),
                ('/camera_info', '/color/camera_info')
            ],
            output='screen'
        ),

         # Node for performing SLAM with RTAB-Map
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            parameters=[{
                'RGBD/MarkerDetection': True,
                'RGBD/MarkerType': 1,   # Marker type according to RTAB-Map documentation
                'RGBD/MarkerSize': 0.1  # Size of markers
            }],
            remappings=[('rgb/image', '/color/image_raw'), 
                        ('depth/image', '/depth/image_raw')],
            output='screen'
        ),

        # Node for static transformation between LiDAR and base
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            name='tf_lidar_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link']),

        # Node for odometry if enabled
        Node(
            condition=IfCondition(LaunchConfiguration('rgbd_odom')),
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[rtabmap_odom_config],
            remappings=remappings,
        ),

        # Node for performing SLAM with RTAB-Map
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_slam_config],
            remappings=remappings,
            arguments=['-d']
            ), 

        # Node for processing RGB-D point clouds
        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', output='screen',
            parameters=[rtabmap_util_config],
            remappings=remappings),

        # Node for visualizing RTAB-Map data
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[rtabmap_viz_config],
            remappings=remappings,
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
        # Call the function to setup nodes
        OpaqueFunction(function=launch_setup)
    ])
