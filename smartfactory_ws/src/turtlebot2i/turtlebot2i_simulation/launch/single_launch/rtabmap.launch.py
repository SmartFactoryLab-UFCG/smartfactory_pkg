# This launch file sets up various ROS 2 nodes for a TurtleBot2i mapping setup.
# It includes nodes for synchronizing RGB-D images, processing point clouds, 
# performing SLAM, and visualizing the results in RViz. The nodes are configured
# based on parameters defined in YAML files, and various launch arguments are 
# used to control the inclusion of optional components.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):  
    # Define paths to configuration files
    rtabmap_odom_config = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'config','sim_rtabmap_odom_params.yaml')
    rtabmap_slam_config = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'config','sim_rtabmap_slam_params.yaml')
    rtabmap_sync_config = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'config','sim_rtabmap_sync_params.yaml')
    rtabmap_util_config = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'config','sim_rtabmap_util_params.yaml')   
    rtabmap_viz_config = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'config','sim_rtabmap_viz_params.yaml')
   
    # Define whether to use simulation time
    sim_time = True
    localization = LaunchConfiguration('localization')
    
    rtabmap_arguments = []
    if localization.perform(context) == 'true':
        rtabmap_arguments.append('Mem/IncrementalMemory:=false')
    else:
        rtabmap_arguments.append('Mem/IncrementalMemory:=true')
        rtabmap_arguments.append('-d')

    return [
        # # Node for RGB-D image synchronization
        # Node(package='tf2_ros', name="tf_odom_to_footprint",executable='static_transform_publisher', output='screen',
        #             arguments=['0', '0', '0', '0', '0', '0','0', 'odom', 'base_footprint']),
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

        TimerAction(
            period=2.0,
            actions=[
                # Node for odometry estimation using RGB-D data
                Node(
                    condition=IfCondition(LaunchConfiguration('rgbd_odom')),
                    package='rtabmap_odom', executable='rgbd_odometry', output='screen',
                    parameters=[rtabmap_odom_config,
                            {
                                'use_sim_time': sim_time,
                                'publish_tf': False
                            }],
                    remappings=[
                        ('/rgb/image', '/color/image_raw'),
                        ('/rgb/camera_info', '/color/camera_info'),
                        ('/depth/image', '/depth/image_raw'),
                        ('/scan', '/lidar_scan'),
                        ('/odom', '/odometry/rgbd')
                    ],
                ),

            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
            # Node for SLAM using RTAB-Map
            Node(
                package='rtabmap_slam', executable='rtabmap', output='screen',
                parameters=[
                    rtabmap_slam_config,
                    {
                        'use_sim_time': sim_time,
                    }],
                remappings=[
                    ('/rgb/image', '/color/image_raw'),
                    ('/rgb/camera_info', '/color/camera_info'),
                    ('/depth/image', '/depth/image_raw'),
                    ('/scan', '/lidar_scan'),
                    ('/odom', '/odometry/rgbd')
                ],
                arguments=rtabmap_arguments
                ),
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
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
                    ('/odom', '/odometry/rgbd')
                ],
                arguments=['-d', os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'config','sim_rgbd_gui.ini')]
            ),
            ]
        ),
        
        TimerAction(
            period=3.0,
            actions=[
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
        ),

        # Node for compressing point clouds
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='point_cloud_transport',
                    executable='republish',
                    name='point_cloud_republisher',
                    arguments=[
                        'in_transport:=raw', 
                        'in:=/cloud_map',
                        'out_transport:=draco'
                    ],
                    remappings=[
                        ('/out/draco', '/cloud_map/compressed'),
                    ]
                ),
            ]
        ),
    ]

def generate_launch_description():
    # Define path to RViz scene file
    rviz_path = os.path.join(get_package_share_directory('turtlebot2i_description'), 'rviz', 'turtlebot2i.rviz')
    

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('rviz_cfg', default_value=rviz_path, 
                              description='Configuration path of rviz2.'),

        DeclareLaunchArgument('rtabmap_viz', default_value='true',  
                              description='Launch RTAB-Map UI (optional).'),

        DeclareLaunchArgument('rviz', default_value='true', 
                              description='Launch RVIZ (optional).'),

        DeclareLaunchArgument('rgbd_odom', default_value='true', 
                              description='Use camera to infer Odometry.'),

        DeclareLaunchArgument('localization', default_value='false',
                              description='Set to true to run in localization mode'),

        # Call the function to setup nodes
        OpaqueFunction(function=launch_setup)
    ])

