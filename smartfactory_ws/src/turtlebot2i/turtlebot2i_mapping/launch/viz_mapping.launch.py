# This launch file sets up nodes for visualizing the TurtleBot2i robot's mapping data.
# It launches RTAB-Map visualization and RViz with the specified configurations.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):
    # Define configuration file path for RTAB-Map visualization
    rtabmap_viz_config = os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rtabmap_viz_params.yaml')  
        
    return [
        # Node for RTAB-Map visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[rtabmap_viz_config],
            arguments=['-d', os.path.join(get_package_share_directory('turtlebot2i_mapping'), 'config','rgbd_gui.ini')]
        ),

        # Node for running RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration("rviz_cfg")],
        ),

    ]
def generate_launch_description():
    # Define path to RViz scene file
    rviz_path = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'rviz', 'real_turtlebot2i.rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('rviz_cfg', default_value=rviz_path, 
                              description='Configuration path of rviz2.'),
        # Call the function to setup nodes
        OpaqueFunction(function=launch_setup)
    ])

