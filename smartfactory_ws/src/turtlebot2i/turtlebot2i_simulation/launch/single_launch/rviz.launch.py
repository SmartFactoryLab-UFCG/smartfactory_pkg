# Launch file to set up and run RViz for visualizing the TurtleBot2i simulation.
import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the RViz scene file
    rviz_path = os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'rviz', 'turtlebot2i.rviz')

    # Declare a launch argument for the RViz scene file
    rviz_scene_arg = DeclareLaunchArgument('rviz_cfg', default_value=rviz_path, description='RViz scene file path.')

    # Node to launch RViz with the specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        )
    
    # Node to set up a static transform between the 'map' and 'odom' frames
    tf = Node(
        package='tf2_ros',
        name='tf_map_odom',
        executable='static_transform_publisher', 
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    return LaunchDescription([
        rviz_scene_arg,
        rviz_node,
        tf
    ])
