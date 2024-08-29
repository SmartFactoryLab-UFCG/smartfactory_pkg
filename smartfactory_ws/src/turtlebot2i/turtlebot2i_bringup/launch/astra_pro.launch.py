# Launch file to start the Astra Pro camera node and image republisher for TurtleBot2i
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():  
    # Load camera parameters from configuration file
    camera_config = os.path.join(
        get_package_share_directory('turtlebot2i_bringup'),
        'config',
        'astra_pro_params.yaml'
    )
    
    # Define the Astra camera node
    camera_node = Node(
        package='astra_camera',
        executable='astra_camera_node',
        name='camera_node',
        parameters=[{camera_config}],
        output='screen'
    )

    # Define the image republisher node for compressing images
    compressed_image= Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',
            arguments=[
                'raw', 'in:=color/image_raw',
                'compressed', 'out:=/color/compressed'
            ],
            remappings=[
                ('/out/compressed', '/color/compressed')
            ]
        )
    
    # Return the launch description containing both nodes
    return LaunchDescription([
        camera_node,
        compressed_image
    ])
