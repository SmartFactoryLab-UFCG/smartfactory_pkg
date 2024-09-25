# Launch file to set up and run Gazebo simulation environment for TurtleBot2i.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the packages
    turtlebot2i = FindPackageShare(package="turtlebot2i_simulation").find("turtlebot2i_simulation")
    gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")

    # Set the path to the SDF model files
    gazebo_models_path = os.path.join(turtlebot2i, 'models')  
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gazebo.launch.py"),
        )
    )

    # Include the launch file to spawn the TurtleBot2i on Gazebo
    spawn_turtlebot2i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot2i, "launch", 'single_launch',
                         "spawn.launch.py")
        ),
    )

    # Declare the argument for the Lab. Smart Factory world file
    world = DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                turtlebot2i, 'worlds', 'lab_smart_factory.world'), ''],
            description='SDF world file')
    
    # Set up static transforms
    # Base to Left Wheel
    transf1 = Node(package='tf2_ros', name = 'tf_base_lwheel', executable='static_transform_publisher', output='screen',
                    arguments=['0', '0', '0', '0', '0', '0','base_link_kobuki', 'wheel_left_link'])
    # Base to Right Wheel
    transf2 = Node(package='tf2_ros',name='tf_base_rwheel' ,executable='static_transform_publisher', output='screen',
                    arguments=['0', '0', '0', '0', '0', '0', 'base_link_kobuki', 'wheel_right_link'])
    
    return LaunchDescription([
        world,
        gazebo,
        transf1,
        transf2,
        spawn_turtlebot2i,
    ])