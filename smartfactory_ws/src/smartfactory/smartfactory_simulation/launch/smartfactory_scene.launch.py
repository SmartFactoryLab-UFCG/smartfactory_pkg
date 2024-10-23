
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Locate the turtlebot2i_simulation package
    smartfactory_sim = FindPackageShare(package="smartfactory_simulation").find("smartfactory_simulation")
    gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    turtlebot2i = FindPackageShare(package="turtlebot2i_simulation").find("turtlebot2i_simulation")
    ur = FindPackageShare(package="ur_simulation_gazebo").find("ur_simulation_gazebo")
    
    # Set the path to the SDF model files
    gazebo_models_path = os.path.join(smartfactory_sim, 'models')  
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gazebo.launch.py"),
        )
    )

    # Declare the argument for the Lab. Smart Factory world file
    world = DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                smartfactory_sim, 'world', 'lab_smart_factory.world'), ''],
            description='SDF world file')
    
    # Include the launch file to spawn the TurtleBot2i on Gazebo
    spawn_turtlebot2i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot2i, "launch", 'single_launch',
                         "spawn.launch.py")
        ),
    )

    # Include the launch file to spawn the Kinect on Gazebo
    spawn_ur = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur, "launch",
                         "ur_sim_control.launch.py")
        ),
         # Adiciona os argumentos aqui
        launch_arguments={
            'gazebo_gui':'false',
            'launch_rviz':'false',
            'ur_type':'ur10'
        }.items(),
    )

    # Include the launch file to spawn the Kinect on Gazebo
    spawn_cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(smartfactory_sim, "launch",
                         "spawn_cameras.launch.py")
        ),
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('smartfactory_description'), 'rviz', 'smartfactory.rviz')],
    )

    # Include the launch file to spawn the Kinect on Gazebo
    spawn_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(smartfactory_sim, "launch",
                         "spawn_aruco.launch.py")
        ),
    )
     # Include the launch file to spawn the Kinect on Gazebo
    spawn_aruco_conveyor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(smartfactory_sim, "launch",
                         "spawn_aruco_conveyor.launch.py")
        ),
    )

    tf_map = Node(package='tf2_ros', name="tf_map_world",executable='static_transform_publisher', output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'])

    tf = Node(package='tf2_ros', name="tf_world_odom",executable='static_transform_publisher', output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'])

    return LaunchDescription([
        world,
        gazebo,
        tf_map,
        tf,
        spawn_cameras,
        spawn_ur,
        spawn_turtlebot2i,
        rviz,
        spawn_aruco,
        spawn_aruco_conveyor
    ])