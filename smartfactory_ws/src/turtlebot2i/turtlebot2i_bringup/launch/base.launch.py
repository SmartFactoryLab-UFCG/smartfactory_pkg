# This script configures and launches the base components of the TurtleBot2i, including
# the robot description, RViz visualization, and specific sensor configurations.
import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Execute RViz simulation.')
    declare_3D_sensor = DeclareLaunchArgument('TURTLEBOT_3D_SENSOR', default_value=EnvironmentVariable('TURTLEBOT_3D_SENSOR'), description= 'Type of 3D Sensor')
    declare_base = DeclareLaunchArgument('TURTLEBOT_BASE', default_value=EnvironmentVariable('TURTLEBOT_BASE'), description= 'Type of Base')
    declare_lidar_sensor = DeclareLaunchArgument('TURTLEBOT_LIDAR_SENSOR', default_value=EnvironmentVariable('TURTLEBOT_LIDAR_SENSOR'), description= 'Type of Lidar')
    declare_stacks = DeclareLaunchArgument('TURTLEBOT_STACKS', default_value=EnvironmentVariable('TURTLEBOT_STACKS'), description= 'Type of Stacks')
    
    def launch_base(context, *args, **kwargs):
        # Initialize list of actions
        actions = []

        # Retrieve configurations
        type_3D_sensor = LaunchConfiguration('TURTLEBOT_3D_SENSOR').perform(context)
        type_base = LaunchConfiguration('TURTLEBOT_BASE').perform(context)
        type_lidar_sensor = LaunchConfiguration('TURTLEBOT_LIDAR_SENSOR').perform(context)
        type_stacks = LaunchConfiguration('TURTLEBOT_STACKS').perform(context)

        # Define paths
        turtlebot2i = FindPackageShare(package="turtlebot2i_description").find("turtlebot2i_description")
        turtlebot2i_bringup = FindPackageShare(package="turtlebot2i_bringup").find("turtlebot2i_bringup")
        rviz_path = os.path.join(turtlebot2i, 'rviz', 'turtlebot2i.rviz')

        # Generate URDF
        lidar_name = 'rplidar' if 'rplidar' in type_lidar_sensor else type_lidar_sensor
        urdf = xacro.process_file(os.path.join(turtlebot2i, 'robots', f'{type_base}_{type_stacks}_{type_3D_sensor}_{lidar_name}.urdf.xacro'))
        pretty_urdf = urdf.toprettyxml(indent='   ')
        
        # Node to publish robot state
        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": pretty_urdf,
                "use_sim_time": False,
                "publish_frequency": 30.0
                }]
        ))

        # Initialize RViz
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('turtlebot2i_simulation'), 'launch','single_launch','rviz.launch.py')),
            condition=IfCondition(LaunchConfiguration("rviz")),
            launch_arguments={'rviz_scene': rviz_path}.items()
        ))

        # Start base
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(turtlebot2i_bringup, 'launch', f'{type_base}.launch.py')),
        ))

        return actions

    return LaunchDescription([
        rviz_arg,
        declare_3D_sensor,
        declare_base,
        declare_lidar_sensor,
        declare_stacks,
        OpaqueFunction(function=launch_base)
    ])
