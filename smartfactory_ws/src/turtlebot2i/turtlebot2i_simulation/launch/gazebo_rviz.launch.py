# Launch file to set up and run both Gazebo and RViz for the TurtleBot2i simulation.
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate the turtlebot2i_simulation package
    turtlebot2i = FindPackageShare(package="turtlebot2i_simulation").find("turtlebot2i_simulation")

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot2i,'launch','single_launch','gazebo.launch.py')),
    )

    # Include the RViz launch file
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot2i,'launch','single_launch','rviz.launch.py')),
    )

    return LaunchDescription([
        gazebo_launch,
        rviz_launch
    ])