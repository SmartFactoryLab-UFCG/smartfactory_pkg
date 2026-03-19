from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    enable_kinect_arg = DeclareLaunchArgument(
        "enable_kinect", default_value="true", description="Habilitar o Kinect"
    )
    enable_ur10_arg = DeclareLaunchArgument(
        "enable_ur10", default_value="true", description="Habilitar o UR10"
    )

    return LaunchDescription([
        enable_kinect_arg,
        enable_ur10_arg,
        Node(
            package='smartfactory_behavior_tree',
            executable='behavior_pick_and_place',
            name='behavior_pick_and_place',
            parameters=[
                {"enable_kinect": LaunchConfiguration("enable_kinect")},
                {"enable_ur10": LaunchConfiguration("enable_ur10")},
            ],
            output='screen',
        )
    ])
