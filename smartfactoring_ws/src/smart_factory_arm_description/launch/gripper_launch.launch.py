from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(_):

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    description_launchfile = LaunchConfiguration("description_launchfile")

    joint_state_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "robot_ip": robot_ip,
            "ur_type": ur_type,
        }.items(),
    )

    return [joint_state_gui, rsp]

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur15",
                "ur20",
                "ur30",
            ],
            default_value="ur10",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached.",
            default_value="0.0.0.0"
        )
    )

    declared_arguments.append(
            DeclareLaunchArgument(
                "description_launchfile",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"]
                ),
                description="Launchfile (absolute path) providing the description. "
                "The launchfile has to start a robot_state_publisher node that "
                "publishes the description topic.",
            )
        )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("smart_factory_arm_description"),
                 "urdf", "smart_factory_arm.urdf.xacro"]
            ),
            description="URDF/XACRO description file with the robot.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
