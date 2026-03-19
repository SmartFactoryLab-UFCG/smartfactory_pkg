from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="smart_factory_moveit")
        .robot_description_semantic(Path("srdf") / "smartfactory_arm.urdf.xacro", {"name": "smartfactory_arm"})
        .to_moveit_configs()
    ).to_dict()

    # Load Gripper Controller Configuration
    # controller_config = Path("config") / "gripper_controllers.yaml"

    # Node for loading the gripper controller configuration
    # load_gripper_controllers = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     output="screen",
    #     arguments=["gripper_controller"],
    # )

    # MTC Demo node
    pick_place_demo = Node(
        package="smart_factory_moveit",
        executable="mtc_ur10_test",
        output="screen",
        parameters=[
            moveit_config,
        ],

    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'gripper_config',
        #     default_value=str(controller_config),
        #     description='Path to the gripper controller YAML configuration file'
        # ),
        # # Load gripper controller parameters
        # Node(
        #     package='ros2_control_node',
        #     executable='ros2_control_node',
        #     output='screen',
        #     parameters=[LaunchConfiguration('gripper_config')],
        # ),
        pick_place_demo,
        static_tf,
        # load_gripper_controllers,
    ])