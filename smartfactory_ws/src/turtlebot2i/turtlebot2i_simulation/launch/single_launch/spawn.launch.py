# Launch file to spawn the TurtleBot2i robot in the Gazebo simulation.
import os
import xacro
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate the necessary packages
    turtlebot2i_description = FindPackageShare(package="turtlebot2i_description").find("turtlebot2i_description")
    kobuki_description = FindPackageShare(package="kobuki_description").find("kobuki_description")
    
    # Process the xacro file to generate the URDF
    urdf = (xacro.process_file(os.path.join(turtlebot2i_description,
                                            "robots/sim_kobuki_interbotix_astra_pro_rplidar.urdf.xacro"),
                              ))
    pretty_urdf = urdf.toprettyxml(indent='   ')

    # Define installation directories and paths for Gazebo models and plugins
    install_dir1 = get_package_prefix("turtlebot2i_description")
    install_dir2 = get_package_prefix("kobuki_description")
    gazebo_models_path1 = os.path.join(turtlebot2i_description, "meshes")
    gazebo_models_path2 = os.path.join(kobuki_description, "meshes")

    # Update GAZEBO_MODEL_PATH environment variable
    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = (
            os.environ["GAZEBO_MODEL_PATH"]
            + ":"
            + install_dir2
            + "/share"
            + ":"
            + gazebo_models_path2
        )
    else:
        os.environ["GAZEBO_MODEL_PATH"] = (
            install_dir2 + "/share" + ":" + gazebo_models_path2
        )

    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = (
            os.environ["GAZEBO_MODEL_PATH"]
            + ":"
            + install_dir1
            + "/share"
            + ":"
            + gazebo_models_path1
        )
    else:
        os.environ["GAZEBO_MODEL_PATH"] = (
            install_dir1 + "/share" + ":" + gazebo_models_path1
        )

    # Update GAZEBO_PLUGIN_PATH environment variable
    if "GAZEBO_PLUGIN_PATH" in os.environ:
        os.environ["GAZEBO_PLUGIN_PATH"] = (
            os.environ["GAZEBO_PLUGIN_PATH"] + ":" + install_dir1 + "/lib"
        )
    else:
        os.environ["GAZEBO_PLUGIN_PATH"] = install_dir1 + "/lib"

    # Node to publish the robot state to the 'robot_description' topic
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="turtlebot",
        parameters=[{
            "robot_description": pretty_urdf,
            "use_sim_time": True,
            "publish_frequency": 30.0
            }]
        )

    # Node to spawn the robot entity in Gazebo
    spawn = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_entity",
            output="screen",
            arguments=[
                "-entity",
                "mobile_base",
                "-topic",
                "/turtlebot/robot_description",
                "-x",
                "0",
                "-y",
                "-1.5"
            ],
        )

    return LaunchDescription([
        robot_state_publisher_node, 
        spawn,
    ])
