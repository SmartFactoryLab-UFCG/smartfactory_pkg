import xacro
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    
    # Caminho para o arquivo Xacro
    gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    smartfactory_pkg = FindPackageShare('smartfactory_description').find('smartfactory_description')
    urdf = (xacro.process_file(os.path.join(smartfactory_pkg,
                                            "urdf/aruco.urdf.xacro"),
                              ))
    pretty_urdf = urdf.toprettyxml(indent='   ')

    gazebo_models_path = os.path.join(smartfactory_pkg, "meshes")
    install_dir = get_package_prefix("smartfactory_description")
    # Update GAZEBO_MODEL_PATH environment variable
    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = (
            os.environ["GAZEBO_MODEL_PATH"] + ":" + install_dir + "/share" + ":" + gazebo_models_path
        )
    else:
        os.environ["GAZEBO_MODEL_PATH"] = install_dir + "/share" + ":" + gazebo_models_path

    # Update GAZEBO_PLUGIN_PATH environment variable
    if "GAZEBO_PLUGIN_PATH" in os.environ:
        os.environ["GAZEBO_PLUGIN_PATH"] = os.environ["GAZEBO_PLUGIN_PATH"] + ":" + install_dir + "/lib"
    else:
        os.environ["GAZEBO_PLUGIN_PATH"] = install_dir + "/lib"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gazebo.launch.py"),
        )
    )

        # Comando para converter o Xacro em URDF

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="aruco",
        parameters=[{
            "robot_description": pretty_urdf,
            "use_sim_time": True,
            "publish_frequency": 30.0
            }]
        )

    # Nodo para publicar o URDF no Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'aruco/robot_description', '-entity', 'aruco', '-x', '0', '-y', '-1.5', '-z', '0.476'],
        output='screen'
    )

    return LaunchDescription([
        # gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ])
