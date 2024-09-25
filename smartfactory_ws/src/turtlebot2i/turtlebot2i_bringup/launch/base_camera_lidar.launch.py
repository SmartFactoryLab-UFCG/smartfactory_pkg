# This script launches the base, camera, and lidar components for the TurtleBot2i,
# with options to specify the types of sensors and components to be used.
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    declare_3D_sensor = DeclareLaunchArgument(
            'TURTLEBOT_3D_SENSOR',
            default_value=EnvironmentVariable('TURTLEBOT_3D_SENSOR'),
            description='Type of 3D Sensor'
        )
    
    declare_3D_sensor2 = DeclareLaunchArgument(
            'TURTLEBOT_3D_SENSOR2',
            default_value=EnvironmentVariable('TURTLEBOT_3D_SENSOR2'),
            description='Type of 3D Sensor 2'
        )
    
    declare_arm = DeclareLaunchArgument(
            'TURTLEBOT_ARM',
            default_value=EnvironmentVariable('TURTLEBOT_ARM'),
            description='Type of Arm'
        )
    
    declare_base = DeclareLaunchArgument(
            'TURTLEBOT_BASE',
            default_value=EnvironmentVariable('TURTLEBOT_BASE'),
            description='Type of Base'
        )
    
    declare_battery = DeclareLaunchArgument(
            'TURTLEBOT_BATTERY',
            default_value=EnvironmentVariable('TURTLEBOT_BATTERY'),
            description='Type of Battery'
        )
    
    declare_lidar_sensor = DeclareLaunchArgument(
            'TURTLEBOT_LIDAR_SENSOR',
            default_value=EnvironmentVariable('TURTLEBOT_LIDAR_SENSOR'),
            description='Type of Lidar'
        )
    
    declare_start_base = DeclareLaunchArgument('Start_base', default_value='true', 
                              description='Start the base node.')
    
    declare_start_lidar = DeclareLaunchArgument('Start_lidar', default_value='true', 
                              description='Start the lidar node.')
    
    declare_start_3d = DeclareLaunchArgument('Start_3d', default_value='true', 
                              description='Start the 3d sensor node.')
    
    declare_backup= DeclareLaunchArgument('backup', default_value='false',  
                              description='Launch Mapping Backup Monitor.')

    declare_safety= DeclareLaunchArgument('safety', default_value='false',  
                              description='safety')
    
    def launch_robotxr(context, *args, **kwargs):
        # Define path to package
        turtlebot2i_bringup = FindPackageShare(package="turtlebot2i_bringup").find("turtlebot2i_bringup")

        # Initialize list
        actions = []
        logAction = []

        logAction.append(LogInfo(msg='Extracting environment variables for robot configuration.'))

        type_3D_sensor = LaunchConfiguration('TURTLEBOT_3D_SENSOR').perform(context)
        type_3D_sensor2 = LaunchConfiguration('TURTLEBOT_3D_SENSOR2').perform(context)
        type_arm = LaunchConfiguration('TURTLEBOT_ARM').perform(context)
        type_base = LaunchConfiguration('TURTLEBOT_BASE').perform(context)
        type_battery = LaunchConfiguration('TURTLEBOT_BATTERY').perform(context)
        type_lidar_sensor = LaunchConfiguration('TURTLEBOT_LIDAR_SENSOR').perform(context)

        logAction.append(LogInfo(msg='3D sensor is: {type_3D_sensor}.'))
        logAction.append(LogInfo(msg='3D sensor 2 is: {type_3D_sensor2}.'))
        logAction.append(LogInfo(msg='Arm is: {type_arm}.'))
        logAction.append(LogInfo(msg='Base is: {type_base}'))
        logAction.append(LogInfo(msg='Battery is: {type_battery}'))
        logAction.append(LogInfo(msg='Lidar Sensor is: {type_lidar_sensor}.'))

        # Launch base
        if type_base != 'none':
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(turtlebot2i_bringup, 'launch', 'base.launch.py')]),
                condition=IfCondition(LaunchConfiguration("Start_base").perform(context)),
            ))
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(turtlebot2i_bringup, 'launch', 'kobuki.launch.py')]),
                launch_arguments={'safety': LaunchConfiguration('safety')}.items(),
                condition=IfCondition(LaunchConfiguration('safety'))
            ))
        else:
            logAction.append(LogInfo(msg='Base cannot be none: {type_base}.'))
            return(1)
        
        # Launch lidar sensor
        if type_lidar_sensor != 'none':
            try:
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(turtlebot2i_bringup, 'launch', type_lidar_sensor + '.launch.py')]),
                    condition=IfCondition(LaunchConfiguration("Start_lidar").perform(context)),
                ))
                if type_base == 'kobuki':
                    actions.append(Node(
                            package='tf2_ros', executable='static_transform_publisher', output='screen',
                            name='tf_lidar_to_base',
                            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link']))
            except:
                logAction.append(LogInfo(msg='Undefined lidar variable: {type_lidar_sensor}.'))

        # Launch 3D sensor
        if type_3D_sensor != 'none':
            try:
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(turtlebot2i_bringup, 'launch', type_3D_sensor + '.launch.py')]),
                    condition=IfCondition(LaunchConfiguration("Start_3d").perform(context)),
                ))
            except:
                logAction.append(LogInfo(msg='Undefined 3D sensor variable: {type_3D_sensor}.'))
        
        if type_3D_sensor2 != 'none':
            logAction.append(LogInfo(msg='No launch defined for second 3D sensor: {type_3D_sensor2}'))

        if type_battery != 'none':
            logAction.append(LogInfo(msg='No launch defined for battery: {type_battery}'))
        
        if type_arm != 'none':
            logAction.append(LogInfo(msg='No launch defined for arm: {type_arm}'))

        actions.append(Node(
            package='turtlebot2i_backup',
            condition=IfCondition(LaunchConfiguration("backup")),
            executable='turtlebot2i_monitor',
            name='turtlebot2i_monitor',
            output='screen'
        ))
    
        return actions + logAction
    
    return LaunchDescription([
        # Node(
        #     package='turtlebot2i_mapping',
        #     executable='list_maps_server.py',
        #     name='list_maps_server',
        #     output='screen'
        # ),
        declare_3D_sensor,
        declare_3D_sensor2,
        declare_arm,
        declare_base,
        declare_battery,
        declare_lidar_sensor,
        declare_start_base,
        declare_start_lidar,
        declare_start_3d,
        declare_backup,
        declare_safety,
        OpaqueFunction(function=launch_robotxr)
    ])