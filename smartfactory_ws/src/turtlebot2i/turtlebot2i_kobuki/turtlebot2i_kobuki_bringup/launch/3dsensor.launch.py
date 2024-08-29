import os
from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from typing import Text
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> Text:
        if self.condition == True or self.condition == 'true' or self.condition == 'True':
            return self.text_if
        else:
            return self.text_else
            
class ConditionalBool(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> bool:
        if self.condition:
            return self.text_if
        else:
            return self.text_else
            
def launch_setup(context, *args, **kwargs):
    turtlebot2i = FindPackageShare(package="turtlebot2i").find("turtlebot2i")    
    param_config = os.path.join(turtlebot2i, 'params', 'depth_params_config.yaml')
               
    return [
        # DeclareLaunchArgument('depth', default_value=ConditionalText('depth_registered', 'depth', IfCondition(PythonExpression(["'", LaunchConfiguration('depth_registration'), "' == 'true'"]))._predicate_func(context)), description=''),  
        SetParameter(name='use_sim_time', value='true'),
        # 'use_sim_time' will be set on all nodes following the line above

        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=["0", "0", "0", "1.57", "0", "0", "camera_depth_optical_frame", "camera_rgb_frame"]),

    #     Node(
    #         package='depthimage_to_laserscan',
    #         executable='depthimage_to_laserscan_node',
    #         name='depthimage_to_laserscan',
    #         condition=IfCondition(LaunchConfiguration('scan_processing')),
    #         parameters=[param_config],
    #         remappings=[('depth', '/camera/depth_registered/image_rect_raw'),
    #                     ('depth_camera_info', '/camera/depth_registered/camera_info')]),
    ]
        
        
def generate_launch_description():
    
    sensor_3d_launch = os.path.join(
        get_package_share_directory('turtlebot2i'),
        'launch', 'robotxr_astra_camera.launch.py',
    )

    return LaunchDescription([

        DeclareLaunchArgument('camera', default_value='camera', description='Unique identifier for the device'),
        DeclareLaunchArgument('publish_tf', default_value='false', description='Whether to publish TF or not'),
        DeclareLaunchArgument('3d_sensor', default_value='astra', description='3D sensor type (kinect, asus_xtion_pro)'),
        
        DeclareLaunchArgument('depth_registration', default_value='true', description='Whether depth registration is enabled or not'),
        
        DeclareLaunchArgument('rgb_processing', default_value='true', description='Whether RGB processing is enabled or not'),
        DeclareLaunchArgument('ir_processing', default_value='true', description='Whether IR processing is enabled or not'),
        DeclareLaunchArgument('depth_processing', default_value='true', description='Whether depth processing is enabled or not'),
        DeclareLaunchArgument('depth_registered_processing', default_value='true', description='Whether depth registered processing is enabled or not'),
        DeclareLaunchArgument('disparity_processing', default_value='true', description='Whether disparity processing is enabled or not'),
        DeclareLaunchArgument('disparity_registered_processing', default_value='true', description='Whether disparity registered processing is enabled or not'),
        DeclareLaunchArgument('scan_processing', default_value='true', description='Whether scan processing is enabled or not'),
        DeclareLaunchArgument('num_worker_threads', default_value='4', description='Number of worker threads for the nodelet manager'),
        DeclareLaunchArgument('scan_topic', default_value='scan', description='Scan topic'),
        
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensor_3d_launch),
                condition=UnlessCondition(LaunchConfiguration('scan_processing')),
                launch_arguments={
                    'camera': LaunchConfiguration('camera'),
                    'publish_tf': LaunchConfiguration('publish_tf'),
                    'depth_registration': LaunchConfiguration('depth_registration'),
                    'num_worker_threads': LaunchConfiguration('num_worker_threads'),
                    'rgb_processing': LaunchConfiguration('rgb_processing'),
                    'ir_processing': LaunchConfiguration('ir_processing'),
                    'depth_processing': LaunchConfiguration('depth_processing'),
                    'depth_registered_processing': LaunchConfiguration('depth_registered_processing'),
                    'disparity_processing': LaunchConfiguration('disparity_processing'),
                    'disparity_registered_processing': LaunchConfiguration('disparity_registered_processing')
                }.items(),
            ),

      OpaqueFunction(function=launch_setup)
    ])
