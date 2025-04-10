import launch_ros

import launch


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2",
                namespace="/camera/color"
            ),
            launch_ros.actions.Node(
                package="image_tools",
                executable="showimage",
                name="rgb_showimage",
                parameters=[{"window_name": "RGB"}],
                remappings=[("image", "/camera/color/image_raw")],
            ),
            launch_ros.actions.Node(
                package="image_tools",
                executable="showimage",
                name="depth_showimage",
                parameters=[{"window_name": "Depth"}],
                remappings=[("image", "/camera/aligned_depth_to_color/image_raw")],
            ),
        ]
    )
# import launch
# import launch_ros
# import os
# import subprocess
# import subprocess

# def get_num_kinects():
#     """Conta quantos Kinect estão conectados."""
#     try:
#         result = subprocess.run(["lsusb"], capture_output=True, text=True)
#         num_devices = sum(1 for line in result.stdout.split("\n") if "045e:02ae" in line)
#         return num_devices if num_devices > 0 else 1  # Se não detectar, assume pelo menos 1 Kinect
#     except Exception:
#         return 1  # Em caso de erro, assume 1 Kinect


# def generate_launch_description():
#     num_kinects = get_num_kinects()

#     nodes = []
    
#     for i in range(num_kinects):
#         namespace = f"/kinect{i}"

#         # Nó do Kinect
#         nodes.append(
#             launch_ros.actions.Node(
#                 package="kinect_ros2",
#                 executable="kinect_ros2_node",
#                 name=f"kinect_{i}",
#                 namespace=namespace,
#                 parameters=[{"device_id": i}]
#             )
#         )

#         # Exibição da imagem RGB
#         nodes.append(
#             launch_ros.actions.Node(
#                 package="image_tools",
#                 executable="showimage",
#                 name=f"rgb_showimage_{i}",
#                 parameters=[{"window_name": f"RGB Kinect {i}"}],
#                 remappings=[("image", f"{namespace}/image_raw")],
#             )
#         )

#         # Exibição da imagem de profundidade
#         nodes.append(
#             launch_ros.actions.Node(
#                 package="image_tools",
#                 executable="showimage",
#                 name=f"depth_showimage_{i}",
#                 parameters=[{"window_name": f"Depth Kinect {i}"}],
#                 remappings=[("image", f"{namespace}/depth/image_raw")],
#             )
#         )

#     return launch.LaunchDescription(nodes)

