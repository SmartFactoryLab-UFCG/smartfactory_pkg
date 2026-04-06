# Smart Factory Bringup

This package contains the launch files and runtime configuration used to bring up the main Smart Factory robotic cell scenario.

Its role is to start the core runtime stack around the UR10 manipulator, the Kinect and Astra cameras, the ArUco detection pipeline, pose processing nodes, and RViz-based visualization.

## Purpose

`smart_factory_bringup` provides the entry point for the project runtime environment. In practice, it is the package used to launch the integrated scene that combines:

- the UR10 driver;
- the custom robot description;
- camera descriptions and TF publishers;
- Kinect and Astra sensor drivers;
- ArUco detection nodes;
- pose filtering and kinematics utilities;
- vacuum gripper state monitoring;
- RViz visualization.

## Main launch file

The main entry point is:

`launch/smart_factory_scene.launch.py`

This launch file is intended to start the full application scene used by the project.

## How to run

### 1. Build the workspace

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential
source install/setup.bash
```

### 2. Launch the main scene

```bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py
```

## Launch arguments

The main launch file exposes the following boolean arguments, all enabled by default:

- `rviz:=true|false`
- `kinect:=true|false`
- `astra:=true|false`
- `ur10:=true|false`

Examples:

```bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py rviz:=false
ros2 launch smart_factory_bringup smart_factory_scene.launch.py ur10:=false
ros2 launch smart_factory_bringup smart_factory_scene.launch.py kinect:=true astra:=false
```

## What the main bringup starts

The current `smart_factory_scene.launch.py` starts the following components.

### UR10 stack

When `ur10:=true`, the launch file:

- includes `ur_robot_driver/launch/ur10.launch.py`;
- uses `robot_ip:=<UR10_IP_ADRESS>`;
- disables the driver's internal RViz instance;
- starts a dedicated `robot_state_publisher` in the `custom_ur10` namespace using `smart_factory_arm_description/urdf/smart_factory_arm.urdf`.

## Camera description and TF

The bringup also starts:

- a `robot_state_publisher` in the `cameras` namespace using `smartfactory_description/urdf/smart_spawn.urdf.xacro`;
- a static transform publisher for `map -> world` with identity transform.

## Astra pipeline

When `astra:=true`, the launch file:

- includes `launch/astra_pro_plus.launch.py`;
- starts the Astra camera node using `param/astra_pro_params.yaml`;
- starts `aruco_pose_estimation/aruco_node.py` as `astra_aruco`;
- loads ArUco parameters from `param/aruco_params.yaml`, section `/aruco_node_astra`.

The Astra ArUco pipeline is configured around:

- image topic: `/color/image_raw`
- camera info topic: `/color/camera_info`
- output markers topic: `astra/aruco/markers`

## Kinect pipeline

When `kinect:=true`, the launch file:

- starts `kinect_ros2/kinect_ros2_node` in the `/kinect` namespace;
- starts `aruco_pose_estimation/aruco_node.py` as `kinect_aruco`;
- loads ArUco parameters from `param/aruco_params.yaml`, section `/aruco_node_kinect`;
- starts `smart_factory_bringup/kinect_aruco_pose_transformer`.

The Kinect ArUco pipeline is configured around:

- image topic: `/kinect/image_raw`
- camera info topic: `/kinect/camera_info`
- output markers topic: `kinect/aruco/markers`

## Auxiliary processing nodes

The bringup also starts the following project-specific runtime nodes:

- `smartfactory_aruco_poses/filtered_pose`
  Used for pose filtering and stabilization.
- `smartfactory_ur_utils/calculate_kinematics`
  Computes UR10 joint targets from perception results.
- `smartfactory_ur_utils/vacuum_grip_detect`
  Publishes the vacuum gripper state used by higher-level behaviors.

## RViz

When `rviz:=true`, the bringup opens:

- `rviz2` with `rviz/smart_factory_scene.rviz`

## Other launch files in this package

- `launch/ur10_gripper.launch.py`
  Starts the UR10 control stack through `ur_control.launch.py` and supports `use_mock_hardware`.
- `launch/astra_pro_plus.launch.py`
  Starts the Astra camera driver using the package parameter file.
- `launch/isaacsim_test.launch.py`
  Auxiliary launch file for Isaac Sim-related tests.

## Configuration files

Important configuration files in this package include:

- `param/aruco_params.yaml`
  Defines ArUco detector parameters, camera topics, frame IDs, and output topics for Kinect and Astra.
- `param/astra_pro_params.yaml`
  Defines Astra camera runtime parameters.
- `rviz/smart_factory_scene.rviz`
  RViz configuration used by the bringup.

## Notes

- The robot IP used by the main scene launch is currently hardcoded in `smart_factory_scene.launch.py`.
- This package is focused on system startup and orchestration, not on task logic. High-level execution behavior is handled by packages such as `smartfactory_behavior_tree`.
- The bringup depends on several in-house and third-party packages being available in the workspace, including `smart_factory_arm_description`, `smartfactory_description`, `smartfactory_aruco_poses`, `smartfactory_ur_utils`, `ur_robot_driver`, `kinect_ros2`, `astra_camera`, and `aruco_pose_estimation`.
