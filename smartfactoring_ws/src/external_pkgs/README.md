# External Packages

This folder groups the third-party packages vendored into the project's ROS 2 workspace.

The purpose of `external_pkgs/` is to keep the external dependencies required for build, bringup, robot control, perception, and execution inside the same workspace. The project's core logic lives in the in-house packages under `smartfactoring_ws/src/`, while this folder concentrates reusable libraries, drivers, and frameworks.

## Role in the project

The packages in this folder provide the foundation for:

- UR10 control and communication;
- ROS 2 Control infrastructure and controllers;
- motion planning with MoveIt 2;
- ArUco detection and pose estimation;
- RGB-D camera drivers;
- ROS messages and utility packages;
- Behavior Tree support libraries.

In general, these packages are not the main contribution of this repository. They are the external building blocks that support the system architecture.

## Structure

The following directories are currently included:

- `BehaviorTree.ROS2`: ROS 2 integration for Behavior Tree libraries.
- `control_msgs`: messages and actions used by ROS controllers.
- `control_toolbox`: control-related utilities.
- `kinect_ros2`: Kinect driver and ROS 2 support.
- `kinematics_interface`: auxiliary kinematics interfaces.
- `moveit2`: motion planning and manipulation framework.
- `moveit_msgs`: messages used by the MoveIt ecosystem.
- `realtime_tools`: utilities for components with real-time constraints.
- `ros2-aruco-pose-estimation`: ArUco marker detection and pose estimation.
- `ros2_astra_camera`: Astra camera driver.
- `ros2_control`: hardware control framework for ROS 2.
- `ros2_controllers`: ready-to-use controllers for `ros2_control`.
- `srdfdom`: utilities for robot semantic descriptions.
- `Universal_Robots_Client_Library`: low-level communication library for UR robots.
- `Universal_Robots_ROS2_Description`: UR robot description packages for ROS 2.
- `Universal_Robots_ROS2_Driver`: ROS 2 driver for the UR10.
- `ur_msgs`: message definitions specific to the Universal Robots ecosystem.

## Relationship with the in-house packages

The project-specific packages depend on these external components. For example:

- `smart_factory_bringup` uses drivers, launch files, and controllers from the ROS and UR ecosystems.
- `smartfactory_behavior_tree` relies on ROS 2 infrastructure and robot execution interfaces.
- `smart_factory_moveit` depends on MoveIt 2 and its related packages.
- the perception pipelines use the camera and ArUco packages collected in this folder.

## Maintenance guidelines

- Avoid modifying third-party code directly unless it is necessary.
- Prefer recording upstream versions, forks, or source commits in the package README or in the main repository documentation.
- When updating dependencies, validate compatibility with ROS 2 Jazzy and with the in-house workspace packages.

## Note

This folder documents the vendored external dependencies used by the project. The overall system architecture, execution flow, and integration between packages should be described in the main repository README and in the documentation of the in-house packages.
