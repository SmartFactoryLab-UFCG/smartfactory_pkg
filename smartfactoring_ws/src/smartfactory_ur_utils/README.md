# Smart Factory Utils

This package provides utility nodes and helper modules for UR10 motion execution, kinematics, and vacuum gripper interaction in the Smart Factory workspace.

Its role is to bridge perception outputs and low-level robot interfaces by converting detected object poses into joint targets, forwarding commands to the robot controller, and exposing suction-related signals used by higher-level behaviors.

## Purpose

`smartfactory_ur_utils` contains the UR10-specific support utilities used across the project.

In practice, it is responsible for:

- computing inverse kinematics targets from filtered ArUco poses;
- publishing joint targets for the UR10;
- sending predefined trajectories to the robot controller;
- switching the vacuum gripper on and off through UR IO services;
- converting tool data into a simple gripper-status signal;
- providing shared transformation utilities used by the kinematics code.

## Package type

This is an `ament_python` package that exposes the following console executables:

- `SM_Transformations`
- `calculate_kinematics`
- `send_angles`
- `ventosa_on`
- `ventosa_off`
- `send_conveyor`
- `vacuum_grip_detect`

## Main runtime nodes

### `calculate_kinematics`

This is the main kinematics node used by the bringup pipeline.

It subscribes to:

- `/joint_states`
- `/kinect/aruco/filtered_poses`

and publishes:

- `/ur10/calculated_joint_angles`

The node uses a UR10 kinematic model and internal transformation utilities to estimate inverse-kinematics solutions for the target pose derived from the filtered Kinect ArUco pose.

In the current implementation:

- the first pose in `/kinect/aruco/filtered_poses` is used as the target;
- the target position is adjusted with project-specific offsets;
- one inverse-kinematics solution branch is selected and published as a `Float64MultiArray`.

This makes the node a perception-to-motion converter for the UR10 pipeline.

### `vacuum_grip_detect`

This node converts raw UR tool data into a simplified boolean grasp status signal.

It subscribes to:

- `/io_and_status_controller/tool_data` (`ur_msgs/msg/ToolDataMsg`)

and publishes:

- `/vacuum_gripper_status` (`std_msgs/msg/Bool`)

The implementation uses `analog_input2` and a threshold to infer whether the suction gripper is currently holding an object.

This output is used directly by the Behavior Tree to validate grasp and release stages.

## Additional motion utilities

### `send_angles`

This node subscribes to:

- `/ur10/calculated_joint_angles`

and sends those values to:

- `/scaled_joint_trajectory_controller/follow_joint_trajectory`

through a `FollowJointTrajectory` action client.

It can be used as a simple controller-side consumer of the kinematics output.

### `send_conveyor`

This node sends a predefined two-point joint trajectory that moves the UR10 toward the conveyor transfer position.

It is a utility motion command for the transport phase of the task and mirrors the conveyor-directed movement used in higher-level execution logic.

## Vacuum gripper utilities

### `ventosa_on`

Calls:

- `/io_and_status_controller/set_io`

to activate the suction gripper by enabling digital outputs 1 and 2.

### `ventosa_off`

Calls the same IO service to disable the suction gripper by clearing digital outputs 1 and 2.

These utilities can be used independently for testing or manual control of the end-effector.

## Transformation helpers

### `SM_Transformations`

This module contains shared transformation and pose-conversion utilities used by the kinematics code.

It includes functions for:

- rotation matrix generation;
- rotation vector and roll-pitch-yaw conversion;
- pose transformation between references;
- calibration-related transformations;
- difference and sum operations over poses.

This module is not the main operational entry point of the package, but it supports the math used by `calculate_kinematics`.

## How this package fits into the project

`smartfactory_ur_utils` connects the perception pipeline to the UR10 execution stack.

A typical flow is:

1. ArUco detection produces target pose estimates
2. `smartfactory_aruco_poses` filters those estimates
3. `smartfactory_ur_utils/calculate_kinematics` converts the filtered pose into UR10 joint targets
4. motion nodes send those targets to the UR action interface
5. `vacuum_grip_detect` publishes grasp-state evidence for higher-level logic

This makes the package a key integration layer between perception, robot motion, and task supervision.

## Relationship with other packages

- `smart_factory_bringup`
  Starts `calculate_kinematics` and `vacuum_grip_detect` as part of the main runtime stack.
- `smartfactory_behavior_tree`
  Consumes `/ur10/calculated_joint_angles` and `/vacuum_gripper_status` as part of the closed-loop task logic.
- `smartfactory_aruco_poses`
  Provides the filtered pose input used by the kinematics node.

## Example usage

After sourcing the workspace:

```bash
ros2 run smartfactory_ur_utils calculate_kinematics
ros2 run smartfactory_ur_utils vacuum_grip_detect
```

Other utilities can be run independently as needed:

```bash
ros2 run smartfactory_ur_utils send_angles
ros2 run smartfactory_ur_utils send_conveyor
ros2 run smartfactory_ur_utils ventosa_on
ros2 run smartfactory_ur_utils ventosa_off
```

## Notes

- Several computations in this package are project-specific and use hardcoded offsets or thresholds.
- The kinematics node currently depends on the Kinect filtered pose topic and assumes a specific perception layout.
- Some utilities overlap in functionality with higher-level nodes used inside `smartfactory_behavior_tree`, but this package remains the primary source of reusable UR10 support tools in the workspace.
