# Smart Factory MoveIt

This package provides the MoveIt 2 configuration and planning utilities for the Smart Factory UR10 manipulator setup.

It contains the semantic robot description, planning pipeline configuration, RViz setup, launch files, and a small set of C++ utilities used to test planning and populate the planning scene.

## Purpose

`smart_factory_moveit` is the MoveIt-facing package for the project. Its main responsibilities are:

- defining the semantic and planning configuration for the Smart Factory arm;
- launching `move_group`, RViz, and optional MoveIt Servo;
- providing planning scene configuration files;
- offering example and test executables for motion planning and MoveIt Task Constructor workflows.

## Package contents

The main directories are:

- `config/`: MoveIt planning, controllers, servo, joint limits, RViz, and scene object configuration.
- `launch/`: launch files for MoveIt bringup and MTC-based tests.
- `srdf/`: semantic and MoveIt-specific robot description files.
- `src/`: C++ executables for planning scene setup and planning tests.

## Main launch files

### `launch/ur10_moveit.launch.py`

This is the main MoveIt launch file for the package. It:

- builds the MoveIt configuration using `MoveItConfigsBuilder`;
- waits for the robot description from the UR driver;
- starts `move_group`;
- optionally starts MoveIt Servo;
- optionally opens RViz with the package MoveIt configuration.

Supported launch arguments include:

- `launch_rviz:=true|false`
- `launch_servo:=true|false`
- `use_sim_time:=true|false`
- `publish_robot_description_semantic:=true|false`
- `warehouse_sqlite_path:=<path>`

Example:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch smart_factory_moveit ur10_moveit.launch.py
```

### `launch/mtc_ur10_test.launch.py`

This launch file starts:

- `smart_factory_moveit/mtc_ur10_test`
- a static transform publisher for `world -> base_link`

It is intended as a lightweight MoveIt Task Constructor test entry point.

## Main configuration files

Important files in `config/` include:

- `kinematics.yaml`
- `joint_limits.yaml`
- `moveit_controllers.yaml`
- `ompl_planning.yaml`
- `chomp_planning.yaml`
- `pilz_cartesian_limits.yaml`
- `pilz_industrial_motion_planner_planning.yaml`
- `ur_servo.yaml`
- `moveit.rviz`
- `scene_objects.yaml`

Together, these files define planning behavior, controller integration, RViz visualization, servo parameters, and the collision scene used by helper tools.

## Semantic and robot model integration

The package includes:

- `srdf/smartfactory_arm.srdf`
- `srdf/smartfactory_arm.urdf.xacro`
- `srdf/smartfactory_arm.ros2_control.xacro`

These files adapt the Smart Factory arm description for MoveIt 2 and align the project-specific manipulator model with the planning stack.

The package depends on `smart_factory_arm_description` for the underlying robot description assets.

## Included executables

### `lab_objects_instance`

This node loads collision objects from `config/scene_objects.yaml` and inserts them into the MoveIt planning scene through `PlanningSceneInterface`.

It is useful when you want MoveIt planning to consider the static elements of the lab or workspace.

### `move_box_test`

This executable demonstrates a simple MoveIt planning workflow:

- create a `MoveGroupInterface`;
- add a box obstacle to the planning scene;
- plan to a target pose;
- visualize and execute the resulting trajectory.

It is mainly intended as a planning and visualization test node.

### `mtc_ur10_test`

This executable contains an experimental MoveIt Task Constructor pipeline for a UR10 pick-style task. It sets up a planning scene object, creates an MTC task, plans it, and attempts execution.

This node is best understood as a development or prototype utility rather than a production task controller.

## How this package fits into the project

`smart_factory_moveit` complements the rest of the workspace by providing planning support for the Smart Factory manipulator stack.

In particular:

- `smart_factory_arm_description` defines the arm and custom gripper model;
- `smart_factory_bringup` is responsible for system bringup and runtime orchestration;
- `smart_factory_moveit` provides the planning-side configuration and tooling for the same manipulator setup.
