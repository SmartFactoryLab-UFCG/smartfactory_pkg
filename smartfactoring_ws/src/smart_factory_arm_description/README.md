# Smart Factory Arm Description

This package provides the robot description assets for the Smart Factory manipulator setup.

It defines the UR10 arm description used in the project together with the custom end-effector geometry, meshes, and auxiliary launch files required for visualization and integration with the rest of the ROS 2 workspace.

## Robot structure preview

The image above shows a 3D render of the custom gripper mesh used by this package.

## Purpose

The package is responsible for describing the physical structure of the manipulator used in the Smart Factory cell, including:

- the UR10 robot model;
- the custom gripper attached to the robot flange;
- collision and inertial elements for the end-effector;
- mesh assets used for visualization;
- optional suction plugin support for simulation.

In practice, this package is the reference source for the arm and gripper model consumed by visualization, bringup, and robot state publishing.

## Package contents

The main folders are:

- `urdf/`: URDF and Xacro files defining the arm and gripper model.
- `meshes/`: mesh files used by the custom end-effector.
- `launch/`: helper launch files for loading or visualizing the robot description.
- `rviz/`: RViz configurations associated with the robot and scene.

## Main files

- `urdf/smart_factory_arm.urdf.xacro`: top-level Xacro that combines the UR robot description with the custom gripper.
- `urdf/custom_gripper.urdf.xacro`: defines the custom end-effector structure, links, joints, collision geometry, and inertial properties.
- `urdf/vacuum_gazebo.urdf.xacro`: provides the optional vacuum gripper plugin configuration for simulation.
- `urdf/smart_factory_arm.urdf`: generated URDF version of the robot description.
- `launch/gripper_launch.launch.py`: helper launch file for publishing or visualizing the robot description.

## Description model

The top-level description is built by extending the UR robot model from `ur_robot_driver` and attaching a custom gripper to the robot flange.

The custom end-effector includes:

- an extender body;
- a wrist section;
- a middle gripper section;
- left and right fingers;
- left and right suction attachment links.

This makes the package suitable for applications that need a project-specific manipulator model rather than the default UR10-only description.

## Simulation support

When the `gazebo` argument is enabled in the Xacro description, the package also includes vacuum gripper plugin blocks for the suction links. This allows the same structural model to be reused in simulation workflows that require suction-based grasp behavior.

## How it is used in the project

This package is consumed by other project modules, especially:

- `smart_factory_bringup`, which uses the robot description when publishing the UR10 model;
- visualization workflows based on `robot_state_publisher` and RViz;
- any pipeline that depends on a consistent kinematic and geometric description of the arm and gripper.

## Notes

- `smart_factory_arm.urdf` is a generated file. If the model needs to be changed, update the Xacro sources instead of editing the generated URDF by hand.
- The package depends on `ur_robot_driver` for the base UR robot description.
- This package focuses on robot description assets only. Control, planning, and task execution are handled by other packages in the workspace.
