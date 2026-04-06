# Smart Factory Description

This package provides environment and sensor description assets for the Smart Factory workspace.

It contains the URDF/Xacro files, meshes, and RViz resources used to describe the world frame, the camera setup, and marker-related visual assets that support the perception pipeline.

## Purpose

`smartfactory_description` is the workspace package responsible for static scene and sensor description, especially for perception-related components.

In practice, it is used to:

- define the `world` reference frame;
- describe the Kinect sensor model and its frame hierarchy;
- provide a Basler camera model for description and simulation experiments;
- provide ArUco-related mesh assets;
- compose these elements into a reusable Xacro description consumed by bringup.

## Package type

This is an `ament_python` package used as a resource package.

It does not expose runtime executables. Instead, it installs:

- `urdf/`
- `meshes/`
- `rviz/`

for use by other packages in the workspace.

## Main contents

### `urdf/`

This folder contains the Xacro descriptions that define the static world and sensor layout.

Important files include:

- `smart_spawn.urdf.xacro`
  Main composed description used by the bringup package.
- `smart_library.urdf.xacro`
  Includes the reusable Xacro components of the package.
- `smartfactory_world.urdf.xacro`
  Defines the `world` link used as the base frame.
- `kinect.urdf.xacro`
  Defines the Kinect camera model, links, optical frames, and Gazebo sensor plugin configuration.
- `basler.urdf.xacro`
  Defines a Basler-style camera model and related frame structure.
- `aruco.urdf.xacro`
  Defines a simple ArUco-marked object model for simulation-related usage.

### `meshes/`

This folder contains the geometric and visual assets referenced by the Xacros, including:

- Kinect mesh assets
- Basler mesh assets
- ArUco marker meshes and images

### `rviz/`

- `smartfactory.rviz`
  RViz configuration associated with this package.

## Main composed description

The central composition file is:

`urdf/smart_spawn.urdf.xacro`

This Xacro:

- includes the package Xacro library;
- instantiates the `world` frame;
- instantiates the Kinect sensor description;
- leaves the Basler sensor available but currently commented out.

This means that, in the current workspace configuration, the package is mainly being used to publish the camera frame structure centered on the Kinect setup.

## Kinect description

The Kinect model defined in `kinect.urdf.xacro` includes:

- `camera_rgb_frame_kinect`
- `camera_rgb_optical_frame_kinect`
- `camera_link_kinect`
- `camera_depth_frame_kinect`
- `camera_depth_optical_frame_kinect`

It also contains Gazebo sensor and plugin blocks for simulated RGB/depth output. In addition to the geometric model, the file encodes a fixed placement for the sensor relative to `world`.

## Basler description

The Basler model defined in `basler.urdf.xacro` includes a similar frame structure and simulation-oriented camera plugin configuration.

However, based on the current `smart_spawn.urdf.xacro`, the Basler sensor is not actively instantiated in the main composed description.

## How this package is used in the project

This package is consumed by `smart_factory_bringup`, which loads:

- `smartfactory_description/urdf/smart_spawn.urdf.xacro`

through `robot_state_publisher` in order to publish the camera and world frame structure used by the rest of the system.

This makes `smartfactory_description` the reference package for the static perception-side frame tree.

## Relationship with other packages

- `smart_factory_bringup`
  Uses this package to publish the scene and camera description.
- `smartfactory_aruco_poses`
  Operates on poses expressed relative to the perception frame structure supported by this package.
- `smartfactory_behavior_tree`
  Consumes perception outputs that ultimately depend on the camera and world layout defined here.

## Notes

- This package is focused on description assets, not sensor drivers.
- Sensor runtime nodes such as Kinect and Astra drivers are started elsewhere, mainly by `smart_factory_bringup`.
- Some Xacro files include Gazebo-oriented elements, so this package serves both frame description and simulation-support roles.
