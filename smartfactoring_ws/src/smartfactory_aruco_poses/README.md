# Smart Factory Aruco Poses

This package provides lightweight utilities for processing, smoothing, and analyzing ArUco pose data in the Smart Factory workspace.

Its main runtime role is to post-process ArUco detections and publish a filtered pose stream that can be consumed by downstream components such as kinematics and task execution nodes.

## Purpose

`smartfactory_aruco_poses` is a small perception-support package focused on pose handling rather than marker detection itself.

In the current workspace, it is primarily used to:

- smooth noisy ArUco pose measurements;
- republish filtered pose data in ROS-friendly formats;
- support basic pose logging and plotting for analysis;
- provide helper tools for evaluating pose stability.

## Package type

This is an `ament_python` package with the following console executables:

- `filtered_pose`
- `aruco_metrics`
- `plot`

## Main runtime node

### `filtered_pose`

This is the main operational node of the package and the one used by the project bringup.

It subscribes to a `PoseStamped` topic, applies a moving-average filter to the position coordinates, and republishes the smoothed result.

### Default inputs

- input pose topic: `/kinect/aruco/target_pose_world`

### Default outputs

- filtered pose array topic: `/kinect/aruco/filtered_poses`
- filtered pose topic: `/kinect/aruco/filtered_pose`

### Filtering behavior

The node applies a moving average over a fixed-size window for:

- `x`
- `y`
- `z`

The orientation is forwarded unchanged from the latest input message.

By default, the node uses a window size of `10` samples.

### Message types

- input: `geometry_msgs/msg/PoseStamped`
- outputs: `geometry_msgs/msg/PoseStamped` and `geometry_msgs/msg/PoseArray`

The package publishes both formats to preserve compatibility with different downstream consumers.

## Analysis utilities

### `aruco_metrics`

This node subscribes to a pose array stream and logs the pose samples to a CSV file while also updating interactive matplotlib plots.

It is intended for quick experimental analysis of pose variation over time rather than for production runtime use.

Current implementation details:

- subscribes to `/kinect/aruco/poses`
- reads the first pose from each incoming `PoseArray`
- writes data to `aruco_pose_data.csv`
- displays live plots for `x`, `y`, and `z`

### `plot`

This script reads pose data from a CSV file and computes simple descriptive metrics, including:

- mean
- variance
- standard deviation
- RMSE
- MAE

It also generates static plots and saves an output image for offline inspection.

This script is best understood as an analysis helper for experiments and noise evaluation.

## How this package fits into the project

The package sits after ArUco detection and before downstream motion logic.

A typical flow is:

1. camera drivers publish images
2. ArUco detection nodes estimate marker poses
3. pose transformation nodes publish poses in the desired frame
4. `smartfactory_aruco_poses/filtered_pose` smooths the resulting pose stream
5. downstream packages use the filtered pose for motion or behavior logic

In the main system bringup, this package is used by `smart_factory_bringup`, which launches the `filtered_pose` node as part of the perception-processing pipeline.

## Example usage

After sourcing the workspace:

```bash
ros2 run smartfactory_aruco_poses filtered_pose
```

For analysis utilities:

```bash
ros2 run smartfactory_aruco_poses aruco_metrics
ros2 run smartfactory_aruco_poses plot
```

## Notes

- This package does not perform ArUco detection itself. Detection is handled elsewhere in the workspace, such as by `aruco_pose_estimation`.
- The filtering currently affects only position and does not smooth orientation.
