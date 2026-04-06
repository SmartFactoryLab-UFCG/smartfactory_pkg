# Smart Factoring Workspace

ROS 2 Jazzy workspace for the Smart Factory manipulation system described in the paper:

**Closed-loop grasp verification via structured multi-sensor reasoning under occlusion**

This workspace implements the runtime stack used to evaluate open-loop and closed-loop execution for UR10-based pick-and-place under partial observability.

## Overview

The system addresses a manipulation scenario in which a UR10 robot must pick an object from a mobile support platform and place it onto a conveyor. The central challenge is that grasp execution occurs under partial observability:

- the top marker used for object localization becomes occluded during grasping;
- a secondary camera only provides indirect evidence after lift-off;
- the suction sensor provides local contact evidence but is not sufficient by itself to fully validate grasp success.

Following the architecture described in the article, this workspace implements a task pipeline that combines:

- visual pose estimation from multiple cameras;
- filtered target pose generation;
- inverse kinematics for the UR10;
- motion execution through ROS 2 control interfaces;
- vacuum gripper sensing;
- Behavior Tree-based execution logic for verification and recovery.

## Research motivation

The key idea of the paper is that many industrial pick-and-place pipelines are still executed in open loop: the robot is commanded to grasp and move, and success is implicitly assumed once actions are issued.

In the Smart Factory scenario, this assumption is fragile because:

- the object marker is not continuously visible during grasping;
- the available sensors provide complementary but incomplete evidence;
- failures can propagate silently if the controller does not explicitly verify task outcomes.

This workspace therefore supports two execution styles:

- `open-loop`
  Executes the nominal sequence without the full verification logic.
- `closed-loop`
  Uses a Behavior Tree to validate pickup, transport, and placement through multi-sensor evidence and expected visibility transitions.

## System architecture

At a high level, the runtime architecture is:

1. cameras publish RGB/depth observations
2. ArUco detection estimates marker poses
3. pose processing nodes filter and republish the target pose
4. UR10 utilities convert target pose into joint commands
5. the robot executes motion through ROS 2 action interfaces
6. vacuum sensing and visual evidence are evaluated during task execution
7. the Behavior Tree decides whether to continue, interrupt, or recover

This architecture is aligned with the paper's closed-loop execution model: action completion is not treated as equivalent to task success. Instead, the controller advances only when task-relevant conditions are satisfied.

## Open-loop vs. closed-loop in this workspace

### Open-loop baseline

The open-loop baseline is represented in the Behavior Tree package by:

```bash
ros2 run smartfactory_behavior_tree behavior_pick_and_place
```

This mode follows the nominal manipulation sequence but does not include the full verification logic used in the closed-loop supervisor.

### Closed-loop execution

The closed-loop supervisor is:

```bash
ros2 run smartfactory_behavior_tree pick_and_place
```

This mode verifies manipulation outcomes using:

- top-marker availability before grasping;
- vacuum confirmation after suction activation;
- stable side-marker evidence after lift-off;
- continuous monitoring during transport;
- explicit release and placement confirmation.

This is the execution mode that reflects the main contribution of the article.

## Main workspace packages

The workspace source tree is organized around the following in-house packages.

- `smart_factory_bringup`
  Starts the integrated runtime scene, including UR10, cameras, ArUco nodes, pose utilities, and RViz.
- `smart_factory_arm_description`
  Provides the UR10 and custom gripper description assets.
- `smartfactory_description`
  Provides the world and perception-side sensor descriptions.
- `smartfactory_aruco_poses`
  Filters and republishes ArUco pose data for downstream consumers.
- `smartfactory_ur_utils`
  Computes UR10 target joint angles and publishes suction-related status signals.
- `smartfactory_behavior_tree`
  Implements the open-loop baseline and the closed-loop supervisor.
- `smart_factory_moveit`
  Provides MoveIt 2 configuration and planning-related utilities.
- `external_pkgs`
  Contains vendored third-party packages required by the workspace.

## How to build

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential
source install/setup.bash
```

## How to run the main system

### 1. Start the integrated scene

In one terminal:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py
```

The main bringup supports:

- `rviz:=true|false`
- `kinect:=true|false`
- `astra:=true|false`
- `ur10:=true|false`

Example:

```bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py rviz:=false
```

### 2. Run one of the execution modes

In another terminal:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source install/setup.bash
```

Open-loop baseline:

```bash
ros2 run smartfactory_behavior_tree behavior_pick_and_place
```

Closed-loop execution:

```bash
ros2 run smartfactory_behavior_tree pick_and_place
```

## Main runtime signals

Important runtime interfaces in this workspace include:

- `/kinect/aruco/markers`
- `/astra/aruco/markers`
- `/kinect/aruco/filtered_pose`
- `/kinect/aruco/filtered_poses`
- `/ur10/calculated_joint_angles`
- `/vacuum_gripper_status`
- `/scaled_joint_trajectory_controller/follow_joint_trajectory`
- `/io_and_status_controller/set_io`

These signals connect perception, kinematics, motion execution, and execution-time verification.

## Experimental interpretation

Consistent with the article, the workspace is designed to compare:

- identical perception and motion layers across both modes;
- different execution logic at the task-control level.

In other words, the contribution is not a new detector, planner, or grasp generator. The main contribution is the structured execution logic that verifies whether the expected effects of an action actually occurred.

## Related documentation

Detailed package documentation is available in:

- `src/smart_factory_bringup/README.md`
- `src/smart_factory_arm_description/README.md`
- `src/smartfactory_description/README.md`
- `src/smartfactory_aruco_poses/README.md`
- `src/smartfactory_ur_utils/README.md`
- `src/smartfactory_behavior_tree/README.md`
- `src/smart_factory_moveit/README.md`
- `src/external_pkgs/README.md`

## Notes

- The workspace mixes production-oriented modules with experimental utilities used during development and evaluation.
- Several values in the current implementation, such as IP addresses, offsets, thresholds, and topic assumptions, are project-specific and may need to be adapted for reuse in another setup.
