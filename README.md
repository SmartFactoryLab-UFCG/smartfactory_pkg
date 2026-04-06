# Smart Factory Package Repository

Monorepo for the Smart Factory manipulation project based on ROS 2 Jazzy.

This repository contains the code, simulation assets, experimental material, and documentation used in the study:

**Closed-loop grasp verification via structured multi-sensor reasoning under occlusion**

The main ROS 2 workspace is located in `smartfactoring_ws/`.

## Repository overview

This repository supports a UR10-based pick-and-place scenario in which grasp execution happens under partial observability. The project compares:

- an `open-loop` baseline that executes the nominal manipulation sequence;
- a `closed-loop` execution architecture that verifies task outcomes through a Behavior Tree and multi-sensor evidence.

The central contribution, consistent with the paper, is not a new perception or planning algorithm. It is the execution logic that verifies whether a grasp, transport, and placement actually succeeded under occlusion.

## Main repository structure

- `smartfactoring_ws/`
  Main ROS 2 workspace containing the runtime stack and in-house packages.
- `smartfactoring_isaac/`
  Isaac Sim scene assets and simulation environment files.
- `dependencies/`
  Additional third-party dependencies stored outside the ROS 2 source tree.
- `ros-jazzy-ros1-bridge/`
  ROS bridge-related workspace artifacts.

## Main workspace packages

The main in-house packages inside `smartfactoring_ws/src/` are:

- `smart_factory_bringup`
- `smart_factory_arm_description`
- `smartfactory_description`
- `smartfactory_aruco_poses`
- `smartfactory_ur_utils`
- `smartfactory_behavior_tree`
- `smart_factory_moveit`
- `external_pkgs`

See the workspace-level documentation for details:

- [smartfactoring_ws/README.md](/workspaces/smartfactory_pkg/smartfactoring_ws/README.md)

## Architecture summary

At a high level, the implemented runtime pipeline is:

1. cameras publish observations of the workspace
2. ArUco nodes estimate marker poses
3. pose utilities filter the target pose
4. kinematics utilities generate UR10 joint targets
5. the robot executes motion via ROS 2 control interfaces
6. suction and marker signals are evaluated during execution
7. a Behavior Tree decides whether to continue, fail, or recover

This reflects the closed-loop execution model described in the article: task progression depends on verified environmental state changes, not only on whether an action was sent.

## Quick start

### 1. Build the ROS 2 workspace

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential
source install/setup.bash
```

### 2. Start the main bringup

In one terminal:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py
```

### 3. Run one of the execution modes

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

## Where to look next

- Workspace overview:
  [smartfactoring_ws/README.md](/workspaces/smartfactory_pkg/smartfactoring_ws/README.md)
- Bringup package:
  [smart_factory_bringup/README.md](/workspaces/smartfactory_pkg/smartfactoring_ws/src/smart_factory_bringup/README.md)
- Behavior Tree package:
  [smartfactory_behavior_tree/README.md](/workspaces/smartfactory_pkg/smartfactoring_ws/src/smartfactory_behavior_tree/README.md)
- UR10 utilities:
  [smartfactory_ur_utils/README.md](/workspaces/smartfactory_pkg/smartfactoring_ws/src/smartfactory_ur_utils/README.md)
- Isaac Sim assets:
  [smartfactoring_isaac/README.md](/workspaces/smartfactory_pkg/smartfactoring_isaac/README.md)

## Notes

- The repository mixes runtime code, simulation assets, vendored dependencies, and experimental artifacts.
- Some values in the current implementation, such as hardware IP addresses, offsets, thresholds, and topic assumptions, are specific to the original experimental setup.