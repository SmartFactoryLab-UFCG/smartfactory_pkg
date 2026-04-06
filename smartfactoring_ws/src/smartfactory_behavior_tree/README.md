# Smart Factory Behavior Tree

This package implements the task-level execution logic of the Smart Factory system using Behavior Trees on ROS 2.

It is responsible for coordinating the pick-and-place cycle by combining perception signals, UR10 motion commands, vacuum gripper control, and execution-time verification logic.

## Purpose

`smartfactory_behavior_tree` is the high-level orchestration layer of the workspace.

Instead of assuming that commanded robot actions succeed, this package structures the task as a Behavior Tree that checks sensor evidence at each critical stage of the manipulation cycle. This allows the system to:

- wait for a valid part detection before starting;
- command the UR10 to move to the grasp pose;
- activate and validate the vacuum gripper;
- verify the grasp using multi-sensor evidence;
- monitor transport toward the conveyor;
- validate release and placement outcomes.

## Open-loop vs. closed-loop execution

The project distinguishes between two execution styles for the pick-and-place task.

### Open-loop execution

In an open-loop version of the task, the robot:

- detects the part;
- computes the motion;
- executes the grasp and transport actions;
- assumes that each commanded step succeeds.

In this mode, action completion is treated as if it were equivalent to task success. If the grasp fails during occlusion or if the part is lost during transport, the system may continue without explicitly detecting the failure.

### Closed-loop execution

The current `pick_and_place` supervisor is designed around closed-loop execution.

In this mode, the tree does not treat motion completion as sufficient evidence of success. Instead, it verifies task progress through sensor-based conditions such as:

- recent detection of the top ArUco target before grasping;
- vacuum confirmation after suction activation;
- stable Astra-side marker visibility after pickup;
- continuous monitoring of grasp validity during transport;
- confirmation that the part was released and then removed by the conveyor.

This means that the tree reasons about expected state transitions in the environment, not only about whether an action was sent successfully.

### Practical interpretation in this package

Within this package:

- the closed-loop logic is implemented explicitly in `supervisores/pick_and_place.py`;
- validation nodes such as `CheckUltrasonicGripper`, `WaitAstraId5`, `MonitorCarry`, `WaitVacuumReleasedStable`, and `WaitAstraId5Gone` are the key elements that make the execution closed-loop;
- a purely open-loop execution would omit these verification steps and would progress based mainly on action dispatch and nominal sequencing.

## Package type

This is an `ament_python` package based on:

- `py_trees`
- `py_trees_ros`
- ROS 2 nodes, topics, services, and actions

## Main execution entry points

The package currently exposes two task-level execution modes:

- `pick_and_place`
  Closed-loop supervisor implemented in `smartfactory_behavior_tree/supervisores/pick_and_place.py`
- `behavior_pick_and_place`
  Open-loop baseline implemented in `smartfactory_behavior_tree/teste/behavior_pick_and_place.py`

## How to run

### 1. Start the perception and robot scene

In one terminal:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch smart_factory_bringup smart_factory_scene.launch.py
```

### 2. Run the closed-loop Behavior Tree

In another terminal:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source install/setup.bash
ros2 run smartfactory_behavior_tree pick_and_place
```

### 3. Run the open-loop baseline

In another terminal:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source install/setup.bash
ros2 run smartfactory_behavior_tree behavior_pick_and_place
```

You can also start the open-loop baseline through the package launch file:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source install/setup.bash
ros2 launch smartfactory_behavior_tree behavior_pick_and_place.launch.py
```

### Execution summary

- closed-loop mode: `ros2 run smartfactory_behavior_tree pick_and_place`
- open-loop baseline: `ros2 run smartfactory_behavior_tree behavior_pick_and_place`

Both modes assume that the bringup stack is already running.

## Behavior Tree structure

At a high level, the closed-loop root of the tree is a `Selector` with two branches:

- a task-execution branch that runs the full pick-carry-release cycle when a part is available;
- a fallback branch that keeps reporting that no part is currently detected.

## Closed-loop execution flow

The current `pick_and_place` supervisor follows this sequence.

### 1. Part availability check

`CheckArucoPose`

The tree waits for a recent filtered ArUco pose from the Kinect perception pipeline. This acts as the condition that enables the grasp cycle.

### 2. Move to grasp

`MoveUR10`

The UR10 motion node subscribes to `/ur10/calculated_joint_angles` and sends a `FollowJointTrajectory` goal to:

- `/scaled_joint_trajectory_controller/follow_joint_trajectory`

### 3. Activate the vacuum gripper

`VentosaOn`

The gripper control node calls:

- `/io_and_status_controller/set_io`

to activate the vacuum outputs.

### 4. Validate the grasp with the vacuum sensor

`CheckUltrasonicGripper`

The tree waits for `/vacuum_gripper_status` to indicate that an object is being held.

### 5. Confirm pickup using Astra-side evidence

`WaitAstraId5`

The tree requires stable detection of marker `id=5` from the Astra pipeline as evidence consistent with a successful pickup and lift.

### 6. Carry to the conveyor with continuous monitoring

This stage is implemented as a `Parallel` node named `CarryToConveyor`:

- `SendConveyorAction` sends the predefined conveyor trajectory;
- `MonitorCarry` keeps checking whether the grasp remains valid during transport.

During this phase, the tree monitors:

- vacuum loss from `/vacuum_gripper_status`;
- recent detection of Astra marker `id=5`;
- whether the Astra marker pose actually changes over time;
- optional Kinect evidence from marker `id=0` indicating that the source platform is now clear.

If any of the required conditions fail, the carry monitor returns failure.

### 7. Release and validate placement

After transport:

- `VentosaOff` disables the vacuum outputs;
- `WaitVacuumReleasedStable` confirms that the gripper is no longer holding an object;
- `WaitAstraId5AfterRelease` expects the side marker to still be visible immediately after release;
- `WaitAstraId5Gone` then expects that marker to disappear as the conveyor moves the part away.

This turns placement validation into an explicit sequence of expected sensor transitions instead of a simple assumption that release succeeded.

## No-part branch

If no top marker is detected, the tree enters the fallback branch:

- `NoPieceFeedback`

This behavior keeps the tree alive and periodically logs that no part is currently visible in the cell.

## Runtime components inside the package

The supervisor instantiates several ROS 2 nodes internally:

- `ArucoPoseSubscriber`
  Reads the filtered target pose from the perception pipeline.
- `UR10Motion`
  Sends grasp motion commands based on calculated joint angles.
- `UR10Sensors`
  Monitors the vacuum sensor state.
- `UR10Gripper`
  Controls the UR digital outputs used by the suction gripper.
- `SendConveyorMotion`
  Sends the predefined transport trajectory toward the conveyor.
- `MarkerIdSubscriber`
  Tracks marker visibility state for specific IDs on Kinect and Astra topics.

## Topics, services, and actions used

The package relies on the following interfaces.

### Perception topics

- `/kinect/aruco/filtered_pose`
- `/kinect/aruco/filtered_poses`
- `/kinect/aruco/markers`
- `/astra/aruco/markers`

### Motion and state topics

- `/ur10/calculated_joint_angles`
- `/vacuum_gripper_status`

### Services

- `/io_and_status_controller/set_io`

### Actions

- `/scaled_joint_trajectory_controller/follow_joint_trajectory`

## Launch file

The package includes:

`launch/behavior_pick_and_place.launch.py`

This launch file starts the `behavior_pick_and_place` executable and exposes:

- `enable_kinect`
- `enable_ur10`

These parameters are currently passed to the node, although the main production entry point in the package is the `pick_and_place` executable.

## Other executables

The package also exposes several helper and development-oriented executables, including:

- `ur10_motion`
- `ur10_gripper`
- `ur10_sensors`
- `aruco_detector`
- `behavior_kinect_node`
- `behavior_pick_and_place`

There is also a `teste/` area with legacy or experimental scripts that appear to support earlier development and debugging workflows.

## How this package fits into the project

Within the workspace:

- `smart_factory_bringup` starts the robot, sensors, ArUco pipelines, and utility nodes;
- `smartfactory_aruco_poses` provides filtered pose information;
- `smartfactory_ur_utils` computes robot targets and sensor-derived signals;
- `smartfactory_behavior_tree` consumes those signals and decides when to move, grasp, carry, release, or stop.

This package is therefore the execution-level decision layer of the Smart Factory application.

## Notes

- The tree emphasizes explicit verification of task outcomes rather than action-only execution.
- The current supervisor is centered on a single pick-and-place behavior.
- Some package modules and executables are clearly experimental or legacy, while `supervisores/pick_and_place.py` is the main operational entry point.
