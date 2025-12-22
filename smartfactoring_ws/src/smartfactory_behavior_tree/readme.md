## smartfactory_behavior_tree

[![ROS 2 Humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)



This ROS 2 package is part of the SmartFactoring Lab's modular control system for industrial robotics. It uses **Behavior Trees** (via the `py_trees` and `py_trees_ros` libraries) to coordinate perception, manipulation, and actuation tasks within a smart manufacturing cell.

Each component — such as the UR10 robotic arm, sensors, gripper, and vision systems — is implemented as an independent ROS 2 node and integrated into a behavior tree supervisor.

🚧 **Note:** This package is currently under active development and may contain incomplete or experimental features.

---

## ▶️ How to Run the Pick and Place Behavior

After starting the factory scene (including robot drivers and perception nodes), you can launch the main behavior tree using:

```bash
ros2 launch smartfactory_bringup scene.launch.py
```
2. Run the behavior tree:

```bash
ros2 launch smartfactory_bringup scene.launch.py
```

This `pick_and_place` behavior includes:

* ArUco marker detection using Kinect or Basler camera

* Motion planning for the UR10 robot to the pickup pose

* Gripper activation via vacuum system

* Motion to the conveyor position

* Releasing the object
