# smartfactory_pkg

Monorepo/workspace do SmartFactory (ROS 2 Jazzy).

## Como rodar o cenário principal (UR10 + câmeras + ArUco + RViz)

Launch: `smartfactoring_ws/src/smart_factory_bringup/launch/smart_factory_scene.launch.py`

### 1) Build do workspace

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2) Executar o launch

```bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py
```

### 3) Argumentos do launch

Todos com default `true`:
- `rviz:=true|false`
- `kinect:=true|false`
- `astra:=true|false`
- `ur10:=true|false`

Exemplos:

```bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py rviz:=false
ros2 launch smart_factory_bringup smart_factory_scene.launch.py ur10:=false
ros2 launch smart_factory_bringup smart_factory_scene.launch.py kinect:=true astra:=false
```

## O que o launch faz

Arquivo: `smartfactoring_ws/src/smart_factory_bringup/launch/smart_factory_scene.launch.py`

- **UR10 (quando `ur10:=true`)**
  - Inclui `ur_robot_driver/launch/ur10.launch.py` com `robot_ip:=192.168.0.104` e `launch_rviz:=false`.
  - Sobe `robot_state_publisher` no namespace `custom_ur10` com o URDF `smart_factory_arm_description/urdf/smart_factory_arm.urdf`.

- **Descrição/frames das câmeras**
  - Sobe `robot_state_publisher` no namespace `cameras` com o Xacro `smartfactory_description/urdf/smart_spawn.urdf.xacro`.

- **Astra (quando `astra:=true`)**
  - Inclui `smart_factory_bringup/launch/astra_pro_plus.launch.py`.
  - Sobe `aruco_pose_estimation/aruco_node.py` como `astra_aruco` usando `smart_factory_bringup/param/aruco_params.yaml` (seção `/aruco_node_astra`).

- **Kinect (quando `kinect:=true`)**
  - Sobe `kinect_ros2/kinect_ros2_node` no namespace `/kinect`.
  - Sobe `aruco_pose_estimation/aruco_node.py` como `kinect_aruco` usando `smart_factory_bringup/param/aruco_params.yaml` (seção `/aruco_node_kinect`).
  - Sobe `smart_factory_bringup/kinect_aruco_pose_transformer`.

- **TF fixo**
  - Sobe `tf2_ros/static_transform_publisher` publicando `map -> world` com transform identidade.

- **Processamento e utilitários**
  - `smartfactory_aruco_poses/filtered_pose` (filtragem/suavização de poses).
  - `smartfactory_ur_utils/calculate_kinematics` (cálculo de cinemática/ângulos do UR10 a partir das poses).
  - `smartfactory_ur_utils/vacuum_grip_detect` (status do vácuo/ventosa).

- **RViz (quando `rviz:=true`)**
  - Abre `rviz2` com `smart_factory_bringup/rviz/smart_factory_scene.rviz`.

