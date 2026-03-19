## smartfactory_pkg

Monorepo/workspace do SmartFactory (ROS 2 Jazzy). O workspace ROS 2 fica em `smartfactoring_ws/`.

## Como executar (bringup + Behavior Tree)

### 1) Build

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2) Subir a cena (bringup)

Em um terminal:

```bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py
```

Argumentos do launch (default `true`): `rviz`, `kinect`, `astra`, `ur10`

```bash
ros2 launch smart_factory_bringup smart_factory_scene.launch.py rviz:=false
ros2 launch smart_factory_bringup smart_factory_scene.launch.py kinect:=true astra:=false
ros2 launch smart_factory_bringup smart_factory_scene.launch.py ur10:=false
```

### 3) Rodar a Behavior Tree (pick and place)

Em outro terminal:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source install/setup.bash
ros2 run smartfactory_behavior_tree pick_and_place
```

Documentação da BT (detalhada): `smartfactoring_ws/src/smartfactory_behavior_tree/readme.md`

## O que o bringup faz

Launch: `smartfactoring_ws/src/smart_factory_bringup/launch/smart_factory_scene.launch.py`

Ele inicia (condicionalmente):
- UR10 (`ur_robot_driver`) e `robot_state_publisher` do braço
- Kinect + Astra
- `aruco_pose_estimation` para Kinect e Astra (tópicos `*/aruco/markers`)
- TF/descrições (`robot_state_publisher` de câmeras, `map -> world`)
- `smartfactory_aruco_poses/filtered_pose` (filtragem/suavização)
- `smartfactory_ur_utils/calculate_kinematics` (gera `/ur10/calculated_joint_angles`)
- `smartfactory_ur_utils/vacuum_grip_detect` (gera `/vacuum_gripper_status`)
- RViz (config `smart_factory_scene.rviz`)

## Como a Behavior Tree funciona (resumo)

Supervisor: `smartfactoring_ws/src/smartfactory_behavior_tree/smartfactory_behavior_tree/supervisores/pick_and_place.py`

Estrutura (alto nível):

- **Raiz = Selector**
  - **Ramo A (peça presente)**: executa `pick + carry + release`.
  - **Ramo B (sem peça)**: loga/aguarda até uma peça aparecer.

Ramo A (sequência simplificada):
- `CheckArucoPose` (topo/target): habilita o ciclo quando o marcador do topo está detectado.
- `MoveUR10`: envia trajetória via action usando `/ur10/calculated_joint_angles`.
- `VentosaOn` + `CheckUltrasonicGripper`: aciona a ventosa e valida com `/vacuum_gripper_status`.
- `WaitAstraId5`: confirma que a Astra vê os marcadores laterais (id=5) de forma estável.
- `CarryToConveyor` (**Parallel**):
  - `SendConveyorAction`: move o UR10 para a região da esteira.
  - `MonitorCarry`: monitora redundâncias durante o movimento (falha se vácuo cair, se Astra perder id=5, e registra evidência do Kinect id=0).
- `VentosaOff` + `ConfirmPlaced`: desliga a ventosa, confirma vácuo liberado e usa “id=5 visto → id=5 some” como evidência de que a esteira levou a peça.

Referência completa: `smartfactoring_ws/src/smartfactory_behavior_tree/README.md`

## Como o projeto funciona (resumo)

Pipeline típico:
- Percepção (AruCo) → pose(s) → cinemática → UR10 executa trajetória → ventosa pega/solta
- `smartfactory_behavior_tree` coordena o fluxo com redundância usando:
  - Kinect/Astra `*/aruco/markers` (`aruco_interfaces/msg/ArucoMarkers`)
  - sensor de vácuo em `/vacuum_gripper_status`
