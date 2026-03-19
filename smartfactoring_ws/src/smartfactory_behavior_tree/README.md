## smartfactory_behavior_tree

Pacote ROS 2 (`ament_python`) para orquestrar tarefas via **Behavior Trees** usando `py_trees` e `py_trees_ros`.

O supervisor principal do fluxo de *pick and place* é:
- `smartfactoring_ws/src/smartfactory_behavior_tree/smartfactory_behavior_tree/supervisores/pick_and_place.py`

## Como executar o `pick_and_place`

### 1) Subir a cena (sensores, aruco, utilitários)

Em um terminal:

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch smart_factory_bringup smart_factory_scene.launch.py
```

### 2) Rodar o supervisor da BT

Em outro terminal (no mesmo workspace):

```bash
cd /workspaces/smartfactory_pkg/smartfactoring_ws
source install/setup.bash
ros2 run smartfactory_behavior_tree pick_and_place
```

## Como a árvore de comportamento funciona

Arquivo: `smartfactoring_ws/src/smartfactory_behavior_tree/smartfactory_behavior_tree/supervisores/pick_and_place.py`

Em alto nível:

- A raiz é um **Selector** com dois ramos:
  - **Ramo A (peça presente)**: executa o ciclo completo de pegar e levar até a esteira.
  - **Ramo B (sem peça)**: mantém o nó rodando e imprime/loga um aviso pedindo para inserir uma peça.

### Ramo A: Pick + Carry + Release + Confirmação

1) **Detecção do topo da peça** (`CheckArucoPose`)
- Lê a pose do ArUco “target” (topo) já filtrada no pipeline do Kinect.

2) **Movimento para pegar** (`MoveUR10`)
- Envia uma `FollowJointTrajectory` com os ângulos publicados em `/ur10/calculated_joint_angles`.

3) **Pega com ventosa e valida com sensor**
- `VentosaOn`: ativa IOs via `/io_and_status_controller/set_io`.
- `CheckUltrasonicGripper`: espera o `/vacuum_gripper_status` indicar que a ventosa está segurando.

4) **Confirmação inicial do “pegou” por visão (Astra)**
- `WaitAstraId5`: exige detecção estável do ArUco `id=5` (lateral) pela Astra.

5) **Levar para a esteira com monitoramento contínuo (Parallel)**
- Um **Parallel** roda:
  - `SendConveyorAction`: move para a pose da esteira.
  - `MonitorCarry`: monitora durante o movimento:
    - falha se o vácuo “soltar” (ultrassônico/vácuo deixa de detectar)
    - falha se a Astra deixar de ver `id=5`
    - falha se a pose do `id=5` não variar (anti falso positivo: marker parado no cenário)
    - registra como evidência extra quando o Kinect detecta `id=0` durante o transporte

6) **Soltar e confirmar entrega (esteira ligada)**
- `VentosaOff`: desativa IOs.
- `ConfirmPlaced`:
  - confirma `vacuum_gripper_status == False` estável
  - exige ver `id=5` logo após soltar (evidência de que foi colocado na esteira)
  - exige que o `id=5` suma em seguida (evidência de que a esteira levou a peça)

### Ramo B: Sem peça

- `NoPieceFeedback`: imprime/loga periodicamente que não há peça detectada (topo não visível).

## Tópicos e sinais usados (resumo)

- Kinect ArUco (ids + poses): `/kinect/aruco/markers` (`aruco_interfaces/msg/ArucoMarkers`)
- Astra ArUco (ids + poses): `/astra/aruco/markers` (`aruco_interfaces/msg/ArucoMarkers`)
- Vácuo/ventosa (Bool): `/vacuum_gripper_status`
- Ângulos alvo do UR10 (Float64MultiArray): `/ur10/calculated_joint_angles`
- Action UR10 (FollowJointTrajectory): `/scaled_joint_trajectory_controller/follow_joint_trajectory`
