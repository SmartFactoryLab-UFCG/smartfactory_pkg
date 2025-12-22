import rclpy
import py_trees
import py_trees_ros
import sys
from rclpy.executors import MultiThreadedExecutor

# Importando os módulos refatorados
from smartfactory_behavior_tree.ur10.ur10_motion import (
    UR10Motion, MoveUR10,
    SendConveyorMotion, SendConveyorAction
)
from smartfactory_behavior_tree.ur10.ur10_sensors import UR10Sensors, CheckUltrasonicGripper
from smartfactory_behavior_tree.ur10.ur10_gripper import UR10Gripper, VentosaOn, VentosaOff
from smartfactory_behavior_tree.algoritms_perception.aruco_detector import ArucoPoseSubscriber, CheckArucoPose


def create_root(ur10_motion, ur10_sensors, ur10_gripper, aruco_detector, send_conveyor_motion):
    """
    Monta a árvore de comportamento do Pick and Place utilizando os módulos dos dispositivos e algoritmos.
    """
    root = py_trees.composites.Sequence(name="PickAndPlace", memory=True)

    # Comportamentos
    check_aruco = CheckArucoPose(aruco_detector)
    move_ur10 = MoveUR10(ur10_motion)
    check_grip = CheckUltrasonicGripper(ur10_sensors)
    ventosa_on = VentosaOn(ur10_gripper)
    send_conveyor = SendConveyorAction(send_conveyor_motion)
    ventosa_off = VentosaOff(ur10_gripper)

    # Sequência de execução
    seq = py_trees.composites.Sequence(name="SequenciaPickAndPlace", memory=True)
    seq.add_children([
        check_aruco,
        move_ur10,
        check_grip,
        ventosa_on,
        send_conveyor,     # ✅ Inserido aqui!
        ventosa_off
    ])

    root.add_child(seq)
    return root


def main(args=None):
    """
    Função principal para iniciar o Pick and Place utilizando ROS2 e Behavior Tree.
    """
    rclpy.init(args=args)

    # Instanciando os módulos (nós ROS2)
    aruco_detector = ArucoPoseSubscriber()
    ur10_motion = UR10Motion()
    ur10_sensors = UR10Sensors()
    ur10_gripper = UR10Gripper()
    send_conveyor_motion = SendConveyorMotion()  # ✅ Adicionado

    # Criando a árvore de comportamento
    root = create_root(
        ur10_motion,
        ur10_sensors,
        ur10_gripper,
        aruco_detector,
        send_conveyor_motion  # ✅ Novo argumento
    )
    behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        behavior_tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError:
        sys.exit(1)

    # Executando a árvore com tick automático
    behavior_tree.tick_tock(period_ms=1000.0)

    # Executor ROS2 com todos os nós
    executor = MultiThreadedExecutor()
    executor.add_node(aruco_detector)
    executor.add_node(ur10_motion)
    executor.add_node(ur10_sensors)
    executor.add_node(ur10_gripper)
    executor.add_node(send_conveyor_motion)  # ✅ Adicionado ao executor
    executor.add_node(behavior_tree.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        behavior_tree.shutdown()
        aruco_detector.destroy_node()
        ur10_motion.destroy_node()
        ur10_sensors.destroy_node()
        ur10_gripper.destroy_node()
        send_conveyor_motion.destroy_node()  # ✅ Limpeza
        rclpy.shutdown()


if __name__ == "__main__":
    main()

