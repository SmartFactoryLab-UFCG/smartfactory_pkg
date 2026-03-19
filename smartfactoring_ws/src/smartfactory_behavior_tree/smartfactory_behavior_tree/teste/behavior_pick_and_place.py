import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees
import py_trees_ros
import sys
import py_trees.console as console
from smartfactory_ur_utils.send_angles import UR10Controller

# Importa as funções de criação das árvores dos módulos
from .behavior_ur10_node import create_root as create_ur10_tree, ArucoPoseSubscriber
from .behavior_kinect_detect import create_main_tree as create_kinect_tree, KinectTreeNode
from smartfactory_behavior_tree.ur10.ur10_motion import SendConveyorMotion

def create_supervisor_tree(kinect_node, ur10_pose_node, ur10_controller):
    """
    Cria a árvore de comportamento principal.
    """
    root = py_trees.composites.Parallel(
        name="Supervisor",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    
    # Adiciona as subárvores do Kinect e do UR10
    # O baseline open-loop já usa a pose do Kinect diretamente na subárvore do UR10.
    # Evitamos iniciar uma segunda subárvore de bringup/câmera aqui porque o experimento
    # já sobe a cena completa externamente no Terminal A.
    if ur10_pose_node and ur10_controller:
        send_conveyor_motion = SendConveyorMotion()
        ur10_tree = create_ur10_tree(ur10_controller, send_conveyor_motion, ur10_pose_node)
        root.add_child(ur10_tree)
        root.send_conveyor_motion = send_conveyor_motion
    
    return root

def main(args=None):
    """
    Função principal do supervisor.
    """
    rclpy.init(args=args)

    # Cria o nó do supervisor para obter parâmetros
    node = rclpy.create_node("supervisor_node")

    # Obtém parâmetros do launch ou define valores padrão
    enable_kinect = node.declare_parameter("enable_kinect", True).value
    enable_ur10 = node.declare_parameter("enable_ur10", True).value

    # Cria os nós com base nos parâmetros
    kinect_node = KinectTreeNode() if enable_kinect else None
    ur10_pose_node = ArucoPoseSubscriber() if enable_ur10 else None
    ur10_controller = UR10Controller() if enable_ur10 else None

    # Cria a árvore supervisora
    root = create_supervisor_tree(kinect_node, ur10_pose_node, ur10_controller)
    send_conveyor_motion = getattr(root, "send_conveyor_motion", None)

    # Configura a árvore para visualização e execução
    behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        behavior_tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(f"Failed to setup the tree: {e}")
        cleanup(behavior_tree, kinect_node, ur10_pose_node, ur10_controller, node)
        sys.exit(1)
    except KeyboardInterrupt:
        console.logwarn("Tree setup interrupted.")
        cleanup(behavior_tree, kinect_node, ur10_pose_node, ur10_controller, node)
        sys.exit(1)

    # Tick-tock para atualizações automáticas da árvore
    behavior_tree.tick_tock(period_ms=1000.0)

    # Configura o executor multi-threaded para gerenciar todos os nós
    executor = MultiThreadedExecutor()
    if kinect_node:
        executor.add_node(kinect_node)
    if ur10_pose_node:
        executor.add_node(ur10_pose_node)
    if ur10_controller:
        executor.add_node(ur10_controller)
    if send_conveyor_motion:
        executor.add_node(send_conveyor_motion)
    executor.add_node(node)
    executor.add_node(behavior_tree.node)

    # Spin para processar os nós
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cleanup(behavior_tree, kinect_node, ur10_pose_node, ur10_controller, send_conveyor_motion, node)

def cleanup(behavior_tree, kinect_node, ur10_pose_node, ur10_controller, send_conveyor_motion, supervisor_node):
    """
    Função para limpar recursos e destruir nós.
    """
    behavior_tree.shutdown()
    if kinect_node:
        kinect_node.destroy_node()
    if ur10_pose_node:
        ur10_pose_node.destroy_node()
    if ur10_controller:
        ur10_controller.destroy_node()
    if send_conveyor_motion:
        send_conveyor_motion.destroy_node()
    supervisor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
