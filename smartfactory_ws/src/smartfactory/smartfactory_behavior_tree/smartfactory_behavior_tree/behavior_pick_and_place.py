# import rclpy
# from rclpy.executors import MultiThreadedExecutor
# import py_trees
# import py_trees_ros
# import sys
# import py_trees.console as console
# from py_trees.decorators import Retry, Timeout, RunningIsFailure
# from py_trees.blackboard import Blackboard

# # Importa as funções de criação das árvores dos módulos
# from .ur10_behavior import create_root as create_ur10_tree, ArucoPoseSubscriber
# from .kinect_behavior_node import create_main_tree as create_kinect_tree, KinectTreeNode
# from .kinect_behavior_node import EstadoCameraPronto, StartKinectNode

# def create_kinect_tree_with_selector(kinect_node):
#     """
#     Cria a subárvore do Kinect usando Selector e Retry para adicionar redundância.
#     """
#     root = py_trees.composites.Selector(name="KinectSelector", memory=False)
#     initialize_sequence = py_trees.composites.Sequence(name="InitializeKinect", memory=True)

#     # Nós principais
#     initialize_sequence.add_child(EstadoCameraPronto())
#     initialize_sequence.add_child(
#         py_trees.decorators.Retry(
#             child=StartKinectNode(),
#             num_failures=3,  # Número de tentativas permitidas
#             name="Retry StartKinectNode"
#         )
#     ) 

#     # Fallback (caso inicialização falhe)
#     fallback_node = py_trees.behaviours.Dummy(name="Fallback: Kinect Offline")
#     root.add_child(initialize_sequence)  # Principal
#     root.add_child(fallback_node)  # Alternativo

#     return root


# def create_ur10_tree_with_decorators(ur10_node):
#     """
#     Cria a subárvore do UR10 com Timeout e Guards.
#     """
#     root = py_trees.composites.Sequence(name="UR10Sequence", memory=True)

#    # Guard para checar poses antes de movimentar o UR10
#     check_pose_guard = py_trees.decorators.RunningIsFailure(
#         name="CheckPoseGuard",
#         child=py_trees.behaviours.CheckBlackboardVariableExists(
#             variable_name="pose",
#             name="Guard: Pose Available"
#         )
#     )


#     # Nós principais
#     root.add_child(check_pose_guard)
#     root.add_child(
#         Timeout(
#             child=py_trees.behaviours.Dummy(name="VaParaObjeto"),
#             duration=10.0,
#             name="Timeout VaParaObjeto"
#         )
#     )
#     root.add_child(py_trees.behaviours.Dummy(name="VentosaOn"))
#     root.add_child(py_trees.behaviours.Dummy(name="VaParaEsteira"))
#     root.add_child(py_trees.behaviours.Dummy(name="VentosaOff"))

#     return root

# def create_supervisor_tree(kinect_node, ur10_node):
#     """
#     Cria a árvore de comportamento principal.
#     """
#     root = py_trees.composites.Parallel(
#         name="Supervisor",
#         policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
#     )
    
#     # Adiciona as subárvores do Kinect e do UR10
#     if kinect_node:
#         kinect_tree = create_kinect_tree_with_selector(kinect_node)
#         root.add_child(kinect_tree)
#     if ur10_node:
#         ur10_tree = create_ur10_tree_with_decorators(ur10_node)
#         root.add_child(ur10_tree)
    
#     return root

# def main(args=None):
#     """
#     Função principal do supervisor.
#     """
#     rclpy.init(args=args)

#     # Cria o nó do supervisor para obter parâmetros
#     node = rclpy.create_node("supervisor_node")

#     # Obtém parâmetros do launch ou define valores padrão
#     enable_kinect = node.declare_parameter("enable_kinect", True).value
#     enable_ur10 = node.declare_parameter("enable_ur10", True).value

#     # Cria os nós com base nos parâmetros
#     kinect_node = KinectTreeNode() if enable_kinect else None
#     ur10_node = ArucoPoseSubscriber() if enable_ur10 else None

#     # Cria a árvore supervisora
#     root = create_supervisor_tree(kinect_node, ur10_node)

#     # Configura a árvore para visualização e execução
#     behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)

#     try:
#         behavior_tree.setup(timeout=15)
#     except py_trees_ros.exceptions.TimedOutError as e:
#         console.logerror(f"Failed to setup the tree: {e}")
#         cleanup(behavior_tree, kinect_node, ur10_node, node)
#         sys.exit(1)
#     except KeyboardInterrupt:
#         console.logwarn("Tree setup interrupted.")
#         cleanup(behavior_tree, kinect_node, ur10_node, node)
#         sys.exit(1)

#     # Tick-tock para atualizações automáticas da árvore
#     behavior_tree.tick_tock(period_ms=1000.0)

#     # Configura o executor multi-threaded para gerenciar todos os nós
#     executor = MultiThreadedExecutor()
#     if kinect_node:
#         executor.add_node(kinect_node)
#     if ur10_node:
#         executor.add_node(ur10_node)
#     executor.add_node(node)
#     executor.add_node(behavior_tree.node)

#     # Spin para processar os nós
#     try:
#         executor.spin()
#     except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
#         pass
#     finally:
#         cleanup(behavior_tree, kinect_node, ur10_node, node)

# def cleanup(behavior_tree, kinect_node, ur10_node, supervisor_node):
#     """
#     Função para limpar recursos e destruir nós.
#     """
#     behavior_tree.shutdown()
#     if kinect_node:
#         kinect_node.destroy_node()
#     if ur10_node:
#         ur10_node.destroy_node()
#     supervisor_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees
import py_trees_ros
import sys
import py_trees.console as console

# Importa as funções de criação das árvores dos módulos
from .behavior_ur10_node import create_root as create_ur10_tree, ArucoPoseSubscriber
from .behavior_kinect_detect import create_main_tree as create_kinect_tree, KinectTreeNode

def create_supervisor_tree(kinect_node, ur10_node):
    """
    Cria a árvore de comportamento principal.
    """
    root = py_trees.composites.Parallel(
        name="Supervisor",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    
    # Adiciona as subárvores do Kinect e do UR10
    if kinect_node:
        kinect_tree = create_kinect_tree(kinect_node)
        root.add_child(kinect_tree)
    if ur10_node:
        ur10_tree = create_ur10_tree(ur10_node)
        root.add_child(ur10_tree)
    
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
    ur10_node = ArucoPoseSubscriber() if enable_ur10 else None

    # Cria a árvore supervisora
    root = create_supervisor_tree(kinect_node, ur10_node)

    # Configura a árvore para visualização e execução
    behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        behavior_tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(f"Failed to setup the tree: {e}")
        cleanup(behavior_tree, kinect_node, ur10_node, node)
        sys.exit(1)
    except KeyboardInterrupt:
        console.logwarn("Tree setup interrupted.")
        cleanup(behavior_tree, kinect_node, ur10_node, node)
        sys.exit(1)

    # Tick-tock para atualizações automáticas da árvore
    behavior_tree.tick_tock(period_ms=1000.0)

    # Configura o executor multi-threaded para gerenciar todos os nós
    executor = MultiThreadedExecutor()
    if kinect_node:
        executor.add_node(kinect_node)
    if ur10_node:
        executor.add_node(ur10_node)
    executor.add_node(node)
    executor.add_node(behavior_tree.node)

    # Spin para processar os nós
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cleanup(behavior_tree, kinect_node, ur10_node, node)

def cleanup(behavior_tree, kinect_node, ur10_node, supervisor_node):
    """
    Função para limpar recursos e destruir nós.
    """
    behavior_tree.shutdown()
    if kinect_node:
        kinect_node.destroy_node()
    if ur10_node:
        ur10_node.destroy_node()
    supervisor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
