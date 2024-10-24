#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees_ros.trees
import py_trees.visitors
import py_trees.blackboard
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray

# Nodo principal que gerencia os comportamentos do Kinect e ArUco
class KinectBehaviorNode(Node):
    def __init__(self):
        super().__init__('kinect_behavior_node')
        
        # Assinando ao tópico do ArUco para monitorar poses
        self.aruco_pose_subscriber = self.create_subscription(
            PoseArray,
            '/aruco/poses',
            self.aruco_pose_callback,
            10)
        
        # Armazena a pose do ArUco detectado
        self.aruco_pose = None
        self.get_logger().info("Kinect Behavior Node iniciado.")

    def aruco_pose_callback(self, msg):
        # Callback que é acionado quando uma nova pose é publicada no tópico /aruco/poses
        self.aruco_pose = msg.poses
        self.get_logger().info(f"Aruco detectado: {len(self.aruco_pose)} poses")

    def has_detected_aruco(self):
        # Verifica se o ArUco foi detectado
        return self.aruco_pose is not None

# Behavior para verificar se o Kinect está publicando dados
class WaitForArucoPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(WaitForArucoPose, self).__init__(name)
        self.node = node

    def update(self):
        # Verifica se a pose do ArUco foi detectada
        if self.node.has_detected_aruco():
            self.node.get_logger().info("Aruco detectado, pronto para capturar.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn("Aguardando detecção do ArUco...")
            return py_trees.common.Status.RUNNING

# Behavior para capturar as informações de pose do ArUco
class CaptureArucoPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(CaptureArucoPose, self).__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        # Captura e processa as poses dos ArUcos detectados
        if self.node.aruco_pose:
            self.blackboard.aruco_pose = self.node.aruco_pose
            self.node.get_logger().info(f"Capturando a pose do ArUco: {self.node.aruco_pose}")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn("Nenhuma pose do ArUco disponível.")
            return py_trees.common.Status.FAILURE

# Função para criar a árvore de comportamento
def create_behavior_tree(node):
    # Criar o nó raiz como Parallel para maior flexibilidade
    root = py_trees.composites.Parallel(name="Kinect ArUco Parallel", 
                                        policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    # Nó para verificar se o ArUco foi detectado
    wait_for_aruco_pose = WaitForArucoPose("Aguardar Detecção do ArUco", node)

    # Nó para capturar as poses do ArUco
    capture_aruco_pose = CaptureArucoPose("Capturar Pose do ArUco", node)

    # Nó Idle para caso o ArUco não seja detectado
    idle = py_trees.behaviours.Running(name="Idle")

    # Fallback para alternar entre detecção do ArUco e estado Idle
    fallback = py_trees.composites.Selector(name="Detecção ArUco ou Idle", memory=False)
    fallback.add_children([wait_for_aruco_pose, idle])

    # Adicionar os comportamentos à árvore raiz
    root.add_children([fallback, capture_aruco_pose])

    # Criar a árvore de comportamento
    tree = py_trees.trees.BehaviourTree(root)

    # Habilitar a transmissão de snapshots para o PyTrees Viewer via ROS
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree.visitors.append(snapshot_visitor)

    return tree

def main(args=None):
    rclpy.init(args=args)
    node = KinectBehaviorNode()

    # Criar a árvore de comportamento
    tree = create_behavior_tree(node)

    # Ativar a visualização do Blackboard
    py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)

    # Executor para a árvore e o nó
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        tree.setup(timeout=15)
        while rclpy.ok():
            # Tick da árvore
            tree.tick()
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()














































# -------- old code -----------

# import rclpy
# from rclpy.node import Node
# import py_trees
# import py_trees_ros
# from std_msgs.msg import String
# from geometry_msgs.msg import PoseArray

# # Nodo principal que gerencia os comportamentos do Kinect e ArUco
# class KinectBehaviorNode(Node):
#     def __init__(self):
#         super().__init__('kinect_behavior_node')
        
#         # Assinando ao tópico do ArUco para monitorar poses
#         self.aruco_pose_subscriber = self.create_subscription(
#             PoseArray,
#             '/aruco/poses',
#             self.aruco_pose_callback,
#             10)
        
#         # Armazena a pose do ArUco detectado
#         self.aruco_pose = None
#         self.get_logger().info("Kinect Behavior Node iniciado.")

#     def aruco_pose_callback(self, msg):
#         # Callback que é acionado quando uma nova pose é publicada no tópico /aruco/poses
#         self.aruco_pose = msg.poses
#         self.get_logger().info(f"Aruco detectado: {len(self.aruco_pose)} poses")

#     def has_detected_aruco(self):
#         # Verifica se o ArUco foi detectado
#         return self.aruco_pose is not None

# # Behavior para verificar se o Kinect está publicando dados
# class WaitForArucoPose(py_trees.behaviour.Behaviour):
#     def __init__(self, name, node):
#         super(WaitForArucoPose, self).__init__(name)
#         self.node = node

#     def update(self):
#         # Verifica se a pose do ArUco foi detectada
#         if self.node.has_detected_aruco():
#             self.node.get_logger().info("Aruco detectado, pronto para capturar.")
#             return py_trees.common.Status.SUCCESS
#         else:
#             self.node.get_logger().warn("Aguardando detecção do ArUco...")
#             return py_trees.common.Status.RUNNING

# # Behavior para capturar as informações de pose do ArUco
# class CaptureArucoPose(py_trees.behaviour.Behaviour):
#     def __init__(self, name, node):
#         super(CaptureArucoPose, self).__init__(name)
#         self.node = node

#     def update(self):
#         # Captura e processa as poses dos ArUcos detectados
#         if self.node.aruco_pose:
#             self.node.get_logger().info(f"Capturando a pose do ArUco: {self.node.aruco_pose}")
#             return py_trees.common.Status.SUCCESS
#         else:
#             self.node.get_logger().warn("Nenhuma pose do ArUco disponível.")
#             return py_trees.common.Status.FAILURE

# # Função para criar a árvore de comportamento
# def create_behavior_tree(node):
#     # Adicione o argumento memory à criação da sequência
#     root = py_trees.composites.Sequence(name="Kinect ArUco Sequence", memory=True)

#     # Adiciona os behaviours à árvore
#     wait_for_aruco_pose = WaitForArucoPose("Aguardar Detecção do ArUco", node)
#     capture_aruco_pose = CaptureArucoPose("Capturar Pose do ArUco", node)

#     root.add_children([wait_for_aruco_pose, capture_aruco_pose])

#     return py_trees.trees.BehaviourTree(root)

# def main(args=None):
#     rclpy.init(args=args)
#     node = KinectBehaviorNode()

#     # Cria a árvore de comportamento
#     tree = create_behavior_tree(node)

#     # Integração com o py_trees.viewer
#     py_trees.display.render_dot_tree(tree.root)
#     print(py_trees.display.ascii_tree(tree.root))

#     # Conexão com o visualizador py_trees.viewer
#     py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
#     snapshot_visitor = py_trees.visitors.SnapshotVisitor()
#     tree.visitors.append(snapshot_visitor)

#     # Executor para a árvore e o nó
#     executor = rclpy.executors.SingleThreadedExecutor()
#     executor.add_node(node)

#     try:
#         tree.setup(timeout=15)
#         while rclpy.ok():
#             tree.tick()
#             rclpy.spin_once(node, timeout_sec=1.0)
#     except KeyboardInterrupt:
#         node.get_logger().info("Encerrando...")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
