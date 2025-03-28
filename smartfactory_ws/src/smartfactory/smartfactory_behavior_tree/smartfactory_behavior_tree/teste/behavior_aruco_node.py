import rclpy
import py_trees
import py_trees_ros
import sys
import time
import py_trees.console as console
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class ArucoTreeNode(Node):
    def __init__(self):
        super().__init__("camera_behavior_tree_node")
        self.info_msg = None
        self.last_msg_time = None  # Armazena o timestamp da última mensagem recebida
        self.last_msg_time_1 = None  # Timestamp para as mensagens de ArUco

        # Subscription para informações da câmera com QoS configurado
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.camera_info_callback, qos_profile)

    def camera_info_callback(self, msg):
        self.info_msg = msg
        self.last_msg_time = time.time()  # Atualiza o timestamp

class VerifyCameraInfo(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="CameraStream?"):
        super().__init__(name)
        self.node = node
        self.check_interval = 1  # Intervalo de verificação em segundos

    def update(self):
        current_time = time.time()
        
        # Verifica se houve uma mensagem recente (nos últimos `check_interval` segundos)
        if self.node.last_msg_time and (current_time - self.node.last_msg_time < self.check_interval):
            self.node.get_logger().info("Câmera transmitindo.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn("Falha na transmissão da câmera!")
            return py_trees.common.Status.RUNNING

class MonitorArucoPose(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="MonitorArucoPose"):
        super().__init__(name)
        self.node = node
        self.pose_received = False
        self.check_interval_1 = 1  # Intervalo de verificação em segundos

        # Inscrevendo-se no tópico que publica as poses do ArUco
        self.node.create_subscription(PoseArray, "/kinect/aruco/poses", self.pose_callback, 10)

    def pose_callback(self, msg):
        # Armazena as poses recebidas para futura publicação
        self.node.pose_array = msg
        self.node.last_msg_time_1 = time.time()  # Atualiza o timestamp
        self.pose_received = True

    def update(self):
        current_time_1 = time.time()
        if self.node.last_msg_time_1 and (current_time_1 - self.node.last_msg_time_1 < self.check_interval_1):
            self.node.get_logger().info("Poses do ArUco recebidas.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn("Sem poses do ArUco!")
            return py_trees.common.Status.FAILURE

class PublishResults(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="PublishResults"):
        super().__init__(name)
        self.node = node

    def update(self):
        if hasattr(self.node, 'pose_array'):
            self.node.poses_pub.publish(self.node.pose_array)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

def create_root(node) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Raiz",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    # Sequência para verificar informações da câmera
    verify_camera_sequence = py_trees.composites.Sequence(name="Sequencia", memory=False)
    verify_camera_sequence.add_child(VerifyCameraInfo(node))
    verify_camera_sequence.add_child(MonitorArucoPose(node))

    # Adiciona a sequência ao nó paralelo
    root.add_child(verify_camera_sequence)

    return root

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTreeNode()
    # Criação da árvore de comportamento
    root = create_root(node)

    # Configuração da árvore para visualização e execução
    behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    
    try:
        behavior_tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        behavior_tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        behavior_tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    # Configura o tick_tock para atualizações automáticas da árvore
    behavior_tree.tick_tock(period_ms=1000.0)

    # Configura o executor multi-threaded para gerenciar o behavior_tree e o nó principal
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(behavior_tree.node)

    # Spin com o MultiThreadedExecutor para processar ambos os nós
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        behavior_tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
