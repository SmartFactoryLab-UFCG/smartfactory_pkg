import rclpy
import subprocess
import py_trees
import py_trees_ros
import sys
import time
import py_trees.console as console
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class EstadoCameraPronto(py_trees.behaviour.Behaviour):
    def __init__(self, name="CameraLigada?"):
        super().__init__(name)

    def update(self):
        # Aqui se considera que a câmera Basler já está configurada e pronta para transmitir
        # Portanto, retornamos sucesso diretamente para simplificação
        self.logger.info("A câmera Basler está pronta para transmitir.")
        return py_trees.common.Status.SUCCESS

class StartBaslerNode(py_trees.behaviour.Behaviour):
    def __init__(self, name="IniciaBaslerROS"):
        super().__init__(name)
        self.process = None
        self.already_started = False  # Flag para garantir que seja executado apenas uma vez

    def initialise(self):
        # Inicia o nó da Basler em um novo terminal xterm apenas se ainda não foi iniciado
        if not self.already_started:
            self.process = subprocess.Popen([
                "xterm", "-hold", "-e", 
                "ros2 launch pylon_ros2_camera pylon_camera.launch.py"
            ])
            self.already_started = True  # Define a flag para indicar que o nó foi iniciado

    def update(self):
        # Verifica se o processo foi iniciado e ainda está em execução
        if self.process and self.process.poll() is None:
            self.logger.info("Nó da câmera Basler inicializado.")
            return py_trees.common.Status.SUCCESS
        elif self.process:
            self.logger.warning("Nó da câmera Basler falhou.")
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.FAILURE

class BaslerTreeNode(Node):
    def __init__(self):
        super().__init__("camera_behavior_tree_node")
        self.info_msg = None
        self.last_msg_time = None  # Armazena o timestamp da última mensagem recebida
        self.aruco_pose_counts = {}  # Contagem de poses dos ArUcos
        self.max_ids = 7  # Limite de IDs únicos de ArUcos a serem monitorados

        # Subscription para informações da câmera com QoS configurado
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(CameraInfo, "/basler/image_rect", self.camera_info_callback, qos_profile)
        
        # Subscription para informações de ArUcos detectados
        self.create_subscription(ArucoMarkers, "/basler/aruco/markers", self.aruco_callback, 10)

    def camera_info_callback(self, msg):
        self.info_msg = msg
        self.last_msg_time = time.time()  # Atualiza o timestamp
        self.get_logger().info("Imagem recebida no callback de /basler/image_rect")

    def aruco_callback(self, msg):
        if not msg.poses:
            return  # Ignora se não houver ArUcos

        for aruco_id, pose in zip(msg.marker_ids, msg.poses):
            # Limita o rastreamento a um máximo de 7 IDs únicos
            if aruco_id not in self.aruco_pose_counts:
                if len(self.aruco_pose_counts) >= self.max_ids:
                    self.get_logger().info(f"Limite de {self.max_ids} IDs de ArUcos alcançado. Ignorando ArUco ID: {aruco_id}")
                    continue
                self.aruco_pose_counts[aruco_id] = 0  # Inicializa o contador para o novo ArUco ID

            self.aruco_pose_counts[aruco_id] += 1  # Incrementa a contagem do ArUco
            self.get_logger().info(f"ArUco ID: {aruco_id} detectado {self.aruco_pose_counts[aruco_id]} vezes.")


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

class MonitorArucoPoseCount(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="MonitorArucoPoseCount"):
        super().__init__(name)
        self.node = node

    def update(self):
        if self.node.aruco_pose_counts:
            for aruco_id, count in self.node.aruco_pose_counts.items():
                self.node.get_logger().info(f"ArUco ID: {aruco_id} foi detectado {count} vezes.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn("Nenhum ArUco detectado.")
            return py_trees.common.Status.FAILURE

def create_root(node) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Raiz",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    
    verify_camera_sequence = py_trees.composites.Sequence(name="Sequencia", memory=False)
    verify_camera_sequence.add_child(EstadoCameraPronto())
    verify_camera_sequence.add_child(StartBaslerNode())
    verify_camera_sequence.add_child(VerifyCameraInfo(node))
    verify_camera_sequence.add_child(MonitorArucoPoseCount(node))

    root.add_child(verify_camera_sequence)
    return root

def main(args=None):
    rclpy.init(args=args)
    node = BaslerTreeNode()
    root = create_root(node)

    behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    
    try:
        behavior_tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "Falha ao configurar a árvore, abortando [{}]".format(str(e)) + console.reset)
        behavior_tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("Configuração da árvore interrompida")
        behavior_tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    behavior_tree.tick_tock(period_ms=1000.0)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(behavior_tree.node)

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
