# import rclpy
# import subprocess
# import py_trees
# import py_trees_ros
# import sys
# import time
# import py_trees.console as console
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.node import Node
# from sensor_msgs.msg import CameraInfo  # Modificado para Image ao invés de CameraInfo
# from geometry_msgs.msg import PoseArray
# from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# class EstadoCameraPronto(py_trees.behaviour.Behaviour):
#     def __init__(self, name="CameraLigada?"):
#         super().__init__(name)

#     def update(self):
#         # Presume que a câmera está pronta, pois está conectada via Ethernet
#         self.logger.info("Câmera Basler conectada e pronta para transmitir.")
#         return py_trees.common.Status.SUCCESS

# class StartBaslerNode(py_trees.behaviour.Behaviour):
#     def __init__(self, name="IniciaBaslerROS"):
#         super().__init__(name)
#         self.process = None
#         self.already_started = False

#     def initialise(self):
#         if not self.already_started:
#             # Inicia o nó da Basler em um novo terminal xterm
#             self.process = subprocess.Popen([
#                 "xterm", "-hold", "-e", 
#                 "ros2 launch smartfactory_bringup scene.launch.py start_ur10:=false start_kinect:=false"
#             ])
#             self.already_started = True

#     def update(self):
#         if self.process and self.process.poll() is None:
#             self.logger.info("Nó da câmera Basler inicializado.")
#             return py_trees.common.Status.SUCCESS
#         elif self.process:
#             self.logger.warning("Nó da câmera Basler falhou.")
#             return py_trees.common.Status.FAILURE
#         else:
#             return py_trees.common.Status.FAILURE

# class BaslerTreeNode(Node):
#     def __init__(self):
#         super().__init__("camera_behavior_tree_node")
#         self.info_msg = None
#         self.last_msg_time = None

#         # Subscription para a imagem calibrada da câmera com QoS configurado
#         qos_profile = QoSProfile(depth=10)
#         qos_profile.reliability = ReliabilityPolicy.RELIABLE
#         qos_profile.durability = DurabilityPolicy.VOLATILE
#         self.create_subscription(CameraInfo, "/basler/camera_info", self.camera_info_callback, qos_profile)

#     def camera_info_callback(self, msg):
#         self.info_msg = msg
#         self.last_msg_time = time.time()

# class VerifyCameraInfo(py_trees.behaviour.Behaviour):
#     def __init__(self, node, name="CameraStream"):
#         super().__init__(name)
#         self.node = node
#         self.check_interval = 1

#     def update(self):
#         current_time = time.time()
#         if self.node.last_msg_time and (current_time - self.node.last_msg_time < self.check_interval):
#             self.node.get_logger().info("Câmera Basler está transmitindo dados.")
#             return py_trees.common.Status.SUCCESS
#         else:
#             self.node.get_logger().warn("Falha na transmissão de dados da câmera!")
#             return py_trees.common.Status.RUNNING
        
# class ArucoTreeNode(Node):
#     def __init__(self):
#         super().__init__("camera_behavior_tree_node")
#         self.info_msg = None
#         self.last_msg_time = None  # Armazena o timestamp da última mensagem recebida
#         self.last_msg_time_1 = None  # Timestamp para as mensagens de ArUco

#         # Subscription para informações da câmera com QoS configurado
#         qos_profile = QoSProfile(depth=10)
#         qos_profile.reliability = ReliabilityPolicy.RELIABLE
#         qos_profile.durability = DurabilityPolicy.VOLATILE
#         self.create_subscription(CameraInfo, "/basler/camera_info", self.camera_info_callback, qos_profile)

#     def camera_info_callback(self, msg):
#         self.info_msg = msg
#         self.last_msg_time = time.time()  # Atualiza o timestamp
# class VerifyCameraInfo(py_trees.behaviour.Behaviour):
#     def __init__(self, node, name="CameraStream?"):
#         super().__init__(name)
#         self.node = node
#         self.check_interval = 1  # Intervalo de verificação em segundos

#     def update(self):
#         current_time = time.time()
        
#         # Verifica se houve uma mensagem recente (nos últimos check_interval segundos)
#         if self.node.last_msg_time and (current_time - self.node.last_msg_time < self.check_interval):
#             self.node.get_logger().info("Câmera transmitindo.")
#             return py_trees.common.Status.SUCCESS
#         else:
#             self.node.get_logger().warn("Falha na transmissão da câmera!")
#             return py_trees.common.Status.RUNNING    
# class MonitorArucoPose(py_trees.behaviour.Behaviour):
#     def __init__(self, node, name="MonitorArucoPose"):
#         super().__init__(name)
#         self.node = node
#         self.pose_received = False
#         self.check_interval_1 = 1

#         # Inscrevendo-se no tópico que publica as poses do ArUco
#         self.node.create_subscription(PoseArray, "/basler/aruco/poses", self.pose_callback, 10)

#     def pose_callback(self, msg):
#         self.node.pose_array = msg
#         self.node.last_msg_time_1 = time.time()
#         self.pose_received = True

#     def update(self):
#         current_time_1 = time.time()
#         if self.node.last_msg_time_1 and (current_time_1 - self.node.last_msg_time_1 < self.check_interval_1):
#             self.node.get_logger().info("Poses do ArUco recebidas.")
#             return py_trees.common.Status.SUCCESS
#         else:
#             self.node.get_logger().warn("Sem poses do ArUco!")
#             return py_trees.common.Status.FAILURE

# class PublishResults(py_trees.behaviour.Behaviour):
#     def __init__(self, node, name="PublishResults"):
#         super().__init__(name)
#         self.node = node

#     def update(self):
#         if hasattr(self.node, 'pose_array'):
#             self.node.poses_pub.publish(self.node.pose_array)
#             return py_trees.common.Status.SUCCESS
#         return py_trees.common.Status.FAILURE

# def create_root_aruco(node) -> py_trees.behaviour.Behaviour:
#     verify_camera_sequence = py_trees.composites.Sequence(name="Sequencia", memory=False)
#     verify_camera_sequence.add_child(VerifyCameraInfo(node))
#     verify_camera_sequence.add_child(MonitorArucoPose(node))
#     return verify_camera_sequence

# def create_root_camera() -> py_trees.behaviour.Behaviour:
#     verify_camera_sequence = py_trees.composites.Sequence(name="Sequencia", memory=False)
#     verify_camera_sequence.add_child(EstadoCameraPronto())
#     verify_camera_sequence.add_child(StartBaslerNode())
#     return verify_camera_sequence

# def create_main_tree(node) -> py_trees.behaviour.Behaviour:
#     root = py_trees.composites.Parallel(name="Raiz", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
#     root.add_child(create_root_camera())
#     root.add_child(create_root_aruco(node))
#     return root

# def main(args=None):
#     rclpy.init(args=args)
#     node = BaslerTreeNode()
#     root = create_main_tree(node)
#     behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    
#     try:
#         behavior_tree.setup(timeout=15)
#     except py_trees_ros.exceptions.TimedOutError as e:
#         console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
#         behavior_tree.shutdown()
#         rclpy.try_shutdown()
#         sys.exit(1)
#     except KeyboardInterrupt:
#         console.logerror("tree setup interrupted")
#         behavior_tree.shutdown()
#         rclpy.try_shutdown()
#         sys.exit(1)

#     behavior_tree.tick_tock(period_ms=1000.0)
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)
#     executor.add_node(behavior_tree.node)

#     try:
#         executor.spin()
#     except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
#         pass
#     finally:
#         behavior_tree.shutdown()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()

import rclpy
import subprocess
import py_trees
import py_trees_ros
import sys
import time
import py_trees.console as console
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from aruco_interfaces.msg import ArucoMarkers
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class StartBaslerNode(py_trees.behaviour.Behaviour):
    def __init__(self, name="IniciaBaslerROS"):
        super().__init__(name)
        self.process = None
        self.already_started = False

    def initialise(self):
        if not self.already_started:
            self.process = subprocess.Popen([
                "xterm", "-hold", "-e", 
                "ros2 launch smartfactory_bringup scene.launch.py start_ur10:=false start_kinect:=false"
            ])
            self.already_started = True

    def update(self):
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
        self.last_msg_time = None
        self.latest_aruco_msg = None

        # Subscription para a câmera
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(CameraInfo, "/basler/camera_info", self.camera_info_callback, qos_profile)

        # Subscription para os ArUcos
        self.create_subscription(ArucoMarkers, "/basler/aruco/markers", self.aruco_callback, qos_profile)

    def camera_info_callback(self, msg):
        self.info_msg = msg
       

    def aruco_callback(self, msg):
        # Armazena a última mensagem dos ArUcos para que os comportamentos possam acessá-la
        self.latest_aruco_msg = msg
        self.last_msg_time = time.time()
        self.pose_received = True

class ArucoDetectado(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="ArucoDetectado"):
        super().__init__(name)
        self.node = node
        self.pose_received = False
        self.check_interval = 1

    def update(self):
        current_time = time.time()

        if self.node.last_msg_time and (current_time - self.node.last_msg_time < self.check_interval):
            self.node.get_logger().info("Aruco detectado!")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info("Nenhum Aruco detectado.")
            return py_trees.common.Status.FAILURE

class ArucoCount(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="ArucoCount"):
        super().__init__(name)
        self.node = node
        self.detected_arucos = set()
        self.aruco_pose_counts = {}

    def update(self):
        msg = self.node.latest_aruco_msg
        if msg and msg.poses:
            for aruco_id, pose in zip(msg.marker_ids, msg.poses):
                if aruco_id not in self.detected_arucos:
                    self.detected_arucos.add(aruco_id)
                    self.aruco_pose_counts[aruco_id] = 0
                    self.node.get_logger().info(f"Novo ArUco detectado! ID: {aruco_id}")
                self.aruco_pose_counts[aruco_id] += 1
                self.node.get_logger().info(f"ArUco ID: {aruco_id} - Contagem de poses: {self.aruco_pose_counts[aruco_id]}")
            self.node.get_logger().info(f"Total de tipos de ArUco detectados: {len(self.detected_arucos)}")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

def create_root(node) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Raiz",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    verify_camera_sequence = py_trees.composites.Sequence(name="SequenciaCamera", memory=False)
    verify_camera_sequence.add_child(StartBaslerNode())

    root.add_child(verify_camera_sequence)

    # Cria a sequência Aruco e adiciona os comportamentos ArucoDetectado e ArucoCount
    sequencia_aruco = py_trees.composites.Sequence(name="SequenciaAruco", memory=False)
    aruco_detectado = ArucoDetectado(node)
    aruco_count = ArucoCount(node)
    sequencia_aruco.add_child(aruco_detectado)
    sequencia_aruco.add_child(aruco_count)

    root.add_child(sequencia_aruco)  # Adiciona a sequência Aruco à raiz

    return root

def main(args=None):
    rclpy.init(args=args)
    node = BaslerTreeNode()
    root = create_root(node)
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



