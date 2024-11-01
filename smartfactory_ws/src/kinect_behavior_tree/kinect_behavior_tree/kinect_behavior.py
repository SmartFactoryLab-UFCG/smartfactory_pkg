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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class EstadoCameraPronto(py_trees.behaviour.Behaviour):
    def __init__(self, name="CameraLigada?"):
        super().__init__(name)

    def update(self):
        # Executa o comando lsusb
        result = subprocess.run(['lsusb'], stdout=subprocess.PIPE)
        usb_devices = result.stdout.decode('utf-8')

        # Verifica se o Kinect está conectado pelo nome do dispositivo
        if "Microsoft Xbox NUI Camera" in usb_devices:
            self.logger.info("Kinect está ligado e pronto para transmitir.")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.warning("O Kinect não está ligado.")
            return py_trees.common.Status.FAILURE
        
class StartKinectNode(py_trees.behaviour.Behaviour):
    def __init__(self, name="IniciaKinectROS"):
        super().__init__(name)
        self.process = None
        self.already_started = False  # Flag para garantir que seja executado apenas uma vez

    def initialise(self):
        # Inicia o nó do Kinect em um novo terminal xterm apenas se ainda não foi iniciado
        if not self.already_started:
            self.process = subprocess.Popen([
                "xterm", "-hold", "-e", 
                "ros2 launch smartfactory_bringup scene.launch.py start_ur10:=false start_basler:=false"
            ])
            self.already_started = True  # Define a flag para indicar que o nó foi iniciado

    def update(self):
        # Verifica se o processo foi iniciado e ainda está em execução
        if self.process and self.process.poll() is None:
            self.logger.info("Nó do Kinect inicializado.")
            return py_trees.common.Status.SUCCESS
        elif self.process:
            self.logger.warning("Nó do Kinect falhou.")
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.FAILURE

class KinectTreeNode(Node):
    def __init__(self):
        super().__init__("camera_behavior_tree_node")
        self.info_msg = None
        self.last_msg_time = None  # Armazena o timestamp da última mensagem recebida

        # Subscription para informações da câmera com QoS configurado
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.camera_info_callback, qos_profile)

    def camera_info_callback(self, msg):
        self.info_msg = msg
        self.last_msg_time = time.time()  # Atualiza o timestamp

class VerifyCameraInfo(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="CameraStream"):
        super().__init__(name)
        self.node = node
        self.check_interval = 1  # Intervalo de verificação em segundos

    def update(self):
        current_time = time.time()
        
        # Verifica se houve uma mensagem recente (nos últimos `check_interval` segundos)
        if self.node.last_msg_time and (current_time - self.node.last_msg_time < self.check_interval):
        # if self.node.info_msg:
            self.node.get_logger().info("Camera is actively streaming data.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn("No recent camera info received!")
            return py_trees.common.Status.RUNNING

def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Raiz",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    # Sequência para verificar informações da câmera
    verify_camera_sequence = py_trees.composites.Sequence(name="Sequencia", memory=False)
    verify_camera_sequence.add_child(EstadoCameraPronto())
    verify_camera_sequence.add_child(StartKinectNode())

    # Adiciona ambas as sequências ao nó paralelo
    root.add_child(verify_camera_sequence)
    return root

def main(args=None):
    rclpy.init(args=args)
    node = KinectTreeNode()
    # Criação da árvore de comportamento
    root = create_root()

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
        # not a warning, nor error, usually a user-initiated shutdown
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


