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
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration

class ArucoPoseSubscriber(Node):
    def __init__(self):
        super().__init__("ur10_behavior_tree_node")
        self.aruco_pose = None
        self.last_msg_time = None  # Timestamp para verificar a última mensagem recebida

        # Subscrição ao tópico de poses do ArUco
        self.aruco_sub = self.create_subscription(
            PoseArray, 
            '/kinect/aruco/filtered_poses', 
            self.aruco_pose_callback, 
            10
        )

    def aruco_pose_callback(self, msg):
        self.aruco_pose = msg
        self.last_msg_time = time.time()  # Atualiza o timestamp
        # self.get_logger().info("Pose do ArUco recebida.")

class CheckArucoPose(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="CheckArucoPose"):
        super().__init__(name)
        self.node = node
        self.check_interval = 1  # Intervalo de verificação em segundos

    def update(self):
        current_time = time.time()
        
        # Verifica se houve uma mensagem recente (nos últimos `check_interval` segundos)
        if self.node.last_msg_time and (current_time - self.node.last_msg_time < self.check_interval):
            self.node.get_logger().info("Pose do ArUco está disponível.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn("Pose do ArUco não está disponível!")
            return py_trees.common.Status.RUNNING
        
class SendJointAnglesAction(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="VaParaObjeto"):
        super(SendJointAnglesAction, self).__init__(name)
        self.node = node

    def initialise(self):
        # Chama o nó `send_angles` como um subprocesso
        self.process = subprocess.Popen(['ros2', 'run', 'smartfactory_simulation', 'send_angles'])
        self.node.get_logger().info("Nó de envio de ângulos de junta iniciado.")

    def update(self):
        # Verifica se o processo ainda está rodando
        if self.process.poll() is None:
            return py_trees.common.Status.RUNNING
        elif self.process.returncode == 0:
            self.node.get_logger().info("Envio de ângulos concluído com sucesso.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error("Erro no envio de ângulos.")
            return py_trees.common.Status.FAILURE

class VentosaOn(py_trees.behaviour.Behaviour):
    def __init__(self, name="VentosaOn"):
        super().__init__(name)
        self.process = None

    def initialise(self):
        self.process = subprocess.Popen(['ros2', 'run', 'smartfactory_simulation', 'ventosa_on'])
        self.feedback_message = "Ativando ventosa..."

    def update(self):
        if self.process.poll() is None:
            return py_trees.common.Status.RUNNING
        elif self.process.returncode == 0:
            self.feedback_message = "Ventosa ativada com sucesso."
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Erro ao ativar a ventosa."
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()

# Comportamento para enviar o comando da esteira
class SendConveyor(py_trees.behaviour.Behaviour):
    def __init__(self, name="VaParaEsteira"):
        super().__init__(name)
        self.process = None

    def initialise(self):
        self.process = subprocess.Popen(['ros2', 'run', 'smartfactory_simulation', 'send_conveyor'])
        self.feedback_message = "Enviando comando para a esteira..."

    def update(self):
        if self.process.poll() is None:
            return py_trees.common.Status.RUNNING
        elif self.process.returncode == 0:
            self.feedback_message = "Comando da esteira enviado com sucesso."
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Erro ao enviar comando da esteira."
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()

# Comportamento para desativar a ventosa
class VentosaOff(py_trees.behaviour.Behaviour):
    def __init__(self, name="VentosaOff"):
        super().__init__(name)
        self.process = None

    def initialise(self):
        self.process = subprocess.Popen(['ros2', 'run', 'smartfactory_simulation', 'ventosa_off'])
        self.feedback_message = "Desativando ventosa..."

    def update(self):
        if self.process.poll() is None:
            return py_trees.common.Status.RUNNING
        elif self.process.returncode == 0:
            self.feedback_message = "Ventosa desativada com sucesso."
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Erro ao desativar a ventosa."
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()

def create_root(node):
    # Instancia os comportamentos na sequência especificada
    root = py_trees.composites.Parallel(
        name="Raiz",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    check_aruco = CheckArucoPose(node)
    # calculate_kinematics = CalculateInverseKinematics(node)
    send_angles = SendJointAnglesAction(node)
    ventosa_on = VentosaOn()
    send_conveyor = SendConveyor()
    ventosa_off = VentosaOff()
    
    # Configura a sequência de inicialização na árvore
    seq = py_trees.composites.Sequence(name="Sequencia", memory=True)
    seq.add_children([check_aruco, send_angles, ventosa_on, send_conveyor, ventosa_off])
    
    root.add_child(seq)
    return root           

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseSubscriber()
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


