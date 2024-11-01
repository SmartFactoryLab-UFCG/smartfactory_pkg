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
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class ArucoPoseSubscriber(Node):
    def __init__(self):
        super().__init__("ur10_behavior_tree_node")
        self.aruco_pose = None
        self.last_msg_time = None  # Timestamp para verificar a última mensagem recebida

        # Subscrição ao tópico de poses do ArUco
        self.aruco_sub = self.create_subscription(
            PoseArray, 
            '/aruco/filtered_poses', 
            self.aruco_pose_callback, 
            10
        )

    def aruco_pose_callback(self, msg):
        self.aruco_pose = msg
        self.last_msg_time = time.time()  # Atualiza o timestamp
        self.get_logger().info("Pose do ArUco recebida.")

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


