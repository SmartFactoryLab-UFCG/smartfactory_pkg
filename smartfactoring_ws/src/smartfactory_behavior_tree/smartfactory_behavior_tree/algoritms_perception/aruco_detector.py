import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import py_trees
import time

class ArucoPoseSubscriber(Node):
    """
    Classe responsável por receber a pose dos marcadores ArUco detectados.
    - Subscrição ao tópico de poses do ArUco.
    - Armazena a pose mais recente do marcador.
    """

    def __init__(self):
        super().__init__("aruco_pose_subscriber")
        self.aruco_pose = None
        self.last_msg_time = None  # Timestamp para verificar a última mensagem recebida

        # Subscrição ao tópico que publica as poses do ArUco detectado
        self.aruco_sub = self.create_subscription(
            PoseArray, 
            '/kinect/aruco/filtered_poses', 
            self.aruco_pose_callback, 
            10
        )

    def aruco_pose_callback(self, msg):
        """Atualiza a pose do marcador ArUco sempre que recebe uma nova mensagem."""
        self.aruco_pose = msg
        self.last_msg_time = time.time()
        #self.get_logger().info("📡 Nova pose do ArUco recebida!")

    def has_recent_detection(self, timeout=1.0):
        """
        Verifica se houve uma detecção recente do ArUco.
        Retorna True se a última mensagem foi recebida dentro do intervalo de timeout.
        """
        return self.last_msg_time and (time.time() - self.last_msg_time < timeout)

class CheckArucoPose(py_trees.behaviour.Behaviour):
    """
    Comportamento da árvore de comportamento para verificar a detecção do ArUco.
    """

    def __init__(self, aruco_subscriber: ArucoPoseSubscriber, name="CheckArucoPose"):
        super().__init__(name)
        self.aruco_subscriber = aruco_subscriber
        self.check_interval = 1  # Tempo máximo para considerar uma detecção válida

    def update(self):
        """Verifica se o ArUco foi detectado recentemente."""
        if self.aruco_subscriber.has_recent_detection(self.check_interval):
            self.logger.info("ArUco detectado!")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("Nenhuma detecção recente do ArUco. Aguardando...")
            return py_trees.common.Status.FAILURE
