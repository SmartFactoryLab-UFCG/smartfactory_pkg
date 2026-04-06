#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
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
        self.aruco_pose = None  # Pose do target (world frame)
        self.last_msg_time = None  # Timestamp para verificar a última mensagem recebida

        self.declare_parameter('input_topic', '/kinect/aruco/filtered_pose')
        self.declare_parameter('fallback_pose_array_topic', '/kinect/aruco/filtered_poses')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.fallback_pose_array_topic = (
            self.get_parameter('fallback_pose_array_topic').get_parameter_value().string_value
        )

        # Preferir PoseStamped (uma pose do target); manter fallback para PoseArray (compatibilidade)
        self.aruco_sub_pose = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.aruco_pose_stamped_callback,
            10,
        )
        self.aruco_sub_array = self.create_subscription(
            PoseArray,
            self.fallback_pose_array_topic,
            self.aruco_pose_array_callback,
            10,
        )

    def aruco_pose_stamped_callback(self, msg: PoseStamped):
        """Atualiza a pose do ArUco target sempre que recebe uma nova mensagem."""
        self.aruco_pose = msg.pose
        self.last_msg_time = time.time()

    def aruco_pose_array_callback(self, msg: PoseArray):
        """Fallback: usa a primeira pose do PoseArray, se existir."""
        if not msg.poses:
            return
        self.aruco_pose = msg.poses[0]
        self.last_msg_time = time.time()

    def has_recent_detection(self, timeout=1.0):
        """
        Verifica se houve uma detecção recente do ArUco.
        Retorna True se a última mensagem foi recebida dentro do intervalo de timeout.
        """
        return self.last_msg_time and (time.time() - self.last_msg_time < timeout)

    def get_target_pose(self) -> Pose | None:
        return self.aruco_pose

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
