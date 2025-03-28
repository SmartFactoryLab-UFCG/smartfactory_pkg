import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class UR10Sensors(Node):
    """
    Classe responsável por lidar com os sensores do UR10.
    - Monitora o sensor ultrassônico da garra.
    - Verifica se há um objeto detectado ou não.
    """

    def __init__(self):
        super().__init__("ur10_sensors_node")
        self.sensor_status = False  # Estado inicial: sem objeto detectado
        
        # Subscrição ao tópico de status do sensor ultrassônico
        self.subscription = self.create_subscription(
            Bool,
            '/vacuum_gripper_status',  # Tópico onde o status do sensor é publicado
            self.sensor_callback,
            10
        )

    def sensor_callback(self, msg):
        """Recebe um Bool (True/False) indicando se há um objeto detectado pela ventosa."""
        self.sensor_status = msg.data
        if msg.data != self.sensor_status:
            self.sensor_status = msg.data
            estado = "OBJETO DETECTADO" if self.sensor_status else "VENTOSA VAZIA"
            self.get_logger().info(f"📡 Atualização do sensor: {estado}")

    def is_object_detected(self):
        """Retorna True se um objeto estiver sendo segurado pela ventosa."""
        return self.sensor_status

# class CheckUltrasonicGripper(py_trees.behaviour.Behaviour):
#     """
#     Comportamento da árvore de comportamento para verificar se a ventosa está próxima o suficiente de um objeto.
#     Essa verificação é feita assumindo que o sensor ultrassónico está detectando o objeto
#     """

#     def __init__(self, sensors: UR10Sensors, name="CheckUltrasonicGripper"):
#         super().__init__(name)
#         self.sensors = sensors

#     def update(self):
#         """Verifica se o sensor detecta um objeto."""
#         if self.sensors.is_object_detected():  
#             self.logger.info("Objeto detectado pelo sensor. Pronto para continuar.")
#             return py_trees.common.Status.SUCCESS
#         else:
#             self.logger.info("Nenhum objeto detectado pelo sensor. Aguardando.")
#             return py_trees.common.Status.FAILURE

class CheckUltrasonicGripper(py_trees.behaviour.Behaviour):
    def __init__(self, sensors: UR10Sensors, name="CheckUltrasonicGripper", timeout=5.0):
        super().__init__(name)
        self.sensors = sensors
        self.timeout = timeout
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()
        self.feedback_message = "Esperando detecção do objeto..."

    def update(self):
        if self.sensors.is_object_detected():
            self.feedback_message = "Objeto detectado pela ventosa."
            return py_trees.common.Status.SUCCESS
        
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            self.feedback_message = "Timeout! Objeto não foi detectado."
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"Aguardando... ({elapsed:.1f}s)"
        return py_trees.common.Status.RUNNING
