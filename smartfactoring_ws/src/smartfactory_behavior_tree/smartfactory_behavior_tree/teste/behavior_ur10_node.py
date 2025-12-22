import rclpy
import subprocess
import py_trees
import py_trees_ros
import sys
import os
import time
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from smartfactory_ur_utils.send_angles import UR10Controller
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String  # Mudamos para String
from rclpy.action import ActionClient

def run_ros2_command(package, executable):
    """
    Executa um comando `ros2 run` garantindo que ele herde corretamente o ROS_DOMAIN_ID.
    
    Args:
        package (str): Nome do pacote ROS2.
        executable (str): Nome do nó/executável a ser rodado.
    
    Returns:
        subprocess.Popen: Processo iniciado.
    """
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "10")  # Obtém o ROS_DOMAIN_ID do ambiente (padrão 10)
    env = os.environ.copy()  # Copia o ambiente atual
    env["ROS_DOMAIN_ID"] = ros_domain_id  # Garante que o subprocesso roda no mesmo domínio

    process = subprocess.Popen(['ros2', 'run', package, executable], env=env)

    print(f"Executando '{package} {executable}' com ROS_DOMAIN_ID={ros_domain_id}")
    return process

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

class CheckArucoPose(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="CheckArucoPose"):
        super().__init__(name)
        self.node = node
        self.check_interval = 1  # Intervalo de verificação em segundos

    def update(self):
        current_time = time.time()
        if self.node.last_msg_time and (current_time - self.node.last_msg_time < self.check_interval):
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class CheckUltrasonicGripper(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="CheckUltrasonicGripper"):
        super().__init__(name)
        self.node = node
        self.sensor_status = False  # Estado inicial é False (VENTOSA VAZIA)
        
        # Corrigindo a assinatura do tópico para receber Bool
        self.subscription = self.node.create_subscription(
            Bool,
            '/vacuum_gripper_status',
            self.sensor_callback,
            10
        )

    def sensor_callback(self, msg):
        """Recebe um Bool (True/False) indicando o status da ventosa."""
        self.sensor_status = msg.data
        estado = "OBJETO DETECTADO" if self.sensor_status else "VENTOSA VAZIA"
        #self.node.get_logger().info(f"📡 Atualização da ventosa: {estado}")

    def update(self):
        """Verifica se a ventosa detectou um objeto."""
        if self.sensor_status:  # Se for True, objeto foi detectado
            #self.node.get_logger().info("✅ Objeto detectado pela ventosa. Pronto para ativação.")
            return py_trees.common.Status.SUCCESS
        else:
            #self.node.get_logger().warn("⚠️ Nenhum objeto detectado pela ventosa.")
            return py_trees.common.Status.FAILURE

class MoveUR10ToPose(py_trees.behaviour.Behaviour):
    def __init__(self, ur10_controller, node, name="MoveUR10ToPose"):
        super().__init__(name)
        self.ur10_controller = ur10_controller
        self.node = node
        self.goal_sent = False
        self.joint_angles = None

        # Subscrição ao tópico dos ângulos calculados
        self.subscription = self.node.create_subscription(
            Float64MultiArray,
            '/ur10/calculated_joint_angles',
            self.joint_angles_callback,
            10
        )

    def joint_angles_callback(self, msg):
        """Armazena os ângulos das juntas recebidos da cinemática inversa."""
        self.joint_angles = msg.data

    def initialise(self):
        """Envia os ângulos quando a árvore inicia esse comportamento."""
        if not self.goal_sent and self.joint_angles is not None:
            self.ur10_controller.send_joint_angles(self.joint_angles)
            self.goal_sent = True
            self.node.get_logger().info("📤 Enviando ângulos calculados para o UR10...")

    def update(self):
        """Monitora se o movimento foi concluído."""
        if self.ur10_controller.sending_goal:
            return py_trees.common.Status.RUNNING

        if self.ur10_controller.last_error_code is not None and self.ur10_controller.last_error_code != 0:
            self.node.get_logger().error(f"❌ Movimento falhou! Código de erro: {self.ur10_controller.last_error_code}")
            return py_trees.common.Status.FAILURE

        self.node.get_logger().info("✅ UR10 atingiu a posição final!")
        return py_trees.common.Status.SUCCESS

class VentosaOnAction(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="VentosaOnAction"):
        super().__init__(name)
        self.node = node
        self.action_client = ActionClient(self.node, Trigger, 'ventosa_on_action')
        self.goal_handle = None
        self.sent_goal = False

    def initialise(self):
        """Envia a ação para ativar a ventosa."""
        self.node.get_logger().info("🔵 Solicitando ativação da ventosa via Action...")
        self.sent_goal = False
        self.action_client.wait_for_server()

        goal_msg = Trigger.Goal()
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Verifica se a action foi aceita pelo servidor."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().error("❌ Ação de ativação da ventosa rejeitada!")
        else:
            self.node.get_logger().info("✅ Ação de ativação da ventosa aceita!")
            self.sent_goal = True

    def update(self):
        """Monitora o status da ação e verifica se a ventosa foi ativada com sucesso."""
        if not self.sent_goal or not self.goal_handle:
            return py_trees.common.Status.RUNNING

        if self.goal_handle.status == 4:  # Sucesso
            self.node.get_logger().info("✅ Ventosa ativada com sucesso!")
            return py_trees.common.Status.SUCCESS
        elif self.goal_handle.status in [5, 6]:  # Abortado ou rejeitado
            self.node.get_logger().error("❌ Erro ao ativar a ventosa!")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING
class VentosaOn(py_trees.behaviour.Behaviour):
    def __init__(self, name="VentosaOn"):
        super().__init__(name)
        self.process = None

    def initialise(self):
        self.process = run_ros2_command('smartfactory_ur_utils', 'ventosa_on')

    def update(self):
        if self.process.poll() is None:
            return py_trees.common.Status.RUNNING
        elif self.process.returncode == 0:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()

# class SendConveyor(py_trees.behaviour.Behaviour):
#     def __init__(self, name="VaParaEsteira"):
#         super().__init__(name)
#         self.process = None

#     def initialise(self):
#         self.process = run_ros2_command('smartfactory_ur_utils', 'send_conveyor')


#     def update(self):
#         if self.process.poll() is None:
#             return py_trees.common.Status.RUNNING
#         elif self.process.returncode == 0:
#             return py_trees.common.Status.SUCCESS
#         else:
#             return py_trees.common.Status.FAILURE

#     def terminate(self, new_status):
#         if self.process and self.process.poll() is None:
#             self.process.terminate()
#             self.process.wait()

class VentosaOff(py_trees.behaviour.Behaviour):
    def __init__(self, name="VentosaOff"):
        super().__init__(name)
        self.process = None

    def initialise(self):
        self.process = run_ros2_command('smartfactory_ur_utils', 'ventosa_off')
        
    def update(self):
        if self.process.poll() is None:
            return py_trees.common.Status.RUNNING
        elif self.process.returncode == 0:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()

def create_root(ur10_controller, node):
    root = py_trees.composites.Sequence(name="Raiz", memory=True)

    check_aruco = CheckArucoPose(node)
    send_angles = MoveUR10ToPose(ur10_controller, node)
    check_vacuum_sensor = CheckUltrasonicGripper(node)
    ventosa_on = VentosaOn()
    #send_conveyor = SendConveyor()
    ventosa_off = VentosaOff()

    seq = py_trees.composites.Sequence(name="Sequencia", memory=True)
    seq.add_children([
        check_aruco, send_angles, check_vacuum_sensor, 
        ventosa_on, ventosa_off
    ])

    #     seq.add_children([
    #     check_aruco, send_angles, check_vacuum_sensor, 
    #     ventosa_on, send_conveyor, ventosa_off
    # ])
    root.add_child(seq)
    return root



def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseSubscriber()
    ur10_controller = UR10Controller()
    root = create_root(ur10_controller,node)

    behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        behavior_tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        sys.exit(1)
    except KeyboardInterrupt:
        behavior_tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    behavior_tree.tick_tock(period_ms=1000.0)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(ur10_controller) 
    executor.add_node(behavior_tree.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        behavior_tree.shutdown()
        node.destroy_node()
        ur10_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()




