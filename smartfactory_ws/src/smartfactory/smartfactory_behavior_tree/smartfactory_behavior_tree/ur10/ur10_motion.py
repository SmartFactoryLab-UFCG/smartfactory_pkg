import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import py_trees
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.duration import Duration
import numpy as np



class UR10Motion(Node):
    """
    Nó responsável por enviar comandos de movimento para o UR10 via Action.
    """

    def __init__(self):
        super().__init__("ur10_motion_node")
        self.joint_angles = None
        self.sending_goal = False
        self.last_error_code = None

        # Subscrição aos ângulos calculados
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/ur10/calculated_joint_angles',
            self.joint_angles_callback,
            10
        )

        # Action client para FollowJointTrajectory
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

    def joint_angles_callback(self, msg):
        self.joint_angles = msg.data
        self.get_logger().info(f"🔄 Ângulos recebidos: {self.joint_angles}")

    def send_joint_angles(self):
        if self.joint_angles is None:
            self.get_logger().warn("⚠️ Nenhum ângulo calculado disponível!")
            return

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Action server não disponível!")
            self.last_error_code = 1
            self.sending_goal = False
            return

        # Cria trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        point = JointTrajectoryPoint()
        point.positions = self.joint_angles
        point.time_from_start.sec = 2  # Pode ajustar conforme o perfil do robô

        trajectory.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.sending_goal = True
        self.last_error_code = None

        # Enviar goal
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejeitado pelo servidor!")
            self.last_error_code = 2
            self.sending_goal = False
            return

        self.get_logger().info("✅ Goal aceito, aguardando resultado...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.sending_goal = False
        self.last_error_code = result.error_code  # 0 = sucesso

        if result.error_code == 0:
            self.get_logger().info("✅ Movimento concluído com sucesso!")
        else:
            self.get_logger().error(f"❌ Movimento falhou! Código: {result.error_code}")

class MoveUR10(py_trees.behaviour.Behaviour):
    """
    Comportamento para acionar o movimento do UR10 via action.
    """

    def __init__(self, ur10_motion, name="MoveUR10"):
        super().__init__(name)
        self.ur10_motion = ur10_motion
        self.goal_sent = False

    def initialise(self):
        if not self.goal_sent and self.ur10_motion.joint_angles is not None:
            self.logger.info("📤 Enviando movimento ao UR10 via Action...")
            self.ur10_motion.send_joint_angles()
            self.goal_sent = True
        else:
            self.logger.warning("⚠️ Nenhum ângulo disponível!")

    def update(self):
        if self.ur10_motion.sending_goal:
            return py_trees.common.Status.RUNNING

        if self.ur10_motion.last_error_code != 0:
            self.logger.error(f"❌ Erro no movimento! Código: {self.ur10_motion.last_error_code}")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS



class SendConveyorMotion(Node):
    """
    Move o UR10 para uma posição fixa associada ao envio para a esteira.
    """
    def __init__(self):
        super().__init__('send_conveyor_motion')
        self.action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.sending_goal = False
        self.last_error_code = None

    def send_conveyor_trajectory(self):
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        target_angles1 = [-235, -75, 37, -58, -90, 0]
        target_angles2 = [-250, -57, 33, -68, -90, 32]

        target_angles1 = [np.radians(a) for a in target_angles1]
        target_angles2 = [np.radians(a) for a in target_angles2]

        point1 = JointTrajectoryPoint()
        point1.positions = target_angles1
        point1.time_from_start = Duration(seconds=5).to_msg()

        point2 = JointTrajectoryPoint()
        point2.positions = target_angles2
        point2.time_from_start = Duration(seconds=10).to_msg()

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        trajectory.points = [point1, point2]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor da action não disponível!")
            self.last_error_code = 1
            self.sending_goal = False
            return

        self.sending_goal = True
        self.last_error_code = None

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Objetivo rejeitado!")
            self.last_error_code = 2
            self.sending_goal = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.last_error_code = result.error_code
        self.sending_goal = False
        if result.error_code == 0:
            self.get_logger().info("Movimento para esteira concluído com sucesso.")
        else:
            self.get_logger().error(f"Erro ao mover para esteira. Código: {result.error_code}")




class SendConveyorAction(py_trees.behaviour.Behaviour):
    def __init__(self, motion_node: SendConveyorMotion, name="SendConveyorAction"):
        super().__init__(name)
        self.motion_node = motion_node
        self.goal_sent = False

    def initialise(self):
        if not self.goal_sent:
            self.logger.info("📤 Enviando movimento para posição da esteira...")
            self.motion_node.send_conveyor_trajectory()
            self.goal_sent = True

    def update(self):
        if self.motion_node.sending_goal:
            return py_trees.common.Status.RUNNING

        if self.motion_node.last_error_code == 0:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
