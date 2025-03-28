import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.duration import Duration

class UR10Controller(Node):
    def __init__(self):
        super().__init__('ur10_controller')

        # Action client para enviar trajetórias
        self._action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

        # Subscrição aos ângulos calculados
        self.calculated_angles_sub = self.create_subscription(
            Float64MultiArray,
            '/ur10/calculated_joint_angles',
            self.calculated_angles_callback,
            10
        )
        
        # Subscrição ao tópico /joint_states para verificar a posição atual das juntas
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = []
        self.joint_angles = []  # Inicializa a variável para armazenar os ângulos desejados
        self.sending_goal = False  # Variável para bloquear o envio de novas trajetórias enquanto uma está em andamento
        self.last_error_code = None

    def calculated_angles_callback(self, msg):
        # Recebe os ângulos calculados
        self.joint_angles = msg.data  # Armazena os ângulos desejados
        # self.get_logger().info(f"Calculated Joint Angles received: {self.joint_angles}")

        # Envia os ângulos para o controlador somente se não estiver enviando outra tarefa
        if not self.sending_goal:
            self.send_joint_angles(self.joint_angles)
        else:
            pass
            # self.get_logger().info("Trajetória em andamento, aguardando finalização para enviar nova.")

    def send_joint_angles(self, joint_angles):
        # Define que uma nova trajetória está sendo enviada
        self.sending_goal = True
        
        # Criação do Goal para a ação FollowJointTrajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # Primeiro ponto da trajetória com os ângulos calculados
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = Duration(seconds=5).to_msg()
        goal_msg.trajectory.points = [point]

        # Envia o Goal
        # self.get_logger().info('Enviando ângulos para o controlador...')
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejeitado')
            self.sending_goal = False  # Libera para enviar uma nova trajetória se o goal foi rejeitado
            return

        self.get_logger().info('Goal aceito, aguardando resultado...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Recebe o feedback durante a execução da trajetória
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Progresso: {feedback.desired.positions}")

    def get_result_callback(self, future):
        result = future.result().result
        self.last_error_code = result.error_code
        self.get_logger().info(f"Trajetória concluída com sucesso! Código de erro: {result.error_code}")
        
        # Trajetória concluída, desbloqueia o envio de novas trajetórias
        self.sending_goal = False

    def joint_state_callback(self, msg):
        # Callback para atualizar a posição atual das juntas
        self.current_joint_positions = msg.position
        # self.get_logger().info(f"Posição atual das juntas: {self.current_joint_positions}")

        # Comparar as posições atuais com os ângulos desejados para verificar se chegou à posição
        
        #if self.is_goal_reached(self.current_joint_positions):
        #    self.get_logger().info("Robô atingiu a posição desejada.")

    def is_goal_reached(self, current_positions):
        # Tolerância de comparação para verificar se o robô atingiu os ângulos desejados
        tolerance = 0.01  # Ajuste conforme necessário
        goal_reached = all(abs(current - target) < tolerance for current, target in zip(current_positions, self.joint_angles))
        return goal_reached

def main(args=None):
    rclpy.init(args=args)
    ur10_controller = UR10Controller()

    try:
        rclpy.spin(ur10_controller)
    except KeyboardInterrupt:
        pass
    finally:
        ur10_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
