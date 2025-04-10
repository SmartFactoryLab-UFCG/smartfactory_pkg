#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
import numpy as np

class UR10MoveToAngles(Node):
    def __init__(self):
        super().__init__('back_to_aruco')
        
        # Cliente de ação para seguir a trajetória das juntas
        self._action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        
        # Define os ângulos de destino em graus
        # self.target_angles1 = [-235, -75, 37, -58, -90, 0]
        self.target_angles2 = [-235, -75, 37, -58, -90, 0]
        # Converte para radianos
        # self.target_angles1 = [np.radians(angle) for angle in self.target_angles1]
        self.target_angles2 = [np.radians(angle) for angle in self.target_angles2]

        # Inicia o envio da trajetória
        self.send_joint_angles()

    def send_joint_angles(self):
        # Cria a mensagem de objetivo para o controlador de trajetória
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 
            'shoulder_lift_joint', 
            'elbow_joint', 
            'wrist_1_joint', 
            'wrist_2_joint', 
            'wrist_3_joint'
        ]

        points = []
        # Cria os pontos de trajetória com os ângulos desejados e tempos de execução
        # point1 = JointTrajectoryPoint()
        # point1.positions = self.target_angles1
        # point1.time_from_start = Duration(seconds=5).to_msg()
        # points.append(point1)
        
        point2 = JointTrajectoryPoint()
        point2.positions = self.target_angles2
        point2.time_from_start = Duration(seconds=3).to_msg()  # Define o tempo para alcançar o segundo ponto
        points.append(point2)

        # Define todos os pontos na trajetória
        goal_msg.trajectory.points = points

        # Envia o objetivo para o controlador
        self._action_client.wait_for_server()
        self.get_logger().info("Enviando ângulos de junta para o UR10...")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Objetivo rejeitado pelo controlador.")
            return

        self.get_logger().info("Objetivo aceito pelo controlador.")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progresso da trajetória: {feedback.desired.positions}')

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("Trajetória concluída com sucesso!")
        else:
            self.get_logger().error(f"Erro ao atingir a trajetória: código de erro {result.error_code}")

def main(args=None):
    rclpy.init(args=args)
    ur10_move_to_angles = UR10MoveToAngles()

    try:
        rclpy.spin(ur10_move_to_angles)
    except KeyboardInterrupt:
        pass
    finally:
        ur10_move_to_angles.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
