#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
from ur_msgs.srv import SetIO 
from geometry_msgs.msg import PoseArray
from control_msgs.action import FollowJointTrajectory
import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
import Transformations as tf
import time
from scipy.spatial.transform import Rotation as R

class UR10KinematicsAction(Node):
    def __init__(self):
        super().__init__('ur10_kinematics_action')

        # Parâmetros do modelo UR10 (Tabela DH)
        self._standard_DH = np.mat([
            [0, -0.612, -0.5723, 0.163941, 0.1157, 0.0922],  # a (m)
            [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0],           # alpha (rad)
            [0.1273, 0, 0, 0.163941, 0.1157, 0.0922],        # d (m)
            [0, 0, 0, 0, 0, 0]                               # theta (rad)
        ])

        delta_DH = np.zeros((5,6))
        self.delta_standard_DH = delta_DH

        self._effective_a = self._standard_DH[0,:] + self.delta_standard_DH[0,:]
        self._effective_alpha = self._standard_DH[1,:] + self.delta_standard_DH[1,:]
        self._effective_d = self._standard_DH[2,:] + self.delta_standard_DH[2,:]
        self._effective_q = np.array(self._standard_DH[3,:] + self.delta_standard_DH[3,:])
        
        # Os dados efetivos equivalem aos dados esperados do UR10 mais os dados de calibração do robô específico.
        Rot_x_1 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,0]), -np.sin(self._effective_alpha[0,0]), 0], [0, np.sin(self._effective_alpha[0,0]),  np.cos(self._effective_alpha[0,0]), 0], [ 0, 0, 0, 1]])
        Rot_x_2 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,1]), -np.sin(self._effective_alpha[0,1]), 0], [0, np.sin(self._effective_alpha[0,1]),  np.cos(self._effective_alpha[0,1]), 0], [ 0, 0, 0, 1]])
        Rot_x_3 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,2]), -np.sin(self._effective_alpha[0,2]), 0], [0, np.sin(self._effective_alpha[0,2]),  np.cos(self._effective_alpha[0,2]), 0], [ 0, 0, 0, 1]])
        Rot_x_4 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,3]), -np.sin(self._effective_alpha[0,3]), 0], [0, np.sin(self._effective_alpha[0,3]),  np.cos(self._effective_alpha[0,3]), 0], [ 0, 0, 0, 1]])
        Rot_x_5 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,4]), -np.sin(self._effective_alpha[0,4]), 0], [0, np.sin(self._effective_alpha[0,4]),  np.cos(self._effective_alpha[0,4]), 0], [ 0, 0, 0, 1]])
        Rot_x_6 = np.mat([[1, 0, 0, 0], [0, np.cos(self._effective_alpha[0,5]), -np.sin(self._effective_alpha[0,5]), 0], [0, np.sin(self._effective_alpha[0,5]),  np.cos(self._effective_alpha[0,5]), 0], [ 0, 0, 0, 1]])

        Trans_d_1 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,0]], [0, 0, 0, 1]])
        Trans_d_2 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,1]], [0, 0, 0, 1]])
        Trans_d_3 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,2]], [0, 0, 0, 1]])
        Trans_d_4 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,3]], [0, 0, 0, 1]])
        Trans_d_5 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,4]], [0, 0, 0, 1]])
        Trans_d_6 = np.mat([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self._effective_d[0,5]], [0, 0, 0, 1]])

        Trans_a_1 = np.mat([[1, 0, 0, self._effective_a[0,0]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Trans_a_2 = np.mat([[1, 0, 0, self._effective_a[0,1]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Trans_a_3 = np.mat([[1, 0, 0, self._effective_a[0,2]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Trans_a_4 = np.mat([[1, 0, 0, self._effective_a[0,3]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Trans_a_5 = np.mat([[1, 0, 0, self._effective_a[0,4]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Trans_a_6 = np.mat([[1, 0, 0, self._effective_a[0,5]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        self._A_0_1 = Trans_d_1 * Trans_a_1 * Rot_x_1
        self._A_0_2 = Trans_d_2 * Trans_a_2 * Rot_x_2
        self._A_0_3 = Trans_d_3 * Trans_a_3 * Rot_x_3
        self._A_0_4 = Trans_d_4 * Trans_a_4 * Rot_x_4
        self._A_0_5 = Trans_d_5 * Trans_a_5 * Rot_x_5
        self._A_0_6 = Trans_d_6 * Trans_a_6 * Rot_x_6

        # Armazenar os nomes das juntas e os estados atuais
        self.joint_names = []
        self.current_joint_positions = []

        # Flag para indicar que os nomes das juntas foram recebidos
        self.joint_names_received = False
        self.aruco_poses = None

        # Action Client para enviar a ação FollowJointTrajectory
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        # Criar cliente para o serviço de IO
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        # Assinando o tópico /joint_states para ouvir os estados das juntas
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)      
        self.aruco_sub = self.create_subscription(PoseArray, '/aruco/poses_world', self.aruco_pose_callback, 10)

    def aruco_pose_callback(self, msg):
        # Callback que recebe as poses do ArUco e armazena
        self.aruco_poses = msg.poses
        # self.get_logger().info(f"Recebido {len(self.aruco_poses)} poses de ArUco.")
        # self.send_joint_angles()
        
    def joint_state_callback(self, msg):
        if not self.joint_names_received:
            self.joint_names = msg.name
            self.joint_names_received = True
            # self.get_logger().info(f"Nomes das juntas recebidos: {self.joint_names}")

        # Atualizar as posições atuais das juntas
        self.current_joint_positions = msg.position
    
    def send_joint_angles(self):
        # Criação do Goal para a ação FollowJointTrajectory
        # print(self.aruco_poses)
        p_Aruco = np.array([-self.aruco_poses[0].position.x, -self.aruco_poses[0].position.y, self.aruco_poses[0].position.z-0.3])
        # p_Aruco = np.round(p_Aruco, 4)
        print("Posição do Aruco", p_Aruco)

        o_Aruco = [self.aruco_poses[0].orientation.x, self.aruco_poses[0].orientation.y, self.aruco_poses[0].orientation.z, self.aruco_poses[0].orientation.w]
        o_Aruco = R.from_quat(o_Aruco)
        o_Aruco = o_Aruco.as_rotvec()
        # o_Aruco = np.round(o_Aruco, 4)
        print("Orientação do Aruco", o_Aruco)

        # p1 = np.concatenate((p_Aruco, o_Aruco))
        position = [0.004, 0.996, -0.245]
        # orientation = [0, 0.7071068, 0.7071068, 0]
        # orientation = [0, 0, 0, 1]
        orientation = [0.5, 0.5, 0.5, 0.5]
        orientation = R.from_quat(orientation)
        orientation = orientation.as_rotvec()
        pose = np.concatenate((position, orientation))
        print(pose)
        self.inverse_k = self.inverse_kinematics(pose)
        print("ik:", self.inverse_k)

        # p2 = p = np.concatenate(([0,0.5,0.8], o_Aruco))
        # self.inverse_k2 = self.inverse_kinematics(p2)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        points = []

        # Primeiro ponto (cinemática inversa)
        a = 0
        point1 = [self.inverse_k[0][a], self.inverse_k[1][a], self.inverse_k[2][a], self.inverse_k[3][a], self.inverse_k[4][a], self.inverse_k[5][a]]
        # point1 = [round(p, 4) for p in point1]
        print(point1)
        point_1 = JointTrajectoryPoint()
        point_1.positions = point1
        point_1.time_from_start = Duration(seconds=10).to_msg()
        points.append(point_1)

        # # Segundo ponto
        # point2 = [self.inverse_k2[0][a], self.inverse_k2[1][a], self.inverse_k2[2][a], -self.inverse_k2[3][a], self.inverse_k2[4][a], 0 * self.inverse_k2[5][a]]
        # point2 = [round(p, 4) for p in point2]
        # point_2 = JointTrajectoryPoint()
        # point_2.positions = point2
        # point_2.time_from_start = Duration(seconds=2).to_msg()
        # points.append(point_2)

        # # Terceiro ponto (convertido para radianos)
        # point3 = [-235, -71, 37, -58, -90, 0]
        # point3 = [np.radians(p) for p in point3]
        # point_3 = JointTrajectoryPoint()
        # point_3.positions = point3
        # point_3.time_from_start = Duration(seconds=4).to_msg()
        # points.append(point_3)

        # # Quarto ponto (convertido para radianos)
        # point4 = [-250, -57, 33, -68, -90, 32]
        # point4 = [np.radians(p) for p in point4]
        # point_4 = JointTrajectoryPoint()
        # point_4.positions = point4
        # point_4.time_from_start = Duration(seconds=2).to_msg()
        # points.append(point_4)
        
        for i, point in enumerate(points):
            goal_msg.trajectory.points = [point]
            # self.io(1.0)

            # Enviar o ponto
            self.get_logger().info(f'Enviando ponto {i+1}...')
            send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            rclpy.spin_until_future_complete(self, send_goal_future)
            
            # Checar se o goal foi aceito e processar o resultado
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().info(f'Goal {i+1} rejeitado :(')
                return

            self.get_logger().info('Goal aceito, aguardando resultado...')
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result()

            if result.result.error_code == 0:
                self.get_logger().info(f'Ponto {i+1} atingido com sucesso!')
            else:

                self.get_logger().error(f'Falha ao atingir o ponto {i+1}, erro: {result.error_code}')
                return

            # Pausar por 10 segundos antes de enviar o próximo ponto
            self.get_logger().info('Aguardando 5 segundos antes do próximo movimento...')
            time.sleep(2)

        self.get_logger().info('Desligando ventosa...')
        self.io(0.0)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejeitado :(')
            return
        self.get_logger().info('Goal aceito!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Receber o feedback da execução da trajetória
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Progresso: {feedback.desired.positions}')
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Trajetória concluída com sucesso! {result}')
        self.io(0.0)

    def _DH(self, a, alpha, d, theta):

        Td = np.asmatrix(np.eye(4))
        Td[2,3] = d
        Ta = np.asmatrix(np.eye(4))
        Ta[0,3] = a
        Rtheta = tf.Rot_z(theta)
        Rtheta = np.mat([[Rtheta[0,0], Rtheta[0,1], Rtheta[0,2], 0], [Rtheta[1,0], Rtheta[1,1], Rtheta[1,2], 0], [Rtheta[2,0], Rtheta[2,1], Rtheta[2,2], 0], [0,0,0,1]])
        Ralpha = tf.Rot_x(alpha)
        Ralpha = np.mat([[Ralpha[0,0], Ralpha[0,1], Ralpha[0,2], 0], [Ralpha[1,0], Ralpha[1,1], Ralpha[1,2], 0], [Ralpha[2,0], Ralpha[2,1], Ralpha[2,2], 0], [0,0,0,1]])

        G = Td * Rtheta * Ta * Ralpha

        return G
    
    def direct_kinematics(self, q, vector = False, rpy = False, apply_offset = False):

        _rot_z_1 = np.mat([[np.cos(q[0]), -np.sin(q[0]), 0, 0],[np.sin(q[0]), np.cos(q[0]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        _rot_z_2 = np.mat([[np.cos(q[1]), -np.sin(q[1]), 0, 0],[np.sin(q[1]), np.cos(q[1]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        _rot_z_3 = np.mat([[np.cos(q[2]), -np.sin(q[2]), 0, 0],[np.sin(q[2]), np.cos(q[2]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        _rot_z_4 = np.mat([[np.cos(q[3]), -np.sin(q[3]), 0, 0],[np.sin(q[3]), np.cos(q[3]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        _rot_z_5 = np.mat([[np.cos(q[4]), -np.sin(q[4]), 0, 0],[np.sin(q[4]), np.cos(q[4]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        _rot_z_6 = np.mat([[np.cos(q[5]), -np.sin(q[5]), 0, 0],[np.sin(q[5]), np.cos(q[5]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        # Utiliza as matrizes definidas no construtor e as de rotação das juntas atuais para retornar a matriz final.
        self._A_1 = _rot_z_1 * self._A_0_1
        self._A_2 = _rot_z_2 * self._A_0_2
        self._A_3 = _rot_z_3 * self._A_0_3
        self._A_4 = _rot_z_4 * self._A_0_4
        self._A_5 = _rot_z_5 * self._A_0_5
        self._A_6 = _rot_z_6 * self._A_0_6

        self._H = self._A_1 * self._A_2 * self._A_3 * self._A_4 * self._A_5 * self._A_6

        return self._H
        
    def inverse_kinematics(self, p):

        rvMatrix = tf.rotationVector2Matrix(p[3:6])

        gd = np.mat(([[rvMatrix[0,0], rvMatrix[0,1], rvMatrix[0,2], p[0]], [rvMatrix[1,0], rvMatrix[1,1], rvMatrix[1,2], p[1]], [rvMatrix[2,0], rvMatrix[2,1], rvMatrix[2,2], p[2]], [0, 0, 0, 1]]))

        theta = np.zeros((6, 8))

        d1 = self._standard_DH[2,0]
        d2 = self._standard_DH[2,1]
        d3 = self._standard_DH[2,2]
        d4 = self._standard_DH[2,3]
        d5 = self._standard_DH[2,4]
        d6 = self._standard_DH[2,5]

        a1 = self._standard_DH[0,0]
        a2 = self._standard_DH[0,1]
        a3 = self._standard_DH[0,2]
        a4 = self._standard_DH[0,3]
        a5 = self._standard_DH[0,4]
        a6 = self._standard_DH[0,5]

        alpha1 = self._standard_DH[1,0]
        alpha2 = self._standard_DH[1,1]
        alpha3 = self._standard_DH[1,2]
        alpha4 = self._standard_DH[1,3]
        alpha5 = self._standard_DH[1,4]
        alpha6 = self._standard_DH[1,5]

        # Calculating theta1
        p05 = gd * np.mat([[0], [0], [-d6], [1]])
        p05 = p05 - np.mat([[0], [0], [0], [1]])
        psi = np.arctan2(p05[1], p05[0])
        p05xy = np.sqrt(p05[1]*p05[1] + p05[0]*p05[0])
        if (d4 > p05xy):
            print ("[WARNING] No solution for Theta1: d4 > P05xy")
            print ("[WARNING] Creating aproximation highly inaccurate")
            d4 = p05xy - 1e-10
        try:
            phi = np.arccos(d4 / p05xy)
        except:
            print("[ERROR] Division by zero: " + str(p05xy))
            return None
        theta[0, 0:4] = np.radians(90) + psi + phi
        theta[0, 4:8] = np.radians(90) + psi - phi
        theta = np.real(theta)

        # Calculating theta5
        cols = np.array([0, 4])
        for i in range(0, cols.size):
            c = cols[i]
            try:
                T10 = inv(self._DH(a1, alpha1, d1, theta[0,c]))
            except:
                print("[ERROR] Could not find inverse: " + str(self._DH(a1, alpha1, d1, theta[0,c])))
                return None
            T16 = T10 * gd
            p16z = T16[2,3]
            try:
                if (((p16z-d4)/d6) > 1):
                    print ("[WARNING] No solution for Theta5: (p16z-d4)/d6) > 1")
                    print ("[WARNING] Creating aproximation highly inaccurate")
                    d6 = (p16z-d4) + 1e-10
                t5 = np.arccos((p16z-d4)/d6)
            except:
                print("[ERROR] Division by zero: " + str(d6))
                return None
            theta[4, c:c+2] = t5
            theta[4, c+2:c+4] = -t5
        theta = np.real(theta)

        # Calculating theta6
        cols = np.array([0, 2, 4, 6])
        for i in range(0, cols.size):
            c = cols[i]
            T01 = self._DH(a1, alpha1, d1, theta[0,c])
            try:
                T61 = inv(gd) * T01
            except:
                print("[ERROR] Could not find inverse: " + str(gd))
                return None
            T61zy = T61[1, 2]
            T61zx = T61[0, 2]
            t5 = theta[4, c]
            if (np.sin(t5) == 0):
                theta[5, c:c+2] = 0
            else:    
                theta[5, c:c+2] = np.arctan2(-T61zy/np.sin(t5), T61zx/np.sin(t5))
        theta = np.real(theta)

        # Calculating theta3
        cols = np.array([0, 2, 4, 6])
        for i in range (0, cols.size):
            c = cols[i]
            try:
                T10 = inv(self._DH(a1, alpha1, d1, theta[0,c]))
                T65 = inv(self._DH(a6, alpha6, d6, theta[5,c]))
                T54 = inv(self._DH(a5, alpha5, d5, theta[4,c]))
            except T10:
                print("[ERROR] Could not find inverse: Theta3, inverse 1, " + str(T10))
                return None
            except T65:
                print("[ERROR] Could not find inverse: Theta3, inverse 2, " + str(T65))
                return None
            except T54:
                print("[ERROR] Could not find inverse: Theta3, inverse 3, " + str(T54))
                return None
            T14 = T10 * gd * T65 * T54
            p13 = T14 * np.mat([[0], [-d4], [0], [1]])
            p13 = p13 - np.mat([[0], [0], [0], [1]])
            p13norm2 = norm(p13) * norm(p13)
            arg = (p13norm2-a2*a2-a3*a3)/(2*a2*a3)
            if (arg > 1 or arg < -1):
                print ("[WARNING] No solution for Theta3: arg < -1 or arg > 1")
                print ("[WARNING] Creating aproximation highly inaccurate")
                if (arg >1):
                    arg = 1 - 1e-10
                else:
                    arg = -1 + 1e-10
            t3p = np.arccos(arg)
            theta[2, c] = t3p
            theta[2, c+1] = -t3p
        theta = np.real(theta)

        # Calculating theta2 and theta4
        cols = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        for i in range (0, cols.size):
            c = cols[i]
            try:
                T10 = inv(self._DH(a1, alpha1, d1, theta[0,c]))
                T65 = inv(self._DH(a6, alpha6, d6, theta[5,c]))
                T54 = inv(self._DH(a5, alpha5, d5, theta[4,c]))
            except T10:
                print("[ERROR] Could not find inverse: Theta2 inverse 1, " + str(T10))
                return None
            except T65:
                print("[ERROR] Could not find inverse: Theta2, inverse 2, " + str(T65))
                return None
            except T54:
                print("[ERROR] Could not find inverse: Theta2, inverse 3, " + str(T54))
                return None
            T14 = T10 * gd * T65 * T54
            p13 = T14 * np.mat([[0], [-d4], [0], [1]]) - np.mat([[0], [0], [0], [1]])
            p13norm = norm(p13)
            theta[1, c] = -np.arctan2(p13[1], -p13[0])+np.arcsin(a3*np.sin(theta[2,c])/p13norm)
            try:
                T32 = inv(self._DH(a3, alpha3, d3, theta[2,c]))
                T21 = inv(self._DH(a2, alpha2, d2, theta[1,c]))
            except T10:
                print("[ERROR] Could not find inverse: Theta4 inverse 1, " + str(T32))
                return None
            except T65:
                print("[ERROR] Could not find inverse: Theta4, inverse 2, " + str(T21))
                return None
            T34 = T32 * T21 * T14
            theta[3, c] = np.arctan2(T34[1,0], T34[0,0])
        theta = np.real(theta)

        for i in range (0, 5):
            for j in range(0,7):
                if theta[i,j] > np.pi:
                    theta[i,j] -= 2*np.pi
                elif theta[i,j] < -np.pi:
                    theta[i,j] += 2*np.pi

        return theta
    
    def io(self, state):
        # Esperar até que o serviço esteja disponível
        if not self.io_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Serviço de IO não disponível.")
            return

        # Configurar a chamada do serviço para o pino 1
        request_pin_1 = SetIO.Request()
        request_pin_1.fun = 1  # Setar saída digital
        request_pin_1.pin = 1
        request_pin_1.state = state  # Acionar o pino 1

        # Configurar a chamada do serviço para o pino 2
        request_pin_2 = SetIO.Request()
        request_pin_2.fun = 1  # Setar saída digital
        request_pin_2.pin = 2
        request_pin_2.state = state  # Acionar o pino 2

        # Chamar o serviço para o pino 1
        self.io_client.call_async(request_pin_1).add_done_callback(self.io_pin_1_callback)

        # Chamar o serviço para o pino 2
        self.io_client.call_async(request_pin_2).add_done_callback(self.io_pin_2_callback)

    def io_pin_1_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Pino 1 acionado com sucesso!")
            else:
                self.get_logger().error("Falha ao acionar o pino 1.")
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar serviço IO para o pino 1: {str(e)}")

    def io_pin_2_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Pino 2 acionado com sucesso!")
            else:
                self.get_logger().error("Falha ao acionar o pino 2.")
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar serviço IO para o pino 2: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    ur10_kinematics_action = UR10KinematicsAction()

    try:
        while rclpy.ok():
            # Processa os callbacks sem bloquear a execução
            rclpy.spin_once(ur10_kinematics_action, timeout_sec=0.1)
            
            # Verifica se há poses do Aruco e então chama o send_joint_angles
            if ur10_kinematics_action.aruco_poses is not None:
                ur10_kinematics_action.send_joint_angles()
                ur10_kinematics_action.aruco_poses = None  # Resetar para evitar múltiplas chamadas
                # ur10_kinematics_action.destroy_node()

    except KeyboardInterrupt:
        pass
    finally:
        ur10_kinematics_action.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()