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
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose


# Implementando o solver de cinemática inversa diretamente no script
class ikSolver():
    def __init__(self, a, d, alpha):
        # Inicializa os parâmetros DH do UR10
        self.a = a
        self.d = d
        self.alpha = alpha

    def create_Transformation_Matrix(self, position, orientation):
        T = np.eye(4)
        
        # Convertendo quaternion para matriz de rotação
        # A ordem do quaternion deve ser (x, y, z, w) para scipy
        rot = R.from_quat(orientation)  # Usando o quaternion aqui
        T[0:3, 0:3] = rot.as_matrix()   # Gerar a matriz de rotação a partir do quaternion
        
        T[0:3, 3] = position
        return T

    def DHLink(self, alpha, a, d, angle):
        T = np.array([[np.cos(angle), -np.sin(angle), 0, a],
                     [np.sin(angle) * np.cos(alpha), np.cos(angle) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
                     [np.sin(angle) * np.sin(alpha), np.cos(angle) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
                     [0, 0, 0, 1]])
        return T

    def nearestQ(self, q_list, last_q):
        weights = np.array([6, 5, 4, 3, 2, 1])
        best_q = np.zeros(6)
        bestConfDist = np.inf
        for q in q_list:
            confDist = np.sum(((q - last_q) * weights) ** 2)
            if confDist < bestConfDist:
                bestConfDist = confDist
                best_q = q
        return np.asarray(best_q)

    def solveIK(self, T06, *last_q):
        theta = np.zeros([8, 6])
        
        # Cálculo de theta1
        P05 = (T06 @ np.array([0, 0, -self.d[5], 1]))[0:3]
        phi1 = np.arctan2(P05[1], P05[0])
        phi2 = np.array([np.arccos(self.d[3] / np.linalg.norm(P05[0:2])), -np.arccos(self.d[3] / np.linalg.norm(P05[0:2]))])

        for i in range(4):
            theta[i, 0] = phi1 + phi2[0] + np.pi / 2
            theta[i + 4, 0] = phi1 + phi2[1] + np.pi / 2

        for i in range(8):
            if theta[i, 0] <= np.pi:
                theta[i, 0] += 2 * np.pi
            if theta[i, 0] > np.pi:
                theta[i, 0] -= 2 * np.pi

        # Cálculo de theta5
        P06 = T06[0:3, 3]
        for i in range(8):
            theta[i, 4] = np.arccos((P06[0] * np.sin(theta[i, 0]) - P06[1] * np.cos(theta[i, 0]) - self.d[3]) / self.d[5])
            if np.isin(i, [2, 3, 6, 7]):
                theta[i, 4] = -theta[i, 4]

        # Cálculo de theta6
        T60 = np.linalg.inv(T06)
        X60 = T60[0:3, 0]
        Y60 = T60[0:3, 1]

        for i in range(8):
            theta[i, 5] = np.arctan2((-X60[1] * np.sin(theta[i, 0]) + Y60[1] * np.cos(theta[i, 0])) / np.sin(theta[i, 4]),
                                     (X60[0] * np.sin(theta[i, 0]) - Y60[0] * np.cos(theta[i, 0])) / np.sin(theta[i, 4]))

        for i in range(8):
            T01 = self.DHLink(self.alpha[0], self.a[0], self.d[0], theta[i, 0])
            T45 = self.DHLink(self.alpha[4], self.a[4], self.d[4], theta[i, 4])
            T56 = self.DHLink(self.alpha[5], self.a[5], self.d[5], theta[i, 5])

            T14 = np.linalg.inv(T01) @ T06 @ np.linalg.inv(T45 @ T56)
            p13 = T14 @ np.array([[0], [-self.d[3]], [0], [1]]) - np.array([[0], [0], [0], [1]])

            # Cálculo de norma de p13 (similar ao p14xz no código original)
            p13norm2 = np.linalg.norm(p13)**2
            
            # Cálculo de cos_theta3 com verificação de intervalo
            arg = (p13norm2 - self.a[1]**2 - self.a[2]**2) / (2 * self.a[1] * self.a[2])
            if (arg > 1 or arg < -1):
                print("[WARNING] No solution for Theta3: argument out of range")
                if arg > 1:
                    arg = 1 - 1e-10
                else:
                    arg = -1 + 1e-10

            # Calcular theta3 (dual solution: cotovelo para cima e para baixo)
            theta3_positive = np.arccos(arg)
            theta3_negative = -theta3_positive

            # Atribuir soluções positivas e negativas para theta3
            theta[i, 2] = theta3_positive
            if i % 2 != 0:  # Alternar entre as soluções para cotovelo para cima/baixo
                theta[i, 2] = theta3_negative

            # Agora podemos calcular theta2
            p13norm = np.linalg.norm(p13)
            theta[i, 1] = -np.arctan2(p13[1], -p13[0]) + np.arcsin(self.a[2] * np.sin(theta[i, 2]) / p13norm)
        # Cálculo de theta3 e theta2
        # for i in range(8):
        #     T01 = self.DHLink(self.alpha[0], self.a[0], self.d[0], theta[i, 0])
        #     T45 = self.DHLink(self.alpha[4], self.a[4], self.d[4], theta[i, 4])
        #     T56 = self.DHLink(self.alpha[5], self.a[5], self.d[5], theta[i, 5])

        #     T14 = np.linalg.inv(T01) @ T06 @ np.linalg.inv(T45 @ T56)
        #     P14xz = np.array([T14[0, 3], T14[2, 3]])

        #     theta[i, 2] = np.arccos((np.linalg.norm(P14xz) ** 2 - self.a[1] ** 2 - self.a[2] ** 2) / (2 * self.a[1] * self.a[2]))
        #     # cos_theta3 = (np.linalg.norm(P14xz) ** 2 - self.a[1] ** 2 - self.a[2] ** 2) / (2 * self.a[1] * self.a[2])
        #     # cos_theta3 = np.clip(cos_theta3, -1, 1)
        #     # theta[i, 2] = np.arccos(cos_theta3)
        #     print(P14xz)
        #     if i % 2 != 0:
        #         theta[i, 2] = -theta[i, 2]

        #     theta[i, 1] = np.arctan2(-P14xz[1], -P14xz[0]) - np.arcsin(-self.a[2] * np.sin(theta[i, 2]) / np.linalg.norm(P14xz))

        # Cálculo de theta4
        for i in range(8):
            T01 = self.DHLink(self.alpha[0], self.a[0], self.d[0], theta[i, 0])
            T12 = self.DHLink(self.alpha[1], self.a[1], self.d[1], theta[i, 1])
            T23 = self.DHLink(self.alpha[2], self.a[2], self.d[2], theta[i, 2])
            T45 = self.DHLink(self.alpha[4], self.a[4], self.d[4], theta[i, 4])
            T56 = self.DHLink(self.alpha[5], self.a[5], self.d[5], theta[i, 5])

            T34 = np.linalg.inv(T01 @ T12 @ T23) @ T06 @ np.linalg.inv(T45 @ T56)

            theta[i, 3] = np.arctan2(T34[1, 0], T34[0, 0])

        if last_q:
            q = self.nearestQ(theta, last_q)
            return q, theta
        else:
            return theta

# Classe para controlar o UR10
class UR10KinematicsAction(Node):
    def __init__(self):
        super().__init__('ur10_kinematics_action')
        # Estados das juntas
        self.joint_names = []
        self.current_joint_positions = []
        self.joint_names_received = False
        self.aruco_poses = None

        # Parâmetros DH do UR10
        a = [0, -0.612, -0.5723, 0.163941, 0.1157, 0.0922]  # a (m)
        d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]        # d (m)
        alpha = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]  # alpha (rad)

        self.ik_solver = ikSolver(a, d, alpha)  # Instanciar o solver de IK

        # Action Client para enviar a ação FollowJointTrajectory
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        # Assinaturas de tópicos
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.aruco_sub = self.create_subscription(PoseArray, '/aruco/poses_world', self.aruco_pose_callback, 10)

    def aruco_pose_callback(self, msg):
        self.aruco_poses = msg.poses

    def joint_state_callback(self, msg):
        if not self.joint_names_received:
            self.joint_names = msg.name
            self.joint_names_received = True
        self.current_joint_positions = msg.position

    def send_joint_angles(self):
        if self.aruco_poses:
            pose = Pose()
            # Definir a posição
            pose.position.x = self.aruco_poses[0].position.x 
            pose.position.y = -self.aruco_poses[0].position.y
            pose.position.z = self.aruco_poses[0].position.z
            
            # Definir a orientação (quaternion)
            pose.orientation.x = self.aruco_poses[0].orientation.x
            pose.orientation.y = self.aruco_poses[0].orientation.y
            pose.orientation.z = self.aruco_poses[0].orientation.z
            pose.orientation.w = self.aruco_poses[0].orientation.w

            # Transformar 'pose' para uma matriz de transformação homogênea
            # position = [pose.position.x, pose.position.y, pose.position.z]
            # orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            position = [0,1,0]
            orientation =[0,0,0,1]
            T06 = self.ik_solver.create_Transformation_Matrix(position, orientation)

            q_d = self.current_joint_positions 
            # Calcular a cinemática inversa para essa pose transformada
            self.inverse_k = self.ik_solver.solveIK(T06, *q_d)

            # Exibir os resultados
            print("Ângulos das juntas calculados (cinemática inversa):")
            print(self.inverse_k[0])

            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

            points = []
            point1 = self.inverse_k[0]
            point1 = list(map(float, point1)) 

            point_1 = JointTrajectoryPoint()
            point_1.positions = point1
            point_1.time_from_start = Duration(seconds=10).to_msg()
            points.append(point_1)

            goal_msg.trajectory.points = points
            send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            rclpy.spin_until_future_complete(self, send_goal_future)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Progresso: {feedback.desired.positions}')

def main(args=None):
    rclpy.init(args=args)
    ur10_kinematics_action = UR10KinematicsAction()

    try:
        while rclpy.ok():
            rclpy.spin_once(ur10_kinematics_action, timeout_sec=0.1)
            if ur10_kinematics_action.aruco_poses is not None:
                ur10_kinematics_action.send_joint_angles()
                ur10_kinematics_action.aruco_poses = None
    except KeyboardInterrupt:
        pass
    finally:
        ur10_kinematics_action.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
