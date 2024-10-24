#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import Transformations as tf
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class UR10KinematicsAction(Node):
    def __init__(self):
        super().__init__('ur10_kinematics_fk')

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

        # Assinando o tópico /joint_states para ouvir os estados das juntas
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10) 
         # Publicar a pose final do end-effector
        self.pose_pub = self.create_publisher(PoseStamped, '/ur10_end_effector_pose', 10)     

    def joint_state_callback(self, msg):
        if not self.joint_names_received:
            self.joint_names = msg.name
            self.joint_names_received = True

        # Atualizar as posições atuais das juntas
        self.current_joint_positions = msg.position
        # Calcular a cinemática direta e publicar a pose
        if len(self.current_joint_positions) == 6:
            pose = self.compute_end_effector_pose(self.current_joint_positions)
            self.pose_pub.publish(pose)

    def compute_end_effector_pose(self, joint_positions):
        # Executa a cinemática direta utilizando os ângulos das juntas
        jp = np.round(joint_positions,2)
        # print(jp)
        transformation_matrix = self.direct_kinematics(jp)
        transformation_matrix = np.round(transformation_matrix,4)
        # print(transformation_matrix)
        # Extrair a translação e rotação da matriz homogênea
        translation = transformation_matrix[:3, 3]
        rotation_matrix = transformation_matrix[:3, :3]
        rotation = R.from_matrix(rotation_matrix)
        quat = rotation.as_quat()  # Convertendo para quaternion
        
        # Publicar a pose no formato ROS
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link_inertia'  # Trocar para o frame correto, se necessário
        pose_msg.pose.position.x = translation[0, 0]
        pose_msg.pose.position.y = translation[1, 0]
        pose_msg.pose.position.z = translation[2, 0]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        return pose_msg

    def _DH(self, a, alpha, d, theta):

        Td = np.asmatrix(np.eye(4))
        Td[2,3] = d
        Ta = np.asmatrix(np.eye(4))
        Ta[0,3] = a
        Rtheta = tf.Rot_z(theta)
        Rtheta = np.mat([[Rtheta[0,0], Rtheta[0,1], Rtheta[0,2], 0], [Rtheta[1,0], Rtheta[1,1], Rtheta[1,2], 0], [Rtheta[2,0], Rtheta[2,1], Rtheta[2,2], 0], [0,0,0,1]])
        Ralpha = tf.Rot_x(alpha)
        Ralpha = np.mat([[Ralpha[0,0], Ralpha[0,1], Ralpha[0,2], 0], [Ralpha[1,0], Ralpha[1,1], Ralpha[1,2], 0], [Ralpha[2,0], Ralpha[2,1], Ralpha[2,2], 0], [0,0,0,1]])

        G = Td @ Rtheta @ Ta @ Ralpha

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

        self._H = self._A_1 @ self._A_2 @ self._A_3 @ self._A_4 @ self._A_5 @ self._A_6

        return self._H

def main(args=None):
    rclpy.init(args=args)
    ur10_kinematics_action = UR10KinematicsAction()

    try:
        while rclpy.ok():
            rclpy.spin_once(ur10_kinematics_action, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ur10_kinematics_action.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()