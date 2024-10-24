#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
from ur_ikfast import ur_kinematics


# Classe para controlar o UR10
class UR10KinematicsAction(Node):
    def __init__(self):
        super().__init__('ur10_kinematics_action')
        # Estados das juntas
        # Action Client para enviar a ação FollowJointTrajectory
        self.ur10 = ur_kinematics.URKinematics('ur10')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_joint_angles(self):
        
        pose_quat = [0.0, -1.0, 0.25, 0.9961306, 0.0523491, 0.0473595, 0.0523491]
        home = [0, -1.5708, 0, -1.5708, 0, 0]

        # pose_aruco = [-0.005, -1.0, (0.505250619615849-0.75), 0.4657, 0.52, 0.435, -0.57]

        pose_aruco = [-0.005, -1.0, (0.505250619615849-0.75), 0.4657+1, 0.435, -0.57, 0.52]

        inv = self.ur10.inverse(pose_aruco, True, q_guess=home)
        # Exibir os resultados
        print("Ângulos das juntas calculados (cinemática inversa):")
        a = 4
        print(inv)

        # joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians
        # print("joint angles", joint_angles)

        # pose_quat = self.ur10.forward(joint_angles)
        # pose_matrix = self.ur10.forward(joint_angles, 'matrix')

        # print("forward() quaternion \n", pose_quat)
        # print("forward() matrix \n", pose_matrix)

        # print("inverse() one from quat", self.ur10.inverse(pose_quat, True))
        # print("inverse() one from matrix", self.ur10.inverse(pose_matrix, False, q_guess=joint_angles))

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        points = []
        point1 = inv[a]
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
            ur10_kinematics_action.send_joint_angles()
    except KeyboardInterrupt:
        pass
    finally:
        ur10_kinematics_action.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
