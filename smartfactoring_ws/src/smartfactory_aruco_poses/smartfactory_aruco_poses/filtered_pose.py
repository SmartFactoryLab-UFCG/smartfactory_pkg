#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import numpy as np

class ArucoFilteredPosePublisher(Node):
    def __init__(self, window_size=10):
        super().__init__('aruco_filtered_pose_publisher')

        self.declare_parameter('input_pose_topic', '/kinect/aruco/target_pose_world')
        self.declare_parameter('output_pose_array_topic', '/kinect/aruco/filtered_poses')
        self.declare_parameter('output_pose_topic', '/kinect/aruco/filtered_pose')

        self.input_pose_topic = (
            self.get_parameter('input_pose_topic').get_parameter_value().string_value
        )
        self.output_pose_array_topic = (
            self.get_parameter('output_pose_array_topic').get_parameter_value().string_value
        )
        self.output_pose_topic = (
            self.get_parameter('output_pose_topic').get_parameter_value().string_value
        )

        # Assinando o tópico para receber a pose do ArUco target já em world frame
        self.subscription = self.create_subscription(
            PoseStamped, self.input_pose_topic, self.pose_callback, 10)
        
        # Publicadores para as poses filtradas
        self.publisher_pose_array = self.create_publisher(PoseArray, self.output_pose_array_topic, 10)
        self.publisher_pose = self.create_publisher(PoseStamped, self.output_pose_topic, 10)

        # Filtros de média móvel (para suavizar as leituras)
        self.window_size = window_size
        self.pose_windows = {'x': [], 'y': [], 'z': []}

    def pose_callback(self, msg):
        pose = msg.pose

        # Adicionar as novas leituras às janelas de filtro
        self.update_window(self.pose_windows['x'], pose.position.x)
        self.update_window(self.pose_windows['y'], pose.position.y)
        self.update_window(self.pose_windows['z'], pose.position.z)

        # Calcula a média móvel para suavizar os dados
        x_smooth = float(np.mean(self.pose_windows['x']))
        y_smooth = float(np.mean(self.pose_windows['y']))
        z_smooth = float(np.mean(self.pose_windows['z']))

        # Criar uma nova Pose suavizada
        filtered_pose = Pose()
        filtered_pose.position.x = x_smooth
        filtered_pose.position.y = y_smooth
        filtered_pose.position.z = z_smooth
        filtered_pose.orientation = pose.orientation  # Mantém a orientação original

        filtered_pose_msg = PoseStamped()
        filtered_pose_msg.header = msg.header
        filtered_pose_msg.pose = filtered_pose

        # Cria um PoseArray para manter compatibilidade com consumidores existentes
        filtered_pose_array = PoseArray()
        filtered_pose_array.header = msg.header
        filtered_pose_array.poses.append(filtered_pose)

        self.publisher_pose.publish(filtered_pose_msg)
        self.publisher_pose_array.publish(filtered_pose_array)

        # Para depuração: exibir no terminal o número de poses suavizadas
        # self.get_logger().info(f"Publicando {len(filtered_pose_array.poses)} poses suavizadas.")

    def update_window(self, window, new_value):
        """ Atualiza a janela do filtro de média móvel """
        window.append(new_value)
        if len(window) > self.window_size:
            window.pop(0)  # Remove o valor mais antigo se a janela estiver cheia


def main(args=None):
    rclpy.init(args=args)
    aruco_filtered_pose_publisher = ArucoFilteredPosePublisher(window_size=10)

    try:
        rclpy.spin(aruco_filtered_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_filtered_pose_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
