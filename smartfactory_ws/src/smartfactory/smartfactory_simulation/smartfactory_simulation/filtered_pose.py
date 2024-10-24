#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import numpy as np

class ArucoFilteredPosePublisher(Node):
    def __init__(self, window_size=10):
        super().__init__('aruco_filtered_pose_publisher')

        # Assinando o tópico para receber as poses do ArUco
        self.subscription = self.create_subscription(
            PoseArray, '/aruco/poses_world', self.pose_callback, 10)
        
        # Publicador para o novo tópico de poses filtradas
        self.publisher_ = self.create_publisher(PoseArray, '/aruco/filtered_poses', 10)

        # Filtros de média móvel (para suavizar as leituras)
        self.window_size = window_size
        self.pose_windows = {}  # Dicionário para armazenar janelas de dados por ID de ArUco

    def pose_callback(self, msg):
        # Cria um PoseArray para armazenar as poses filtradas
        filtered_pose_array = PoseArray()
        filtered_pose_array.header = msg.header  # Mantém o mesmo header

        # Iterar sobre as poses recebidas
        for i, pose in enumerate(msg.poses):
            # A chave será o índice da pose no array (pode ser modificado conforme sua aplicação)
            if i not in self.pose_windows:
                self.pose_windows[i] = {'x': [], 'y': [], 'z': []}

            # Adicionar as novas leituras às janelas de filtro
            self.update_window(self.pose_windows[i]['x'], pose.position.x)
            self.update_window(self.pose_windows[i]['y'], pose.position.y)
            self.update_window(self.pose_windows[i]['z'], pose.position.z)

            # Calcula a média móvel para suavizar os dados
            x_smooth = np.mean(self.pose_windows[i]['x'])
            y_smooth = np.mean(self.pose_windows[i]['y'])
            z_smooth = np.mean(self.pose_windows[i]['z'])

            # Criar uma nova Pose suavizada
            filtered_pose = Pose()
            filtered_pose.position.x = x_smooth
            filtered_pose.position.y = y_smooth
            filtered_pose.position.z = z_smooth
            filtered_pose.orientation = pose.orientation  # Mantém a orientação original

            # Adicionar a pose suavizada no PoseArray
            filtered_pose_array.poses.append(filtered_pose)

        # Publica o PoseArray filtrado no novo tópico
        self.publisher_.publish(filtered_pose_array)

        # Para depuração: exibir no terminal o número de poses suavizadas
        self.get_logger().info(f"Publicando {len(filtered_pose_array.poses)} poses suavizadas.")

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
