#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
import csv

class ArucoPosePlotter(Node):
    def __init__(self):
        super().__init__('aruco_pose_plotter')

        # Assinando o tópico para receber as poses do ArUco (PoseArray)
        self.subscription = self.create_subscription(
            PoseArray, '/kinect/aruco/poses', self.pose_callback, 10)
        
        # Inicializar buffers de dados para múltiplas poses
        self.x_data = []
        self.y_data = []
        self.z_data = []

        # Abrir arquivo CSV para escrita
        self.csv_file = open('aruco_pose_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['X', 'Y', 'Z'])  # Cabeçalhos do CSV

        # Inicializar a plotagem
        self.init_plot()

    def pose_callback(self, msg):
        if msg.poses:
            # Pegando a primeira pose do PoseArray
            pose = msg.poses[0]

            # Adicionar os dados nas listas
            self.x_data.append(pose.position.x)
            self.y_data.append(pose.position.y)
            self.z_data.append(pose.position.z)

            # Salvar no CSV
            self.csv_writer.writerow([pose.position.x, pose.position.y, pose.position.z])
            
            # Atualizar o gráfico
            self.update_plot()

    def init_plot(self):
        # Configurar o gráfico
        self.fig, (self.ax_x, self.ax_y, self.ax_z) = plt.subplots(3, 1, figsize=(8, 6))

        self.ax_x.set_title('Posição X ao longo do tempo')
        self.ax_x.set_xlabel('Tempo (quadros)')
        self.ax_x.set_ylabel('X (m)')

        self.ax_y.set_title('Posição Y ao longo do tempo')
        self.ax_y.set_xlabel('Tempo (quadros)')
        self.ax_y.set_ylabel('Y (m)')

        self.ax_z.set_title('Posição Z ao longo do tempo')
        self.ax_z.set_xlabel('Tempo (quadros)')
        self.ax_z.set_ylabel('Z (m)')

        # Inicializar as linhas
        self.line_x, = self.ax_x.plot([], [], lw=2, label='X')
        self.line_y, = self.ax_y.plot([], [], lw=2, label='Y')
        self.line_z, = self.ax_z.plot([], [], lw=2, label='Z')

        # Mostrar o gráfico
        plt.tight_layout()
        plt.ion()
        plt.show()

    def update_plot(self):
        # Atualizar os dados no gráfico
        self.line_x.set_data(range(len(self.x_data)), self.x_data)
        self.line_y.set_data(range(len(self.y_data)), self.y_data)
        self.line_z.set_data(range(len(self.z_data)), self.z_data)

        # Ajustar limites dos eixos
        self.ax_x.set_xlim(0, len(self.x_data))
        self.ax_x.set_ylim(min(self.x_data, default=0), max(self.x_data , default=1))

        self.ax_y.set_xlim(0, len(self.y_data))
        self.ax_y.set_ylim(min(self.y_data, default=0), max(self.y_data , default=1))

        self.ax_z.set_xlim(0, len(self.z_data))
        self.ax_z.set_ylim(min(self.z_data, default=0), max(self.z_data, default=1))

        # Atualizar o gráfico
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def __del__(self):
        # Fechar o arquivo CSV ao finalizar
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePlotter()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.1)  # Permitir que o matplotlib atualize a janela de plotagem
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






