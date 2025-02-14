#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ur_msgs.srv import SetIO

class VentosaOn(Node):
    def __init__(self):
        super().__init__('ventosa_on')
        
        # Configura o cliente de serviço para o controle de IO
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        
        # Aguarda o serviço de IO ficar disponível
        while not self.io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando pelo serviço de IO...')

        # Aciona a ventosa
        self.activate_vacuum()

    def activate_vacuum(self):
        # Define o estado da ventosa para "ligado" (1.0)
        request_pin_1 = SetIO.Request()
        request_pin_1.fun = 1  # Configura saída digital
        request_pin_1.pin = 1
        request_pin_1.state = 1.0  # Aciona o pino 1

        request_pin_2 = SetIO.Request()
        request_pin_2.fun = 1  # Configura saída digital
        request_pin_2.pin = 2
        request_pin_2.state = 1.0  # Aciona o pino 2

        # Envia as chamadas de serviço para os pinos
        self.io_client.call_async(request_pin_1).add_done_callback(self.io_callback)
        self.io_client.call_async(request_pin_2).add_done_callback(self.io_callback)

    def io_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Ventosa ativada com sucesso!")
            else:
                self.get_logger().error("Falha ao ativar a ventosa.")
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar serviço IO: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    ventosa_node = VentosaOn()

    try:
        rclpy.spin(ventosa_node)
    except KeyboardInterrupt:
        pass
    finally:
        ventosa_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
