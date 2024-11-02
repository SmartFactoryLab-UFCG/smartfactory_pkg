#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ur_msgs.srv import SetIO

class SuctionControlOff(Node):
    def __init__(self):
        super().__init__('suction_control_off')

        # Cria um cliente para o serviço de IO
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')

    def disable_suction(self):
        # Espera até que o serviço esteja disponível
        if not self.io_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Serviço de IO não disponível.")
            return

        # Configura a chamada do serviço para desligar a ventosa
        # Pino 1
        request_pin_1 = SetIO.Request()
        request_pin_1.fun = 1  # Configura como saída digital
        request_pin_1.pin = 1  # Seleciona o pino 1
        request_pin_1.state = 0.0  # Define o estado como desligado

        # Pino 2
        request_pin_2 = SetIO.Request()
        request_pin_2.fun = 1  # Configura como saída digital
        request_pin_2.pin = 2  # Seleciona o pino 2
        request_pin_2.state = 0.0  # Define o estado como desligado

        # Chama o serviço para o pino 1 e pino 2
        self.io_client.call_async(request_pin_1).add_done_callback(self.io_callback)
        self.io_client.call_async(request_pin_2).add_done_callback(self.io_callback)

    def io_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Ventosa desligada com sucesso!")
            else:
                self.get_logger().error("Falha ao desligar a ventosa.")
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar o serviço IO: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    suction_control_off = SuctionControlOff()

    try:
        # Desliga as ventosas
        suction_control_off.disable_suction()
        rclpy.spin_once(suction_control_off, timeout_sec=2.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        suction_control_off.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
