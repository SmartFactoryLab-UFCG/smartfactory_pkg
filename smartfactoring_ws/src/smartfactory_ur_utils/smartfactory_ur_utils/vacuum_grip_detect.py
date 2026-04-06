import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ur_msgs.msg import ToolDataMsg 

class VacuumGripperNode(Node):
    def __init__(self):
        super().__init__('vacuum_gripper_node')

        self.subscription = self.create_subscription(
            ToolDataMsg,  
            '/io_and_status_controller/tool_data',
            self.io_callback,
            10
        )
        self.publisher = self.create_publisher(Bool, '/vacuum_gripper_status', 10)
        self.get_logger().info('Nó de detecção de objeto na ventosa inicializado')

    def io_callback(self, msg):
        
        analog_input2 = msg.analog_input2  

        #limiar de detecção do objeto
        objeto_detectado = analog_input2 < 0.007  

        status_msg = Bool()
        status_msg.data = objeto_detectado
        self.publisher.publish(status_msg)

        # Log para monitoramento
        #estado = "OBJETO DETECTADO" if objeto_detectado else "VENTOSA VAZIA"
        #self.get_logger().info(f"Estado: {estado}")

def main(args=None):
    rclpy.init(args=args)
    node = VacuumGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando nó de detecção de objeto na ventosa")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
