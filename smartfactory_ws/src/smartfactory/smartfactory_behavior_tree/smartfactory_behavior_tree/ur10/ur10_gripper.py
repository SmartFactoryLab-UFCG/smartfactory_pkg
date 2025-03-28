import rclpy
import py_trees
from rclpy.node import Node
from std_msgs.msg import Bool
from ur_msgs.srv import SetIO


class UR10Gripper(Node):
    """
    Classe para controle da ventosa do UR10.
    Fornece métodos para ativar e desativar o vácuo usando o serviço SetIO.
    """

    def __init__(self):
        super().__init__("ur10_gripper_node")

        # Cliente do serviço de controle de IO
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Aguardando serviço de IO para controle da ventosa...")


    def set_io(self, pin: int, state: float):
        """
        Envia comando ao serviço de IO para controlar pinos da ventosa.
        Args:
            pin (int): Número do pino digital (1 ou 2).
            state (float): 1.0 para ativar, 0.0 para desativar.
        Returns:
            Future: Objeto future da chamada do serviço.
        """
        request = SetIO.Request()
        request.fun = 1  # digital output
        request.pin = pin
        request.state = state
        return self.io_client.call_async(request)


class VentosaOn(py_trees.behaviour.Behaviour):
    """
    Comportamento para ativar a ventosa com verificação de sucesso.
    """

    def __init__(self, gripper: UR10Gripper, name="VentosaOn"):
        super().__init__(name)
        self.gripper = gripper
        self.future1 = None
        self.future2 = None

    def initialise(self):
        self.logger.info("🔄 Enviando comando para ativar a ventosa...")
        self.future1 = self.gripper.set_io(1, 1.0)
        self.future2 = self.gripper.set_io(2, 1.0)

    def update(self):
        if not self.future1.done() or not self.future2.done():
            return py_trees.common.Status.RUNNING

        try:
            result1 = self.future1.result()
            result2 = self.future2.result()
            if result1.success and result2.success:
                self.logger.info("✅ Ventosa ativada com sucesso!")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error("❌ Falha ao ativar a ventosa.")
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.error(f"⚠️ Erro ao acionar a ventosa: {e}")
            return py_trees.common.Status.FAILURE


class VentosaOff(py_trees.behaviour.Behaviour):
    """
    Comportamento para desativar a ventosa com verificação de sucesso.
    """

    def __init__(self, gripper: UR10Gripper, name="VentosaOff"):
        super().__init__(name)
        self.gripper = gripper
        self.future1 = None
        self.future2 = None

    def initialise(self):
        self.logger.info("🔄 Enviando comando para desativar a ventosa...")
        self.future1 = self.gripper.set_io(1, 0.0)
        self.future2 = self.gripper.set_io(2, 0.0)

    def update(self):
        if not self.future1.done() or not self.future2.done():
            return py_trees.common.Status.RUNNING

        try:
            result1 = self.future1.result()
            result2 = self.future2.result()
            if result1.success and result2.success:
                self.logger.info("✅ Ventosa desativada com sucesso!")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error("❌ Falha ao desativar a ventosa.")
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.error(f"⚠️ Erro ao desativar a ventosa: {e}")
            return py_trees.common.Status.FAILURE
