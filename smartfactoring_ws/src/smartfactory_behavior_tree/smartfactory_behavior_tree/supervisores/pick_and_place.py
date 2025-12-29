import rclpy
import py_trees
import py_trees_ros
import sys
import time
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from aruco_interfaces.msg import ArucoMarkers
from rclpy.node import Node

# Importando os módulos refatorados
from smartfactory_behavior_tree.ur10.ur10_motion import UR10Motion, MoveUR10, SendConveyorMotion, SendConveyorAction
from smartfactory_behavior_tree.ur10.ur10_sensors import UR10Sensors, CheckUltrasonicGripper
from smartfactory_behavior_tree.ur10.ur10_gripper import UR10Gripper, VentosaOn, VentosaOff
from smartfactory_behavior_tree.algoritms_perception.aruco_detector import ArucoPoseSubscriber, CheckArucoPose


class NoPieceFeedback(py_trees.behaviour.Behaviour):
    def __init__(self, name="SemPecaFeedback", log_interval_s=5.0):
        super().__init__(name)
        self.log_interval_s = log_interval_s
        self._last_log_time = 0.0

    def update(self):
        now = time.time()
        if now - self._last_log_time >= self.log_interval_s:
            self.logger.info("Nenhuma peça detectada (topo não visível). Insira uma peça na célula.")
            self._last_log_time = now
        return py_trees.common.Status.RUNNING


class MarkerIdSubscriber(Node):
    def __init__(self, *, node_name: str, topic: str, target_id: int):
        super().__init__(node_name)
        self.topic = topic
        self.target_id = int(target_id)
        self.last_seen_time: float | None = None
        self.last_pose: Pose | None = None

        self.create_subscription(ArucoMarkers, self.topic, self._cb, 10)

    def _cb(self, msg: ArucoMarkers):
        try:
            idx = list(msg.marker_ids).index(self.target_id)
        except ValueError:
            return
        self.last_seen_time = time.time()
        self.last_pose = msg.poses[idx]

    def seen_recently(self, max_age_s: float) -> bool:
        return self.last_seen_time is not None and (time.time() - self.last_seen_time) <= max_age_s


class WaitMarkerSeenStable(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        subscriber: MarkerIdSubscriber,
        *,
        name: str,
        max_age_s: float = 0.5,
        stable_ticks: int = 3,
        timeout_s: float = 2.0,
    ):
        super().__init__(name)
        self.subscriber = subscriber
        self.max_age_s = float(max_age_s)
        self.stable_ticks = int(stable_ticks)
        self.timeout_s = float(timeout_s)
        self._start_time: float | None = None
        self._stable_count = 0

    def initialise(self):
        self._start_time = time.time()
        self._stable_count = 0

    def update(self):
        if self.subscriber.seen_recently(self.max_age_s):
            self._stable_count += 1
        else:
            self._stable_count = 0

        if self._stable_count >= self.stable_ticks:
            return py_trees.common.Status.SUCCESS

        if self._start_time is not None and (time.time() - self._start_time) >= self.timeout_s:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class WaitMarkerGoneStable(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        subscriber: MarkerIdSubscriber,
        *,
        name: str,
        max_age_s: float = 0.5,
        stable_ticks: int = 3,
        timeout_s: float = 5.0,
    ):
        super().__init__(name)
        self.subscriber = subscriber
        self.max_age_s = float(max_age_s)
        self.stable_ticks = int(stable_ticks)
        self.timeout_s = float(timeout_s)
        self._start_time: float | None = None
        self._stable_count = 0

    def initialise(self):
        self._start_time = time.time()
        self._stable_count = 0

    def update(self):
        if not self.subscriber.seen_recently(self.max_age_s):
            self._stable_count += 1
        else:
            self._stable_count = 0

        if self._stable_count >= self.stable_ticks:
            return py_trees.common.Status.SUCCESS

        if self._start_time is not None and (time.time() - self._start_time) >= self.timeout_s:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class WaitVacuumReleasedStable(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        sensors: UR10Sensors,
        *,
        name: str = "WaitVacuumReleased",
        stable_ticks: int = 3,
        timeout_s: float = 2.0,
    ):
        super().__init__(name)
        self.sensors = sensors
        self.stable_ticks = int(stable_ticks)
        self.timeout_s = float(timeout_s)
        self._start_time: float | None = None
        self._stable_count = 0

    def initialise(self):
        self._start_time = time.time()
        self._stable_count = 0

    def update(self):
        if not self.sensors.is_object_detected():
            self._stable_count += 1
        else:
            self._stable_count = 0

        if self._stable_count >= self.stable_ticks:
            return py_trees.common.Status.SUCCESS

        if self._start_time is not None and (time.time() - self._start_time) >= self.timeout_s:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class MonitorCarry(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        *,
        name: str,
        ur10_sensors: UR10Sensors,
        kinect_floor_subscriber: MarkerIdSubscriber | None,
        astra_side_subscriber: MarkerIdSubscriber,
        send_conveyor_motion: SendConveyorMotion,
        astra_max_age_s: float = 0.5,
        motion_stale_timeout_s: float = 1.5,
        motion_translation_epsilon_m: float = 0.003,
        vacuum_false_fail_ticks: int = 2,
        kinect_max_age_s: float = 0.5,
        kinect_stable_ticks: int = 3,
    ):
        super().__init__(name)
        self.ur10_sensors = ur10_sensors
        self.kinect_floor_subscriber = kinect_floor_subscriber
        self.astra_side_subscriber = astra_side_subscriber
        self.send_conveyor_motion = send_conveyor_motion
        self.astra_max_age_s = float(astra_max_age_s)
        self.motion_stale_timeout_s = float(motion_stale_timeout_s)
        self.motion_translation_epsilon_m = float(motion_translation_epsilon_m)
        self.vacuum_false_fail_ticks = int(vacuum_false_fail_ticks)
        self.kinect_max_age_s = float(kinect_max_age_s)
        self.kinect_stable_ticks = int(kinect_stable_ticks)

        self._vacuum_false_count = 0
        self._last_pose: Pose | None = None
        self._last_motion_time: float | None = None
        self._movement_started = False
        self._kinect_stable_count = 0
        self._kinect_floor_confirmed = False

    def initialise(self):
        self._vacuum_false_count = 0
        self._last_pose = self.astra_side_subscriber.last_pose
        self._last_motion_time = time.time()
        self._movement_started = False
        self._kinect_stable_count = 0
        self._kinect_floor_confirmed = False

    def update(self):
        if not self._movement_started:
            if self.send_conveyor_motion.sending_goal:
                self._movement_started = True
            else:
                return py_trees.common.Status.RUNNING

        if not self.send_conveyor_motion.sending_goal:
            if self.kinect_floor_subscriber is not None and not self._kinect_floor_confirmed:
                self.logger.warning("Movimento finalizado sem confirmação do Kinect (id=0).")
            return py_trees.common.Status.SUCCESS

        if not self.ur10_sensors.is_object_detected():
            self._vacuum_false_count += 1
            if self._vacuum_false_count >= self.vacuum_false_fail_ticks:
                self.logger.error("Peça caiu: ultrassônico/vácuo perdeu detecção.")
                return py_trees.common.Status.FAILURE
        else:
            self._vacuum_false_count = 0

        if self.kinect_floor_subscriber is not None and not self._kinect_floor_confirmed:
            if self.kinect_floor_subscriber.seen_recently(self.kinect_max_age_s):
                self._kinect_stable_count += 1
            else:
                self._kinect_stable_count = 0

            if self._kinect_stable_count >= self.kinect_stable_ticks:
                self._kinect_floor_confirmed = True
                self.logger.info("Kinect confirmou id=0 (base vazia / peça removida).")

        if not self.astra_side_subscriber.seen_recently(self.astra_max_age_s):
            self.logger.error("Perdi a peça: Astra não detecta ArUco id=5 recentemente.")
            return py_trees.common.Status.FAILURE

        current_pose = self.astra_side_subscriber.last_pose
        now = time.time()
        if current_pose is not None and self._last_pose is not None:
            dx = current_pose.position.x - self._last_pose.position.x
            dy = current_pose.position.y - self._last_pose.position.y
            dz = current_pose.position.z - self._last_pose.position.z
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            if dist >= self.motion_translation_epsilon_m:
                self._last_motion_time = now
                self._last_pose = current_pose

        if self._last_motion_time is not None and (now - self._last_motion_time) > self.motion_stale_timeout_s:
            self.logger.error("Perdi a peça: pose do ArUco id=5 não muda durante o movimento.")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


def create_root(
    ur10_motion,
    ur10_sensors,
    ur10_gripper,
    aruco_top_detector,
    kinect_floor_subscriber,
    astra_side_subscriber,
    send_conveyor_motion,
):
    """
    Monta a árvore de comportamento do Pick and Place utilizando os módulos dos dispositivos e algoritmos.
    """
    root = py_trees.composites.Selector(name="PickAndPlace", memory=False)

    check_top = CheckArucoPose(aruco_top_detector)
    move_ur10 = MoveUR10(ur10_motion)
    check_grip = CheckUltrasonicGripper(ur10_sensors)
    ventosa_on = VentosaOn(ur10_gripper)
    send_conveyor = SendConveyorAction(send_conveyor_motion)
    ventosa_off = VentosaOff(ur10_gripper)

    confirm_picked = py_trees.composites.Sequence(name="ConfirmPicked", memory=True)
    confirm_picked.add_children([
        # Não usar Kinect id=0 aqui: ele só aparece depois que o robô começa a mover a peça.
        # A validação de "carry" acontece durante o movimento, em MonitorCarry.
        WaitMarkerSeenStable(
            astra_side_subscriber,
            name="WaitAstraId5",
            max_age_s=0.5,
            stable_ticks=3,
            timeout_s=3.0,
        ),
    ])

    carry_parallel = py_trees.composites.Parallel(
        name="CarryToConveyor",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    carry_parallel.add_children([
        send_conveyor,
        MonitorCarry(
            name="MonitorCarry",
            ur10_sensors=ur10_sensors,
            kinect_floor_subscriber=kinect_floor_subscriber,
            astra_side_subscriber=astra_side_subscriber,
            send_conveyor_motion=send_conveyor_motion,
        ),
    ])

    confirm_placed = py_trees.composites.Sequence(name="ConfirmPlaced", memory=True)
    confirm_placed.add_children([
        WaitVacuumReleasedStable(
            ur10_sensors,
            stable_ticks=3,
            timeout_s=2.0,
        ),
        # Após desligar a ventosa, esperamos ver o id=5 rapidamente (evidência de que a peça está na esteira).
        WaitMarkerSeenStable(
            astra_side_subscriber,
            name="WaitAstraId5AfterRelease",
            max_age_s=0.5,
            stable_ticks=2,
            timeout_s=1.0,
        ),
        # Em seguida, com a esteira ligada, o id=5 deve sumir do campo de visão (evidência de que seguiu na esteira).
        WaitMarkerGoneStable(
            astra_side_subscriber,
            name="WaitAstraId5Gone",
            max_age_s=0.5,
            stable_ticks=3,
            timeout_s=5.0,
        ),
    ])

    pick_sequence = py_trees.composites.Sequence(name="SequenciaPickAndPlace", memory=True)
    pick_sequence.add_children([
        check_top,
        move_ur10,
        ventosa_on,
        check_grip,
        confirm_picked,
        carry_parallel,
        ventosa_off,
        confirm_placed,
    ])

    no_piece_sequence = py_trees.composites.Sequence(name="SemPeca", memory=True)
    no_piece_sequence.add_children([NoPieceFeedback()])

    root.add_children([pick_sequence, no_piece_sequence])
    return root


def main(args=None):
    """
    Função principal para iniciar o Pick and Place utilizando ROS2 e Behavior Tree.
    """
    rclpy.init(args=args)

    # Instanciando os módulos (nós ROS2)
    aruco_top_detector = ArucoPoseSubscriber()
    ur10_motion = UR10Motion()
    ur10_sensors = UR10Sensors()
    ur10_gripper = UR10Gripper()
    send_conveyor_motion = SendConveyorMotion()
    kinect_floor_subscriber = MarkerIdSubscriber(
        node_name="kinect_floor_marker_subscriber",
        topic="/kinect/aruco/markers",
        target_id=0,
    )
    astra_side_subscriber = MarkerIdSubscriber(
        node_name="astra_side_marker_subscriber",
        topic="/astra/aruco/markers",
        target_id=5,
    )

    # Criando a árvore de comportamento
    root = create_root(
        ur10_motion,
        ur10_sensors,
        ur10_gripper,
        aruco_top_detector,
        kinect_floor_subscriber,
        astra_side_subscriber,
        send_conveyor_motion,
    )

    behavior_tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        behavior_tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError:
        sys.exit(1)

    # Executando a árvore com tick automático
    behavior_tree.tick_tock(period_ms=1000.0)

    # Executor ROS2 com todos os nós
    executor = MultiThreadedExecutor()
    executor.add_node(aruco_top_detector)
    executor.add_node(ur10_motion)
    executor.add_node(ur10_sensors)
    executor.add_node(ur10_gripper)
    executor.add_node(send_conveyor_motion)
    executor.add_node(kinect_floor_subscriber)
    executor.add_node(astra_side_subscriber)
    executor.add_node(behavior_tree.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        behavior_tree.shutdown()
        aruco_top_detector.destroy_node()
        ur10_motion.destroy_node()
        ur10_sensors.destroy_node()
        ur10_gripper.destroy_node()
        send_conveyor_motion.destroy_node()
        kinect_floor_subscriber.destroy_node()
        astra_side_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
