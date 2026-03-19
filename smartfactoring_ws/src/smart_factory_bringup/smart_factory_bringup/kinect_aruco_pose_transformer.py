import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix
import tf2_ros
from aruco_interfaces.msg import ArucoMarkers

class ArucoPoseWorld(Node):

    def __init__(self):
        super().__init__('kinect_aruco_pose_world')
        self.declare_parameter('target_marker_id', 15)
        self.declare_parameter('markers_topic', 'kinect/aruco/markers')
        self.declare_parameter('camera_frame', 'camera_rgb_frame_kinect')
        self.declare_parameter('world_frame', 'world')

        self.target_marker_id = (
            self.get_parameter('target_marker_id').get_parameter_value().integer_value
        )
        self.markers_topic = (
            self.get_parameter('markers_topic').get_parameter_value().string_value
        )
        self.camera_frame = (
            self.get_parameter('camera_frame').get_parameter_value().string_value
        )
        self.world_frame = (
            self.get_parameter('world_frame').get_parameter_value().string_value
        )

        # Iniciando um broadcaster de transformações e um listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscrição para o tópico com ids + poses do ArUco
        self.pose_subscriber = self.create_subscription(
            ArucoMarkers, self.markers_topic, self.pose_callback, 10)

        # Publicador da pose do ArUco em relação ao mundo (PoseArray com 1 pose, para compatibilidade)
        self.pose_world_publisher = self.create_publisher(PoseArray, 'kinect/aruco/poses_world', 10)
        # Publicador do target em PoseStamped (mais direto para consumo)
        self.target_pose_world_publisher = self.create_publisher(
            PoseStamped, 'kinect/aruco/target_pose_world', 10
        )

    def pose_callback(self, msg):
        pose_array_world = PoseArray()
        pose_array_world.header.stamp = self.get_clock().now().to_msg()
        pose_array_world.header.frame_id = self.world_frame

        if not msg.marker_ids:
            return

        try:
            target_index = list(msg.marker_ids).index(self.target_marker_id)
        except ValueError:
            return

        pose = msg.poses[target_index]

        try:
            # Obter a transformação da câmera para o mundo
            transform_camera_world = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, rclpy.time.Time()
            )

            # Transformação do ArUco para a câmera (T_aruco_c)
            t_aruco_c = TransformStamped()
            t_aruco_c.transform.translation.x = pose.position.z
            t_aruco_c.transform.translation.y = -pose.position.x
            t_aruco_c.transform.translation.z = -pose.position.y
            t_aruco_c.transform.rotation = pose.orientation

            # Converter as transformações para matrizes 4x4
            t_c_w_matrix = quaternion_matrix([
                transform_camera_world.transform.rotation.x,
                transform_camera_world.transform.rotation.y,
                transform_camera_world.transform.rotation.z,
                transform_camera_world.transform.rotation.w,
            ])
            t_c_w_matrix[0:3, 3] = [
                transform_camera_world.transform.translation.x,
                transform_camera_world.transform.translation.y,
                transform_camera_world.transform.translation.z,
            ]

            t_aruco_c_matrix = quaternion_matrix([
                t_aruco_c.transform.rotation.x,
                t_aruco_c.transform.rotation.y,
                t_aruco_c.transform.rotation.z,
                t_aruco_c.transform.rotation.w,
            ])
            t_aruco_c_matrix[0:3, 3] = [
                t_aruco_c.transform.translation.x,
                t_aruco_c.transform.translation.y,
                t_aruco_c.transform.translation.z,
            ]

            # Multiplicar as matrizes para obter T_aruco_w
            t_aruco_w_matrix = t_c_w_matrix @ t_aruco_c_matrix

            # Extrair a posição e orientação da matriz resultante
            t_aruco_w_translation = translation_from_matrix(t_aruco_w_matrix)
            t_aruco_w_rotation = quaternion_from_matrix(t_aruco_w_matrix)

            # Publicar a transformação do Aruco em relação ao mundo via TF
            t_world = TransformStamped()
            t_world.header.stamp = pose_array_world.header.stamp
            t_world.header.frame_id = self.world_frame
            t_world.child_frame_id = f"kinect_aruco_world_{self.target_marker_id}"
            t_world.transform.translation.x = float(t_aruco_w_translation[0])
            t_world.transform.translation.y = float(t_aruco_w_translation[1])
            t_world.transform.translation.z = float(t_aruco_w_translation[2])
            t_world.transform.rotation.x = float(t_aruco_w_rotation[0])
            t_world.transform.rotation.y = float(t_aruco_w_rotation[1])
            t_world.transform.rotation.z = float(t_aruco_w_rotation[2])
            t_world.transform.rotation.w = float(t_aruco_w_rotation[3])
            self.tf_broadcaster.sendTransform(t_world)

            # Criar mensagem de pose em relação ao mundo
            pose_world = Pose()
            pose_world.position.x = float(t_aruco_w_translation[0])
            pose_world.position.y = float(t_aruco_w_translation[1])
            pose_world.position.z = float(t_aruco_w_translation[2])
            pose_world.orientation.x = float(t_aruco_w_rotation[0])
            pose_world.orientation.y = float(t_aruco_w_rotation[1])
            pose_world.orientation.z = float(t_aruco_w_rotation[2])
            pose_world.orientation.w = float(t_aruco_w_rotation[3])

            pose_array_world.poses.append(pose_world)

            target_pose_world = PoseStamped()
            target_pose_world.header = pose_array_world.header
            target_pose_world.pose = pose_world
            self.target_pose_world_publisher.publish(target_pose_world)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().error('Transformação da câmera para o mundo não encontrada!')
            return

        # Publicar pose do target em relação ao mundo (PoseArray com 1 pose)
        self.pose_world_publisher.publish(pose_array_world)
        # self.get_logger().info('Publicado as poses do Aruco target em relação ao mundo.')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
