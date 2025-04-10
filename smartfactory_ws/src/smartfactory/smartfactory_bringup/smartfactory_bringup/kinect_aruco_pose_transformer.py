import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix
import tf2_ros


class ArucoPoseWorld(Node):

    def __init__(self):
        super().__init__('kinect_aruco_pose_world')
        # Iniciando um broadcaster de transformações e um listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscrição para o tópico de poses do Aruco
        self.pose_subscriber = self.create_subscription(
            PoseArray, 'kinect/aruco/poses', self.pose_callback, 10)

        # Publicador da pose do Aruco em relação ao mundo
        self.pose_world_publisher = self.create_publisher(PoseArray, 'kinect/aruco/poses_world', 10)

    def pose_callback(self, msg):
        pose_array_world = PoseArray()
        pose_array_world.header.stamp = self.get_clock().now().to_msg()
        pose_array_world.header.frame_id = 'world'

        for i, pose in enumerate(msg.poses):
            try:
                # Obter a transformação da câmera para o mundo
                transform_camera_world = self.tf_buffer.lookup_transform(
                    'world', 'camera_rgb_frame_kinect', rclpy.time.Time())

                # Transformação do ArUco para a câmera (T_aruco_c)
                t_aruco_c = TransformStamped()
                t_aruco_c.transform.translation.x = pose.position.z 
                t_aruco_c.transform.translation.y = -pose.position.x
                t_aruco_c.transform.translation.z = -pose.position.y
                t_aruco_c.transform.rotation = pose.orientation

                # Converter as transformações para matrizes 4x4
                t_c_w_matrix = quaternion_matrix([transform_camera_world.transform.rotation.x,
                                                  transform_camera_world.transform.rotation.y,
                                                  transform_camera_world.transform.rotation.z,
                                                  transform_camera_world.transform.rotation.w])
                t_c_w_matrix[0:3, 3] = [transform_camera_world.transform.translation.x,
                                        transform_camera_world.transform.translation.y,
                                        transform_camera_world.transform.translation.z]

                t_aruco_c_matrix = quaternion_matrix([t_aruco_c.transform.rotation.x,
                                                      t_aruco_c.transform.rotation.y,
                                                      t_aruco_c.transform.rotation.z,
                                                      t_aruco_c.transform.rotation.w])
                t_aruco_c_matrix[0:3, 3] = [t_aruco_c.transform.translation.x,
                                            t_aruco_c.transform.translation.y,
                                            t_aruco_c.transform.translation.z]

                # Multiplicar as matrizes para obter T_aruco_w
                t_aruco_w_matrix = t_c_w_matrix @ t_aruco_c_matrix

                # Extrair a posição e orientação da matriz resultante
                t_aruco_w_translation = translation_from_matrix(t_aruco_w_matrix)
                t_aruco_w_rotation = quaternion_from_matrix(t_aruco_w_matrix)

                # Publicar a transformação do Aruco em relação ao mundo via TF
                t_world = TransformStamped()
                t_world.header.stamp = self.get_clock().now().to_msg()
                t_world.header.frame_id = 'world'
                t_world.child_frame_id = f"kinect_aruco_world_0_{i}" 
                t_world.transform.translation.x = t_aruco_w_translation[0]
                t_world.transform.translation.y = t_aruco_w_translation[1]
                t_world.transform.translation.z = t_aruco_w_translation[2]
                t_world.transform.rotation.x = t_aruco_w_rotation[0]
                t_world.transform.rotation.y = t_aruco_w_rotation[1]
                t_world.transform.rotation.z = t_aruco_w_rotation[2]
                t_world.transform.rotation.w = t_aruco_w_rotation[3]

                # Enviar a transformação TF
                self.tf_broadcaster.sendTransform(t_world)

                # Criar mensagem de pose em relação ao mundo
                pose_world = Pose()
                pose_world.position.x = t_aruco_w_translation[0]
                pose_world.position.y = t_aruco_w_translation[1]
                pose_world.position.z = t_aruco_w_translation[2]
                pose_world.orientation.x = t_aruco_w_rotation[0]
                pose_world.orientation.y = t_aruco_w_rotation[1]
                pose_world.orientation.z = t_aruco_w_rotation[2]
                pose_world.orientation.w = t_aruco_w_rotation[3]

                # Adicionar à lista de poses
                pose_array_world.poses.append(pose_world)

                # self.get_logger().info(f"Publicado frame TF para Aruco {i} em relação ao mundo")

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().error('Transformação da câmera para o mundo não encontrada!')

        # Publicar todas as poses do Aruco em relação ao mundo
        self.pose_world_publisher.publish(pose_array_world)
        # self.get_logger().info('Publicado as poses dos Arucos em relação ao mundo.')


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
