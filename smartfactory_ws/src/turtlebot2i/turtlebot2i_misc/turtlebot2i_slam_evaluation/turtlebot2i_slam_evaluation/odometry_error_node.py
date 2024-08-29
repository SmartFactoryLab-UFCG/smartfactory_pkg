# This script defines a ROS 2 node that calculates the Euclidean distance error between the robot's odometry
# and ground truth positions. The node subscribes to an odometry topic, retrieves ground truth positions using
# TF, and publishes the calculated error to a topic.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from turtlebot2i_interfaces.msg import EvalError
import numpy as np

class OdometryErrorCalculator(Node):
    def __init__(self):
        super().__init__('odometry_error_calculator')

        # Declare parameters
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('odometry_error', '/odom_euclidian_error')
        self.declare_parameter('ground_truth_frame_parent', '/world')
        self.declare_parameter('ground_truth_frame_child', '/kinect')
        self.declare_parameter('3DoF', False)

        odom_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.gt_frame_parent = self.get_parameter('ground_truth_frame_parent').get_parameter_value().string_value
        self.gt_frame_child = self.get_parameter('ground_truth_frame_child').get_parameter_value().string_value
        error_topic = self.get_parameter('odometry_error').get_parameter_value().string_value
        self.param_3DoF = self.get_parameter('3DoF').get_parameter_value().bool_value

        # Define the QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # Create a subscription to the odometry topic
        self.subscription_odom = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile)
        
        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=qos_profile)
        
        # Create publisher for error
        self.error_publisher = self.create_publisher(EvalError, error_topic, 10)

        self.odom_data = None
        self.gt_data = None

    # Callback function for odometry subscription
    def odom_callback(self, msg):
        self.odom_data = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.calculate_error()

    # Function to calculate the Euclidean distance error
    def calculate_error(self):
        try:
            # Lookup the transform between ground truth frames
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.gt_frame_parent, 
                self.gt_frame_child,
                rclpy.time.Time())
            
            self.gt_data = np.array([transform.transform.translation.x,
                                     transform.transform.translation.y,
                                     transform.transform.translation.z])

            if self.odom_data is not None and self.gt_data is not None:
                # Calculate the error based on 3DoF parameter
                if self.param_3DoF:
                    error = np.linalg.norm(self.odom_data[0:2] - self.gt_data[0:2])
                else:
                    error = np.linalg.norm(self.odom_data - self.gt_data)
                self.get_logger().info(f'Euclidean distance error: {error:.4f}')
                    
                # Publish the error
                error_msg = EvalError()
                error_msg.header.stamp = self.get_clock().now().to_msg()
                error_msg.data = error
                self.error_publisher.publish(error_msg)
            elif self.gt_data is None:
                self.get_logger().info(f'Ground Truth data is none')
            elif self.odom_data is None:
                self.get_logger().info(f'Odom data is none')
                    
        except (LookupException, ConnectivityException, ExtrapolationException) as fallback_e:
            # Handle exceptions during transform lookup
            self.get_logger().error(f'Fallback transform also failed: {fallback_e}')
            return

def main(args=None):
    rclpy.init(args=args)
    node = OdometryErrorCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
