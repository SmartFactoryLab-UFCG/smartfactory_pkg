#!/usr/bin/env python3
# This script creates a ROS 2 node that adjusts the robot's speed based on velocity and scaling factors.
# It synchronizes messages from the velocity and velocity scale topics, computes the new speed, and publishes
# the adjusted velocity.

import rclpy
from geometry_msgs.msg import Twist
from turtlebot2i_interfaces.msg import VelocityScale
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, ReliabilityPolicy

def speed_callback(vel_data, scale_data, velocity_publisher):
    """
    Callback function to adjust speed based on the received velocity and scaling factors.

    Parameters:
    - vel_data: The received Twist message containing the current velocity.
    - scale_data: The received VelocityScale message containing the scale factors.
    - velocity_publisher: Publisher to publish the adjusted velocity.
    """ 
    interWheelDistance = 0.137 # Distance between the wheels

    l_scale = scale_data.left_vel_scale # Left wheel scaling factor
    r_scale = scale_data.right_vel_scale # Right wheel scaling factor
    
    new_speed = Twist()  # Variable to hold the new speed

    if vel_data.linear.x < 0.0:
        # Ensure minimum speed limit for reverse motion
        new_speed.linear.x = max(vel_data.linear.x, -0.1)
        new_speed.angular.z = vel_data.angular.z

    elif r_scale < 0.01 and l_scale < 0.01:
        # Stop the robot if both scales are too low
        new_speed.linear.x = 0.0
        new_speed.angular.z = vel_data.angular.z

    else:
        # Calculate new speeds based on scaling factors
        vel_l = (vel_data.linear.x - vel_data.angular.z * interWheelDistance) * l_scale
        vel_r = (vel_data.linear.x + vel_data.angular.z * interWheelDistance) * r_scale
        new_speed.linear.x = (vel_r + vel_l) / 2.0 
        new_speed.angular.z = (vel_r - vel_l) / interWheelDistance
    # Publish the adjusted velocity
    velocity_publisher.publish(new_speed)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('speed_scaling_py')

     # Define QoS profiles
    qos_profile_reliable = QoSProfile(depth=10)
    qos_profile_reliable.reliability = ReliabilityPolicy.RELIABLE
    qos_profile_best_effort = QoSProfile(depth=10)
    qos_profile_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT

    # Create publisher for adjusted velocity
    velocity_publisher = node.create_publisher(Twist, '/safety/velocity', qos_profile_reliable)
    
    # Create subscribers for velocity and velocity scale
    vel_sub = Subscriber(node, Twist, '/commands/velocity')
    vel_scale_sub = Subscriber(node, VelocityScale, '/safety/vel_scale')

    # Synchronize the subscribers
    ts = ApproximateTimeSynchronizer([vel_sub, vel_scale_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(lambda vel_data, scale_data: speed_callback(vel_data, scale_data, velocity_publisher))

    node.get_logger().info("Speed Scaling Node is now running!")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
