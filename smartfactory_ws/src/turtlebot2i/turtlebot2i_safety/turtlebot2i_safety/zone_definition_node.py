#!/usr/bin/env python3
# This script defines a ROS 2 node that determines the robot's current safety zone based on LIDAR data and publishes this zone. 
# The node calculates the distance to the closest obstacle and uses predefined radius values for different zones. 
# It also calculates the robot's speed from the odometry data.

import rclpy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Global variables
zone_publisher = None  # Publisher for the safety zone information
speed = 0.0  # Variable to hold the robot's speed

# Initialize global variables.
def init_var():
    global speed
    speed = 0.0

def speed_callback(data):
    """
    Callback function to calculate and update the robot's speed from odometry data.

    Parameters:
    - data: The received Odometry message containing the robot's velocity.
    """
    global speed
    # Compute the speed as the magnitude of the linear velocity vector
    speed = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2)

def lidar_callback(msg):
    """
    Callback function to determine and publish the current safety zone based on LIDAR data.

    Parameters:
    - msg: The received LaserScan message containing distance measurements from the LIDAR.
    """
    global zone_publisher
    # Find the minimum distance from the LIDAR ranges
    min_distance = min(msg.ranges)

    # Retrieve zone radius parameters
    critical_zone_radius = node.get_parameter('critical_zone_radius').get_parameter_value().double_value
    warning_zone_radius = node.get_parameter('warning_zone_radius').get_parameter_value().double_value
    clear_zone_radius = node.get_parameter('clear_zone_radius').get_parameter_value().double_value

    # Determine the safety zone based on the minimum distance
    if min_distance < critical_zone_radius:
        zone = 'Critical Zone'
    elif min_distance < warning_zone_radius:
        zone = 'Warning Zone'
    elif min_distance < clear_zone_radius:
        zone = 'Clear Zone'
    else:
        zone = 'Safe Zone'

    # Publish the determined zone
    zone_publisher.publish(String(data=zone))

def main(args=None):
    global zone_publisher, node

    rclpy.init(args=args)
    node = rclpy.create_node('zone_definition_node')
    init_var()

    # Declare parameters with default values
    node.declare_parameter('critical_zone_radius', 0.35)
    node.declare_parameter('warning_zone_radius', 0.45)
    node.declare_parameter('clear_zone_radius', 0.6)
	
    # Define QoS profile
    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
    
    # Create subscriptions
    node.create_subscription(LaserScan, '/lidar_scan', lidar_callback, qos_profile)
    node.create_subscription(Odometry, '/odom', speed_callback, qos_profile)

    # Create publisher
    zone_publisher = node.create_publisher(String, '/safety/zone', qos_profile)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
