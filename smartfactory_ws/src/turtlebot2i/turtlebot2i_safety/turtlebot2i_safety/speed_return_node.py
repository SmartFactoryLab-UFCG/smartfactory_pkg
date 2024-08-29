#!/usr/bin/env python3
# This script creates a ROS 2 node that adjusts the robot's speed based on its current safety zone. 
# It subscribes to the robot's velocity and safety zone topics, processes the information, and publishes
# adjusted velocity and operation mode. The robot's speed is reduced or halted depending on the safety zone.

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Global variables
node = None
vel_teleoperada = Twist()
zone = 'Safe Zone'
control_mode = 'Teleop'

def teleop_callback(msg):
    """
    Callback function for the teleoperated velocity topic. Updates the velocity and adjusts speed
    based on the current zone.

    Parameters:
    - msg: The received Twist message containing the teleoperated velocity.
    """
    global vel_teleoperada, control_mode
    if control_mode == 'Teleop':
        vel_teleoperada = msg
        adjust_speed()

def zone_callback(msg):
    """
    Callback function for the safety zone topic. Updates the zone and adjusts speed accordingly.

    Parameters:
    - msg: The received String message containing the safety zone information.
    """
    global zone
    zone = msg.data
    adjust_speed()

# Adjusts the robot's speed based on the current safety zone. Publishes the adjusted velocity and operation mode.
def adjust_speed():
    global vel_teleoperada, zone, node, control_mode

    velocity_new = Twist() # Variable to hold the new velocity
    operation_mode = String() #  Variable to hold the operation mode

    if zone == 'Critical Zone':
        # Stop the robot in critical zone
        velocity_new.linear.x = 0.0
        velocity_new.angular.z = 0.0
        operation_mode.data = 'Robot Control'
        control_mode = 'Robot Control'

    elif zone == 'Warning Zone':
        # Reduce velocity in warning zone
        velocity_new.linear.x = vel_teleoperada.linear.x / 2
        velocity_new.angular.z = vel_teleoperada.angular.z / 2
        operation_mode.data = 'Reduce Velocity'
        control_mode = 'Teleop'

    else:
        # Maintain teleoperated velocity in safe zone
        velocity_new = vel_teleoperada
        operation_mode.data = 'Teleop'
        control_mode = 'Teleop'

    velocity_publisher.publish(velocity_new)
    mode_publisher.publish(operation_mode)

def main(args=None):
    global node, velocity_publisher, mode_publisher

    rclpy.init(args=args)
    node = rclpy.create_node('speed_return_node')

    # Define QoS profile
    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    # Create subscriptions
    node.create_subscription(Twist, '/commands/velocity', teleop_callback, qos_profile)
    node.create_subscription(String, '/safety/zone', zone_callback, qos_profile)

    # Create publishers
    velocity_publisher = node.create_publisher(Twist, '/safety/velocity', 10)
    mode_publisher = node.create_publisher(String, '/safety/operation_mode', 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
