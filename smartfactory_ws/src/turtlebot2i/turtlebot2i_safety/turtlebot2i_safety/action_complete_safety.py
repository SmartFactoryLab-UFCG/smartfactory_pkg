#!/usr/bin/env python3
# This script implements the 'action_complete_safety' node for a TurtleBot2i robot.
# It uses a fuzzy logic system to assess and mitigate risks based on LIDAR data.
# The node adjusts the robot's speed to avoid obstacles and publishes various safety-related information.
import rclpy
import os
import pickle
import math
import std_msgs.msg
import numpy as np
import skfuzzy as fuzz
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from kobuki_ros_interfaces.msg import Led
from turtlebot2i_interfaces.msg import VelocityScale
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from skfuzzy import control as ctrl

def init_var(node):
    # Initialize various variables for the node
    node.n_sensors = 360
    node.lidar_min_deg = -180.0
    node.lidar_max_deg = 180.0
    node.idx_per_deg = (node.lidar_max_deg - node.lidar_min_deg) / node.n_sensors
    node.deg_per_idx = node.n_sensors / (node.lidar_max_deg - node.lidar_min_deg)

    node.IZW = 0.4
    step_meter = 0.02
    step_meter_per_second = 0.02
    step_risk = 0.05
    node.range_degree = np.arange(-180, 180+1, 1.0)
    node.idx_degree = np.arange(0, node.n_sensors+1, 1.0)
    node.range_meter = np.arange(0, 3.0+step_meter, step_meter)
    node.range_meter_per_second = np.arange(-0.5, 2.0+step_meter_per_second, step_meter_per_second)
    node.range_risk = np.arange(0, 5+step_risk, step_risk)

    node.vel_scale_message = VelocityScale()

    node.time_duration_list = []

    package_path = get_package_share_directory('turtlebot2i_safety')
    node.package_path = package_path

    node.r_warning = 0.0
    node.r_critical = 0.0
    node.obstacle_zone, node.prev_obstacle_zone = 0, 0
    node.clear_zone, node.warning_zone, node.critical_zone = 0, 1, 2

    node.get_logger().info("Variables initialized")

def init_fls_common_part(node):
    # Initialize the common parts of the fuzzy logic system
    node.object_distance = ctrl.Antecedent(node.range_meter, 'distance')
    node.object_direction = ctrl.Antecedent(node.range_degree, 'direction')

    node.object_distance['Near'] = fuzz.trapmf(node.range_meter, [0, 0, node.IZW, 2*node.IZW])
    node.object_distance['Medium'] = fuzz.trimf(node.range_meter, [node.IZW, 2*node.IZW, 4*node.IZW])
    node.object_distance['Far'] = fuzz.trapmf(node.range_meter, [2*node.IZW, 4*node.IZW, 3, 3])

    n_front = 0.0
    n_far_r = -135.0
    n_right = -90.0
    n_fr = -45.0
    n_fl = 45.0
    n_left = 90.0
    n_far_l = 135.0
    n_rear_r = 135.0
    n_rear_l = -135.0

    node.object_direction['Right'] = fuzz.trimf(node.range_degree, [n_far_r, n_right, n_fr])
    node.object_direction['FrontRight'] = fuzz.trimf(node.range_degree, [n_right, n_fr, n_front])
    node.object_direction['Front'] = fuzz.trimf(node.range_degree, [n_fr, n_front, n_fl])
    node.object_direction['FrontLeft'] = fuzz.trimf(node.range_degree, [n_front, n_fl, n_left])
    node.object_direction['Left'] = fuzz.trimf(node.range_degree, [n_fl, n_left, n_far_l])
    node.object_direction['RearRight'] = fuzz.trimf(node.range_degree, [n_right, n_rear_r, 180])
    node.object_direction['RearLeft'] = fuzz.trimf(node.range_degree, [-180, n_rear_l, n_left])

    node.get_logger().info("init_fls_common_part ok")

def init_risk_assessment(node):
    # Initialize the risk assessment fuzzy logic system
    range_type = np.arange(0, 2+1, 1)
    node.object_risk = ctrl.Consequent(node.range_risk, 'risk')

    node.object_risk['VeryLow'] = fuzz.trimf(node.range_risk, [0, 0, 1])
    node.object_risk['Low'] = fuzz.trimf(node.range_risk, [0, 1, 2])
    node.object_risk['Medium'] = fuzz.trimf(node.range_risk, [1, 2, 3])
    node.object_risk['High'] = fuzz.trimf(node.range_risk, [2, 3, 4])
    node.object_risk['VeryHigh'] = fuzz.trimf(node.range_risk, [3, 4, 4])

    fls_name = "/rules/ra_demo.data"
    fls_data_path = node.package_path + fls_name

    if os.path.exists(fls_data_path):
        node.get_logger().info("FLS demo exists!")
        with open(fls_data_path, 'rb') as f:
            node.ra_fls = pickle.load(f, encoding='latin1')
    else:
        node.get_logger().info("Init FLS demo")
        from assessment_rules_demo import rule_list_generator
        assessment_rule_list = rule_list_generator(node.object_distance, node.object_direction, node.object_risk)
        node.ra_fls = ctrl.ControlSystem(assessment_rule_list)
        with open(fls_data_path, 'wb') as f:
            pickle.dump(node.ra_fls, f)

    node.risk_assessment_instance = ctrl.ControlSystemSimulation(node.ra_fls)

    node.get_logger().info("init_risk_assessment ok")

def init_risk_mitigation(node):
    # Initialize the risk mitigation fuzzy logic system
    node.object_risk_input = ctrl.Antecedent(node.range_risk, 'risk_input')
    node.left_speed = ctrl.Consequent(node.range_meter_per_second, 'left')
    node.right_speed = ctrl.Consequent(node.range_meter_per_second, 'right')

    node.object_risk_input['VeryLow'] = fuzz.gaussmf(node.range_risk, 0, 0.3)
    node.object_risk_input['Low'] = fuzz.gaussmf(node.range_risk, 1, 0.3)
    node.object_risk_input['Medium'] = fuzz.gaussmf(node.range_risk, 2, 0.3)
    node.object_risk_input['High'] = fuzz.gaussmf(node.range_risk, 3, 0.3)
    node.object_risk_input['VeryHigh'] = fuzz.gaussmf(node.range_risk, 4, 0.3)

    node.left_speed['Stop'] = fuzz.gaussmf(node.range_meter_per_second, 0.0, 0.1)
    node.left_speed['Slow'] = fuzz.gaussmf(node.range_meter_per_second, 0.2, 0.2)
    node.left_speed['Medium'] = fuzz.gaussmf(node.range_meter_per_second, 0.8, 0.2)
    node.left_speed['Fast'] = fuzz.gaussmf(node.range_meter_per_second, 1.2, 0.2)

    # Added 'Reverse' to the speed definitions
    node.left_speed['Reverse'] = fuzz.gaussmf(node.range_meter_per_second, -0.5, 0.1)
    node.right_speed['Reverse'] = fuzz.gaussmf(node.range_meter_per_second, -0.5, 0.1)

    node.right_speed['Stop'] = fuzz.gaussmf(node.range_meter_per_second, 0.0, 0.1)
    node.right_speed['Slow'] = fuzz.gaussmf(node.range_meter_per_second, 0.2, 0.2)
    node.right_speed['Medium'] = fuzz.gaussmf(node.range_meter_per_second, 0.8, 0.2)
    node.right_speed['Fast'] = fuzz.gaussmf(node.range_meter_per_second, 1.2, 0.2)

    from turtlebot2i_safety.mitigation_rules import rule_list_generator
    mitigation_rule_list = rule_list_generator(node.object_distance, node.object_direction, node.object_risk_input, node.left_speed, node.right_speed)

    node.risk_mitigation_fls = ctrl.ControlSystem(mitigation_rule_list)
    node.risk_mitigation_instance = ctrl.ControlSystemSimulation(node.risk_mitigation_fls)

    node.get_logger().info("init_risk_mitigation ok")

def cal_risk(node, object_distance, object_direction):
    # Calculate the risk value based on object distance and direction
    node.risk_assessment_instance.input['distance'] = object_distance
    node.risk_assessment_instance.input['direction'] = object_direction
    node.risk_assessment_instance.compute()
    return node.risk_assessment_instance.output['risk']

def cal_safe_vel(node, object_distance, object_direction, object_risk):
    # Calculate safe velocities for the robot based on object distance, direction, and risk level.
    node.risk_mitigation_instance.input['distance'] = object_distance
    node.risk_mitigation_instance.input['direction'] = object_direction
    node.risk_mitigation_instance.input['risk_input'] = object_risk

    try:
        node.risk_mitigation_instance.compute()
        left_speed = node.risk_mitigation_instance.output['left']
        right_speed = node.risk_mitigation_instance.output['right']
        node.get_logger().info(f'Risk Mitigation - Distance: {object_distance}, Direction: {object_direction}, Risk: {object_risk}, Left Speed: {left_speed}, Right Speed: {right_speed}')
        return left_speed, right_speed
    
    except Exception as e:
        node.get_logger().error(f'Erro na mitigação de risco: {str(e)}')
        return 0.0, 0.0


def pub_safe_vel(node, left_vel_scale, right_vel_scale):
    # Publish the calculated safe velocities to the /safety/vel_scale topic.
    node.vel_scale_message.header = std_msgs.msg.Header()
    node.vel_scale_message.header.stamp = node.get_clock().now().to_msg()
    node.vel_scale_message.left_vel_scale = left_vel_scale
    node.vel_scale_message.right_vel_scale = right_vel_scale
    node.safe_vel_pub.publish(node.vel_scale_message)

def lidar_callback(data, node):
    # Callback function for processing LIDAR data. Calculates distance to the closest object, assesses risk, adjusts velocities, and publishes safety and zone data.
    min_reading = 0.3
    fov = 360 # Full field of view of the RPLIDAR A3

    # Adjust safety zone radii
    r_clear = 0.5  # Reduced from 0.6
    r_warning = 0.35  # Reduced from 0.45
    r_critical = 0.25  # Reduced from 0.35

    # Initialize risk counter and stuck variable
    if not hasattr(node, 'high_risk_count'):
        node.high_risk_count = 0
    if not hasattr(node, 'stuck'):
        node.stuck = 0.0

    # Initialize variables to avoid reference before assignment
    closest_dist = 0.0
    closest_dir = 0.0
    risk_val = 0.0
    led_msg = Led()

    try:
        ranges = list(np.where(np.array(data.ranges) < min_reading, np.inf, data.ranges))
        sensor_reads = ranges

        closest_dist = min(sensor_reads)
        closest_idx = np.argmin(sensor_reads)
        closest_dir = (closest_idx * data.angle_increment) * 180 / math.pi - fov / 2

        node.get_logger().info(f'closest_dist: {closest_dist} | closest_dir: {closest_dir}')

        risk_val = cal_risk(node, closest_dist, closest_dir)
        speed_l, speed_r = cal_safe_vel(node, closest_dist, closest_dir, risk_val)
        pub_safe_vel(node, speed_l, speed_r)

        led_msg = Led()

        if risk_val > 3.0:
            node.high_risk_count += 1
            led_msg.value = Led.RED
            risk_val = 3.0
            node.get_logger().warn('Robô em situação de risco elevado e não consegue sair!')
        else:
            node.high_risk_count = 0
            if risk_val > 2.0 and risk_val < 3:
                risk_val = 2.0
                led_msg.value = Led.ORANGE
            elif risk_val > 1.0 and risk_val < 2:
                risk_val = 1.0
                led_msg.value = Led.GREEN
            else:
                risk_val = 0.0
                led_msg.value = Led.BLACK

        # Determine if the robot is stuck
        if node.high_risk_count >= 5: 
            node.stuck = 1.0
        else:
            node.stuck = 0.0

        node.led2_pub.publish(led_msg)

    except Exception as e:
        node.stuck = 1.0
        led_msg.value = Led.RED
        node.get_logger().error(f'Erro na callback do LiDAR: {str(e)}')
        node.led2_pub.publish(led_msg)

    # Publish safety data
    safety_data = Float32MultiArray()
    safety_data.data = [float(closest_dist), float(closest_dir), float(risk_val), float(node.stuck)]
    node.safety_pub.publish(safety_data)

    # Publish zone information
    zone_size = Float32MultiArray()
    zone_size.data = [r_clear, r_warning, r_critical]
    node.safe_zone_pub.publish(zone_size)


def speed_callback(data, node):
    # Callback function for processing odometry data. Calculates the robot's speed.
    node.speed = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('action_complete_safety')

    # Initialize variables
    init_var(node)
    init_fls_common_part(node)
    init_risk_assessment(node)
    init_risk_mitigation(node)

    # Configure QoS
    qos_profile_reliable = QoSProfile(depth=10)
    qos_profile_reliable.reliability = ReliabilityPolicy.RELIABLE

    qos_profile_best_effort = QoSProfile(depth=10)
    qos_profile_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT

    # Publishers
    node.safe_vel_pub = node.create_publisher(VelocityScale, '/safety/vel_scale', qos_profile_reliable)
    node.led2_pub = node.create_publisher(Led, '/commands/led2', qos_profile_reliable)
    node.safety_pub = node.create_publisher(Float32MultiArray, '/safety/obstacles', qos_profile_reliable)
    node.safe_zone_pub = node.create_publisher(Float32MultiArray, '/safety/safety_zone', qos_profile_reliable)

    # Subscribers
    node.create_subscription(LaserScan, '/lidar_scan', lambda msg: lidar_callback(msg, node), qos_profile_best_effort)
    node.create_subscription(Odometry, '/odom', lambda msg: speed_callback(msg, node), qos_profile_reliable)

    node.get_logger().info("SafetyNode is now running!")

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {str(e)}')
    except:
        node.led2_pub.publish(Led(value=Led.RED))
    

if __name__ == '__main__':
    main()