# This script reads data from a ROS 2 bag file, extracts transformation, odometry, and error data,
# and plots the positions and Euclidean error over time using matplotlib.
import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from turtlebot2i_interfaces.msg import EvalError
import matplotlib.pyplot as plt
import sys

def read_bag_data(bag_path):
    # Initialize ROS 2
    rclpy.init()

    # Create a reader for the bag file
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Define the types of messages we are interested in
    msg_types = {
        '/tf': TFMessage,
        '/odom_camera': Odometry,
        '/odom_euclidian_error': EvalError,
    }

    tf_data = []
    odom_data = []
    euclidian_error_data = []

    # Read messages from the bag file
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        msg_type = msg_types.get(topic)
        if msg_type:
            msg = deserialize_message(data, msg_type)
            if topic == '/tf':
                for transform in msg.transforms:
                    if transform.child_frame_id == '/kinect':
                        tf_data.append((timestamp, transform))
            elif topic == '/odom_camera':
                odom_data.append((timestamp, msg))
            elif topic == '/odom_euclidian_error':
                euclidian_error_data.append((timestamp, msg))

    rclpy.shutdown()
    return tf_data, odom_data, euclidian_error_data

def plot_data(tf_data, odom_data, errors):
    # Extract data for plotting
    tf_times = [stamp/1e9 for (stamp, _) in tf_data]
    tf_positions = [(tfmsg.transform.translation.x, tfmsg.transform.translation.y, tfmsg.transform.translation.z) for (_, tfmsg) in tf_data]

    odom_times = [stamp/1e9 for (stamp, _) in odom_data]
    odom_positions = [(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) for (_, msg) in odom_data]

    error_times = [stamp/1e9 for (stamp, _) in errors]
    error_values = [error.data for (_, error) in errors]

    # Set font sizes for titles and labels
    title_fontsize = 16
    label_fontsize = 14

    # Create figure and axes for positions
    fig1, axs1 = plt.subplots(3, 1, figsize=(8, 8), constrained_layout=True)

    # Plot x positions
    axs1[0].plot(tf_times, [pos[0] for pos in tf_positions], label='TF x')
    axs1[0].plot(odom_times, [pos[0] for pos in odom_positions], label='Odom x', linestyle='dashed')
    axs1[0].set_xlabel('Time (s)', fontsize=label_fontsize)
    axs1[0].set_ylabel('Position X (m)', fontsize=label_fontsize)
    axs1[0].legend()
    axs1[0].grid()
    axs1[0].tick_params(axis='both', which='major', labelsize=label_fontsize)

    # Plot y positions
    axs1[1].plot(tf_times, [pos[1] for pos in tf_positions], label='TF y')
    axs1[1].plot(odom_times, [pos[1] for pos in odom_positions], label='Odom y', linestyle='dashed')
    axs1[1].set_xlabel('Time (s)', fontsize=label_fontsize)
    axs1[1].set_ylabel('Position Y (m)', fontsize=label_fontsize)
    axs1[1].legend()
    axs1[1].grid()
    axs1[1].tick_params(axis='both', which='major', labelsize=label_fontsize)

    # Plot z positions
    axs1[2].plot(tf_times, [pos[2] for pos in tf_positions], label='TF z')
    axs1[2].plot(odom_times, [pos[2] for pos in odom_positions], label='Odom z', linestyle='dashed')
    axs1[2].set_xlabel('Time (s)', fontsize=label_fontsize)
    axs1[2].set_ylabel('Position Z (m)', fontsize=label_fontsize)
    axs1[2].legend()
    axs1[2].grid()
    axs1[2].tick_params(axis='both', which='major', labelsize=label_fontsize)

    # Set titles for each subplot
    axs1[0].set_title('X Position Over Time', fontsize=title_fontsize)
    axs1[1].set_title('Y Position Over Time', fontsize=title_fontsize)
    axs1[2].set_title('Z Position Over Time', fontsize=title_fontsize)

    # Figure 2: Error
    fig2, ax2 = plt.subplots(figsize=(10, 4), constrained_layout=True)
    ax2.plot(error_times, error_values, label='Error')
    ax2.set_xlabel('Time (s)', fontsize=label_fontsize)
    ax2.set_ylabel('Euclidean Error (m)', fontsize=label_fontsize)
    ax2.legend()
    ax2.grid()
    ax2.tick_params(axis='both', which='major', labelsize=label_fontsize)

    # Set title for error plot
    ax2.set_title('Euclidean Error Over Time', fontsize=title_fontsize)

    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python plot_bag_data.py <path_to_bag>")
        sys.exit(1)

    bag_path = sys.argv[1]
    tf_data, odom_data, errors = read_bag_data(bag_path)
    plot_data(tf_data, odom_data, errors)
