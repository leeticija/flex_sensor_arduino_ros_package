import rosbag
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import argparse
import os

def extract_dynamixel_data(msg):
    """
    Extract data from a DynamixelState message.
    
    Args:
        msg: Message containing list of DynamixelState
        
    Returns:
        dict: Dictionary with motor name as key and its data as value
    """
    data = defaultdict(dict)
    
    # Handle individual motor state
    for motor_state in msg.dynamixel_state:
        motor_name = motor_state.name
        data[motor_name] = {
            'position': motor_state.present_position,
            'velocity': motor_state.present_velocity,
            'current': motor_state.present_current
        }
    
    return data

def extract_flex_data(msg):
    """
    Extract data from flex sensor message.
    
    Args:
        msg: Message containing flex sensor data
        
    Returns:
        list: List of flex sensor values
    """
    return msg.data

def read_bag_data(bag_path, topic_name):
    """
    Read data from a ROS bag file for a specific topic.
    
    Args:
        bag_path (str): Path to the ROS bag file
        topic_name (str): Name of the topic to extract
        
    Returns:
        tuple: (timestamps, data_dict)
    """
    timestamps = []
    motor_data = defaultdict(lambda: defaultdict(list))  # nested dict for each motor's data
    flex_data = defaultdict(list)  # for flex sensor data
    
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            timestamp = t.to_sec()
            timestamps.append(timestamp)
            
            if 'dynamixel_state' in topic:
                state_data = extract_dynamixel_data(msg)
                for motor_name, motor_state in state_data.items():
                    for key, value in motor_state.items():
                        motor_data[motor_name][key].append(value)
            else:  # flex sensor data
                sensor_data = extract_flex_data(msg)
                # Only append the first sensor value
                flex_data['flex_angle'].append(sensor_data[0])
    
    # Normalize timestamps to start at 0
    if timestamps:
        start_time = timestamps[0]
        timestamps = [t - start_time for t in timestamps]
    
    # Convert lists to numpy arrays
    if 'dynamixel_state' in topic_name:
        return np.array(timestamps), motor_data
    else:
        return np.array(timestamps), {k: np.array(v) for k, v in flex_data.items()}

def plot_dynamixel_data(fig, timestamps, motor_data):
    """
    Plot Dynamixel motor data.
    
    Args:
        fig: Matplotlib figure
        timestamps: Array of timestamps
        motor_data: Dictionary of motor data
    """
    parameters = ['position', 'velocity', 'current']
    num_motors = len(motor_data)
    
    axes = fig.subplots(len(parameters), 1, sharex=True)
    
    for param_idx, param in enumerate(parameters):
        ax = axes[param_idx]
        
        for motor_name, motor_state in motor_data.items():
            if param in motor_state:
                data = np.array(motor_state[param])
                ax.plot(timestamps, data, '-', label=f'{motor_name}')
        
        ax.set_ylabel(param.capitalize())
        ax.grid(True)
        ax.legend()
    
    axes[-1].set_xlabel('Time (seconds)')
    fig.suptitle('Dynamixel Motor Data')

def plot_flex_data(fig, timestamps, flex_data):
    """
    Plot flex sensor data.
    
    Args:
        fig: Matplotlib figure
        timestamps: Array of timestamps
        flex_data: Dictionary of flex sensor data
    """
    ax = fig.add_subplot(111)
    
    for sensor_name, data in flex_data.items():
        ax.plot(timestamps, data, '-', label=sensor_name)
    
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Flex Angle (degrees)')
    ax.grid(True)
    ax.legend()
    fig.suptitle('Flex Sensor Data')

def plot_rosbag_data(bag_path, topics, title="ROS Bag Data"):
    """
    Create plots from ROS bag file.
    
    Args:
        bag_path (str): Path to the ROS bag file
        topics (list): List of topic names to plot
        title (str): Title for the plot
    """
    for topic in topics:
        if 'dynamixel_state' in topic:
            fig = plt.figure(figsize=(12, 10))
        else:
            fig = plt.figure(figsize=(12, 6))
            
        timestamps, data = read_bag_data(bag_path, topic)
        
        if 'dynamixel_state' in topic:
            plot_dynamixel_data(fig, timestamps, data)
        else:
            plot_flex_data(fig, timestamps, data)
        
        plt.tight_layout()
    
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Plot data from ROS bag file')
    parser.add_argument('bagfile', help='Path to the ROS bag file')
    parser.add_argument('--topics', nargs='+', default=[
        "/dynamixel_workbench/dynamixel_state",
        "/flex_sensor_data"
    ], help='Topics to plot (default: dynamixel state and flex sensor)')
    
    args = parser.parse_args()
    
    # Verify bag file exists
    if not os.path.exists(args.bagfile):
        print(f"Error: Bag file '{args.bagfile}' not found")
        return
    
    # Plot the data
    plot_rosbag_data(args.bagfile, args.topics)

if __name__ == "__main__":
    main()
