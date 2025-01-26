#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import serial
import json

def main():
    # Initialize ROS node
    rospy.init_node('flex_sensor_node')
    
    # Create publisher
    pub = rospy.Publisher('flex_sensor_data', Float32MultiArray, queue_size=10)
    
    # Configure serial connection
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200)
        rospy.loginfo("Connected to /dev/ttyACM0")
    except serial.SerialException as e:
        rospy.logerr("Could not connect to /dev/ttyACM0: %s" % e)
        return
    
    rate = rospy.Rate(100)  # 100Hz
    
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting:
                # Read line from serial
                line=""
                try:
                	line = ser.readline().decode('utf-8').strip()
                except:
                	print("Exception in decoding utf-8.")
                	continue
                
                try:
                    # Parse JSON data
                    data = json.loads(line)
                    
                    # Create message
                    msg = Float32MultiArray()
                    msg.data = [data['x'], data['y']]
                    
                    # Publish message
                    pub.publish(msg)
                    
                except json.JSONDecodeError as e:
                    rospy.logwarn("Could not parse JSON data: %s" % line)
                except KeyError as e:
                    rospy.logwarn("Missing key in JSON data: %s" % e)
                    
        except serial.SerialException as e:
            rospy.logerr("Serial error: %s" % e)
            break
            
        rate.sleep()
    
    # Close serial connection
    ser.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
