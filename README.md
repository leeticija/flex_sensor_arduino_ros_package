# Flex Sensor ROS Package

ROS package for reading and publishing data from flex sensors connected to Arduino.

## Hardware Requirements
- Arduino board
- Flex sensors
- Voltage divider circuit for each sensor

## Software Requirements
- ROS Noetic
- Arduino IDE
- rosserial_arduino package

## Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/leeticija/flex_sensor_arduino_ros_package.git
cd ~/catkin_ws
catkin_make
