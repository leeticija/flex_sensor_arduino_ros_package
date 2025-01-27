# Flex Sensor ROS Package

ROS package for reading and publishing data from flex sensors connected to Arduino.

## Hardware Requirements
- Arduino board
- Flex sensors

## Software Requirements
- ROS Noetic
- Arduino IDE

## Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/leeticija/flex_sensor_arduino_ros_package.git
cd ~/catkin_ws
catkin_make
```
## Runnig
```bash
roslaunch robosoft sofia.launch
rosrun robosoft flex_sensor_node.py
```