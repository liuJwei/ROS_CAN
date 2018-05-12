# ROS_CAN
Send data via USB-CAN in ROS

Requirements
---
CANalyst-II Linux version<br>
ROS kinetic

Steps
----

1. Creating a catkin Package, for example:
```
cd ~/catkin_ws/src
catkin_create_pkg can_send_lv std_msgs rospy roscpp
```
2. Adding or replacing with these files in this repository. The two files in the `inlcude` dictionary are provied by the USB-CAN seller

3. Building the catkin workspace
```
cd ~/catkin_ws
catkin_make 
```
4. Running 
```
roscore
```
In a new terminal,
```
rosrun can_send_lv can_send
```
The normal user should have the authority to read/write the corresponding USB device. 
