# ROS2 Turtlesim Autonomous Navigation

## Candidate Name
Satwik Halappanavar

## GitHub Username
Satwik-hub5


---

## Overview
This project implements autonomous navigation for the ROS2 turtlesim robot using odometry and TF broadcasting.

---

## Build Instructions

source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash



## Run Instructions

Terminal 1:
ros2 run turtlesim turtlesim_node

Terminal 2:
ros2 run turtle_autonomy turtle_odometry

Terminal 3:
ros2 run turtle_autonomy turtle_autonomy



## Approach

- Converted turtlesim pose to nav_msgs/Odometry
- Published odometry on /turtle1/odom
- Broadcasted TF frames: map → odom → base_link
- Implemented autonomous navigation using unicycle model
- Turtle selects angular velocity to move toward goal

## Contact Info

Name: Satwik Halappanavar  
Phone: +91 7899076437  
Email: satwwikhalapanavarr@gmail.com
