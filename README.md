# Overview
This is a home service robot simulation that can pick up an object at one location and drop off at another.

This project was tested using:
- Lubuntu
- ROS Kinetic
- C++ 11

# Setup
Firstly, clone this workspace into user's home directory.

Install mapping package.
```
sudo apt-get install ros-kinetic-gmapping
```

Install navigation package.
```
sudo apt-get install ros-kinetic-move-base
```

Install teleop package.
```
sudo apt-get install ros-kinetic-move-base
```

Install localization package.
```
sudo apt-get install ros-kinetic-robot-localization
```

Finally build the workspace.
```
cd ~/robotics_ws/
catkin_make
```

Don't forget to source the workspace.
```
source ~/robotics_ws/devel/setup.bash
```

# Running

Run the following in a terminal.
```
cd ~/robotics_ws/src/scripts
./home_service.sh
```
This will launch the necessary nodes and the simulation will begin.

# Mechanism
[![Home Service Robot in Action](https://youtu.be/aDHzU-5HDXc/0.jpg)](https://youtu.be/aDHzU-5HDXc "Home Service Robot in Action")
The objective is to develop a home service robot to pick up an object from one location and drop off the object in another location. The code is written in C++ under ROS (Robot Operating System) framework. The environment is simulated in Gazebo while visualization is achieved through rViz. The map of the environment can be generated through the Gmapping SLAM package and Frontier Exploration package autonomously. Then, the localization of the robot is handled by both the Robot Localization EKF (Extended Kalman Filter) package and AMCL (adaptive Monte Carlo localization) package in order to fuse odometry, IMU and laser scan data. Navigation of the robot is implemented by the Move Base package. ROS nodes are utilized to publish goal positions and also to display markers in the visualization.
