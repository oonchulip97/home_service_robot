#!/bin/sh

# Launch robot in world
xterm  -e  "roslaunch my_robot world.launch" &
sleep 5

# Launch SLAM
xterm  -e  "rosrun gmapping slam_gmapping _base_frame:=robot_footprint" &
sleep 5

# Launch keyboard control
xterm  -e  "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
sleep 5
