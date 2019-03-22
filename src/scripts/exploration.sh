#!/bin/sh

# Launch robot in world
xterm  -e  "roslaunch my_robot world.launch" &
sleep 5

# Launch exploration
xterm  -e  "roslaunch my_robot exploration.launch" &
sleep 5

