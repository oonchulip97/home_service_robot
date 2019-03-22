#!/bin/sh

# Launch robot in world
xterm  -e  "roslaunch my_robot world.launch" &
sleep 5

# Launch navigation
xterm  -e  "roslaunch my_robot home_service.launch" &
sleep 5

# Launch pick_objects
xterm  -e  "rosrun pick_objects pick_objects" &
sleep 5

# Launch add_markers
xterm  -e  "rosrun add_markers add_markers" &
sleep 5
