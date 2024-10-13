#!/bin/bash

cd /root/workspace
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch websocket_server websocket_server.launch

tail -f /dev/null