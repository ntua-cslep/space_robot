# space_robot
A ROS package tah include all the source code that runs on Space robot's PC's.

# Instructions

## In every boot run the following commands: 
- sudo chmod a+rw /dev/input/by-path/"your mouse path"         
tip:run this for all mouses
- cd ~/catkin_ws
- soure devel/setup.bash
- export ROS_MASTER_URI=http://192.168.1.231:11311
- export ROS_HOSTNAME=192.168.1.231
- export ROS_IP=192.168.1.231
