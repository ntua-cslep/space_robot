#!/bin/bash
#
# Script to load enviroment variables
#
# Ilias Patsiaouras
# Mo 27 Jul 2015
#
#
#
export ROS_MASTER_URI=http://192.168.1.160:11311
export ROS_HOSTNAME=192.168.1.160
export ROS_IP=192.168.1.160 
printf "\033[1;34m The Following enviroment variables changed \nROS_IP: %s \nROS_HOSTNAME: %s \nROS_MASTER_URI: %s \033[0m\n" $ROS_IP $ROS_HOSTNAME $ROS_MASTER_URI
# source /home/ros/catkin_ws/devel/setup.bash
# printf "\033[1;34m \nsourced: /home/ros/catkin_ws/devel/setup.bash \033[0m\n"
