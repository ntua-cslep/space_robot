#!/bin/bash          
#
# Script to load RTD DM6814 encoder card drivers module
# 
# Ilias Patsiaouras
# Mo 27 Jul 2015 
#
# 
#

printf "\033[1;34m \nDM6814 encoder card drivers loading script started\033[0m\n"
make -C /home/ros/catkin_ws/src/space_robot/DM6814_Linux_V02.02.00_Preliminary/driver insmod
make -C /home/ros/catkin_ws/src/space_robot/DM6814_Linux_V02.02.00_Preliminary/driver devices
printf "\033[1;34m DM6814 encoder card drivers loading script ended\033[0m\n\n"