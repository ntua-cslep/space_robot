#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "cassiopeia_hw.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "space_robot_interface");
    ros::NodeHandle n;
    ros::Rate loop_rate(200.0);

    CassiopeiaHW robot;
    controller_manager::ControllerManager cm(&robot);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();

    while(ros::ok())
    {
        ros::Time curr_time = ros::Time::now();
        ros::Duration period = curr_time - prev_time;
        prev_time = curr_time;

        robot.readEncoders(period);
        cm.update(curr_time, period);
        robot.writeMotors();

        loop_rate.sleep();
    }

    return 0;
}