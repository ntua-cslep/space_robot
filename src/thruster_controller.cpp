#include <ros/ros.h>
#include "dm6604.h"

DM6604Device dac(640); //0x280 
int count=0;
int duty = 200;

void timerCallback(const ros::TimerEvent& e)
{
	ROS_INFO("count: %d, duty: %d", count, duty);

	if(count > 1000) count = 0;

	if(count > duty) dac.writeDigitalIO(0, 1);//portA
	else dac.writeDigitalIO(0, 0);//portA
		
	count++;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "thruster_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    dac.configureIOPorts(OUTPUT,OUTPUT,OUTPUT);

    ros::Timer timer = n.createTimer(ros::Duration(0.001), timerCallback);



    ros::spin();
    
    return 0;
}