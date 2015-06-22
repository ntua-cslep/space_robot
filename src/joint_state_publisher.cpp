#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/UInt16.h"

#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <dm6814_library.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(100);


    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.name[0] ="shoulder";
    joint_state.position[0] = 0;
    joint_state.name[1] ="elbow";
    joint_state.position[1] = 0;
    joint_state.name[2] ="wrist";
    joint_state.position[2] = 0;
    joint_state.name[3] ="intruder";
    joint_state.position[3] = 0;

    static DM6814Device DM6814;
    uint32_t minor_number = 1;
    // Encoder cards
    if (!DM6814.OpenBoard6814(minor_number)) 
        ROS_ERROR("ERROR: OpenBoard6814() FAILED");
    if (!DM6814.InitBoard6814()) 
        ROS_ERROR("ERROR: InitBoard6814() FAILED");
    
    for (int i=1; i<=3; i++)
    {
        // Disable the encoder
        if (! DM6814.EnableEncoder6814(1, false)) 
            ROS_ERROR("ERROR: EnableEncoder6814(%d, false) FAILED", i); 
        // Disable encoder interrupt
        if (! DM6814.EnableEncoderIrq6814(1, false))
            ROS_ERROR("ERROR: EnableEncoderIrq6814(%d) FAILED", i);
        //Load a value into the encoder.  The encoder should be disabled before
        //loading a value into it.
        if (! DM6814.LoadEncoder6814(1, 0x0000))
            ROS_ERROR("ERROR: LoadEncoder6814(%d) FAILED", i);
        // Enable the encoder
        if (! DM6814.EnableEncoder6814(1, true)) 
            ROS_ERROR("ERROR: EnableEncoder6814(%d, true) FAILED", i);
    }

    //encoder types
    uint16_t encoder_1_val;
    uint16_t encoder_2_val;
    uint16_t encoder_3_val;

    while (ros::ok()) 
    {
        //read robots joint state
        if (! DM6814.ReadEncoder6814(1, &encoder_1_val))
            ROS_ERROR("ERROR: ReadEncoder6814(1) FAILED");

        if (! DM6814.ReadEncoder6814(2, &encoder_2_val))
            ROS_ERROR("ERROR: ReadEncoder6814(2) FAILED");

        if (! DM6814.ReadEncoder6814(3, &encoder_3_val))
            ROS_ERROR("ERROR: ReadEncoder6814(3) FAILED");

        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = (double)encoder_1_val;
        joint_state.position[1] = (double)encoder_2_val;
        joint_state.position[2] = (double)encoder_3_val;
        joint_state.position[3] = 0;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    (void) DM6814.CloseBoard6814();
    return 0;
}