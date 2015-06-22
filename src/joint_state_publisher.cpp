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
    ros::Rate loop_rate(500);


    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(5);
    joint_state.position.resize(5);
    joint_state.name[0] ="right_shoulder";
    joint_state.position[0] = 0;
    joint_state.name[1] ="right_elbow";
    joint_state.position[1] = 0;
    joint_state.name[2] ="left_shoulder";
    joint_state.position[2] = 0;
    joint_state.name[3] ="left_elbow";
    joint_state.position[3] = 0;
    joint_state.name[4] ="wheel";
    joint_state.position[4] = 0;

    static DM6814Device dm6814_right;
    static DM6814Device dm6814_left;

    // Encoder cards
    uint32_t minor_number = 0;
    uint32_t minor_number_2 = 1;
    if (!dm6814_right.OpenBoard6814(minor_number)) 
        ROS_ERROR("ERROR: OpenBoard6814() FAILED");
    if (!dm6814_right.InitBoard6814()) 
        ROS_ERROR("ERROR: InitBoard6814() FAILED");

    if (!dm6814_left.OpenBoard6814(minor_number_2)) 
        ROS_ERROR("ERROR: OpenBoard6814() FAILED");
    if (!dm6814_left.InitBoard6814()) 
        ROS_ERROR("ERROR: InitBoard6814() FAILED");

    //second card init for left hand
    for (int i=1; i<=3; i++)
    {
        // Disable the encoder
        if (! dm6814_right.EnableEncoder6814(i, false)) 
            ROS_ERROR("ERROR: EnableEncoder6814(%d, false) FAILED", i); 
        // Disable encoder interrupt
        if (! dm6814_right.EnableEncoderIrq6814(i, false))
            ROS_ERROR("ERROR: EnableEncoderIrq6814(%d) FAILED", i);
        //Load a value into the encoder.  The encoder should be disabled before
        //loading a value into it.
        if (! dm6814_right.LoadEncoder6814(i, 0x9C40))
            ROS_ERROR("ERROR: LoadEncoder6814(%d) FAILED", i);
        // Enable the encoder
        if (! dm6814_right.EnableEncoder6814(i, true)) 
            ROS_ERROR("ERROR: EnableEncoder6814(%d, true) FAILED", i);
    }
    //second card init for left hand
    for (int i=1; i<=3; i++)
    {
        // Disable the encoder
        if (! dm6814_left.EnableEncoder6814(i, false)) 
            ROS_ERROR("ERROR: EnableEncoder6814(%d, false) FAILED", i); 
        // Disable encoder interrupt
        if (! dm6814_left.EnableEncoderIrq6814(i, false))
            ROS_ERROR("ERROR: EnableEncoderIrq6814(%d) FAILED", i);
        //Load a value into the encoder.  The encoder should be disabled before
        //loading a value into it.
        if (! dm6814_left.LoadEncoder6814(i, 0x9C40))
            ROS_ERROR("ERROR: LoadEncoder6814(%d) FAILED", i);
        // Enable the encoder
        if (! dm6814_left.EnableEncoder6814(i, true)) 
            ROS_ERROR("ERROR: EnableEncoder6814(%d, true) FAILED", i);
    }

    //encoder types
    uint16_t encoder_1_val;
    uint16_t encoder_2_val;
    uint16_t encoder_3_val;
    uint16_t encoder_4_val;
    uint16_t encoder_5_val;
    uint16_t encoder_6_val;

    int16_t encoder_2_val_int;

    while (ros::ok()) 
    {
    //read robots joint state
        //right card
        if (! dm6814_right.ReadEncoder6814(1, &encoder_1_val))
            ROS_ERROR("ERROR: ReadEncoder6814(1) FAILED");

        if (! dm6814_right.ReadEncoder6814(2, &encoder_2_val))
            ROS_ERROR("ERROR: ReadEncoder6814(2) FAILED");

        if (! dm6814_right.ReadEncoder6814(3, &encoder_3_val))
            ROS_ERROR("ERROR: ReadEncoder6814(3) FAILED");
        //left card
        if (! dm6814_left.ReadEncoder6814(1, &encoder_4_val))
            ROS_ERROR("ERROR: ReadEncoder6814(1) FAILED");

        if (! dm6814_left.ReadEncoder6814(2, &encoder_5_val))
            ROS_ERROR("ERROR: ReadEncoder6814(2) FAILED");

        if (! dm6814_left.ReadEncoder6814(3, &encoder_6_val))
            ROS_ERROR("ERROR: ReadEncoder6814(3) FAILED");

        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = (double)encoder_1_val/(40*2000);
        joint_state.position[1] = (double)encoder_2_val/(40*2000);
        //joint_state.position[6] = (double)encoder_3_val;
        joint_state.position[2] = (double)encoder_4_val/(40*2000);;
        joint_state.position[3] = (double)encoder_5_val/(40*2000);;
        joint_state.position[4] = (double)encoder_6_val/(40*2000);;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    (void) dm6814_right.CloseBoard6814();
    (void) dm6814_left.CloseBoard6814();
    return 0;
}