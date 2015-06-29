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

    //ROS PARAMETERS
    float cpr;
    ros::param::param<float>("~counts_per_revolution", cpr, 2048);
    float gear_ratio;
    ros::param::param<float>("~gear_ratio", gear_ratio, 55);

    //ROS JOINT STATES
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

    //ENCODER CARD
    static DM6814Device dm6814_right;
    static DM6814Device dm6814_left;

    // TRY ACCESS CARDS
    uint32_t right_minor_number = 0;
    uint32_t  left_minor_number = 1;

    if (!dm6814_right.OpenBoard6814(right_minor_number)) 
        ROS_ERROR("ERROR: OpenBoard6814() FAILED");
    if (!dm6814_right.InitBoard6814()) 
        ROS_ERROR("ERROR: InitBoard6814() FAILED");

    if (!dm6814_left.OpenBoard6814(left_minor_number)) 
        ROS_ERROR("ERROR: OpenBoard6814() FAILED");
    if (!dm6814_left.InitBoard6814()) 
        ROS_ERROR("ERROR: InitBoard6814() FAILED");

    // LOADING INITIAL VALUES 
    uint16_t encoder_init_value = 0;
  
    //RIGHT
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
        if (! dm6814_right.LoadEncoder6814(i, encoder_init_value ) )
            ROS_ERROR("ERROR: LoadEncoder6814(%d) FAILED", i);
        // Enable the encoder
        if (! dm6814_right.EnableEncoder6814(i, true)) 
            ROS_ERROR("ERROR: EnableEncoder6814(%d, true) FAILED", i);
    }
    
    //LEFT
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
        if (! dm6814_left.LoadEncoder6814(i, encoder_init_value))
            ROS_ERROR("ERROR: LoadEncoder6814(%d) FAILED", i);
        // Enable the encoder
        if (! dm6814_left.EnableEncoder6814(i, true)) 
            ROS_ERROR("ERROR: EnableEncoder6814(%d, true) FAILED", i);
    }

    //ENCODER VALUES FROM CARD
    uint16_t encoder_1_val;
    uint16_t encoder_2_val;
    uint16_t encoder_3_val;
    uint16_t encoder_4_val;
    uint16_t encoder_5_val;
    uint16_t encoder_6_val;

    //ENCODER VALUES HISTORY FROM CARD
    uint16_t encoder_1_old = encoder_init_value;
    uint16_t encoder_2_old = encoder_init_value;
    uint16_t encoder_3_old = encoder_init_value;
    uint16_t encoder_4_old = encoder_init_value;
    uint16_t encoder_5_old = encoder_init_value;
    uint16_t encoder_6_old = encoder_init_value;

    //ENCODER OVERFLOW COUNTERS
    int encoder_1_ovf = 0;
    int encoder_2_ovf = 0;
    int encoder_3_ovf = 0;
    int encoder_4_ovf = 0;
    int encoder_5_ovf = 0;
    int encoder_6_ovf = 0;

    //ENCODER VALUES TO TRANSMIT
    int encoder_1 = encoder_init_value;
    int encoder_2 = encoder_init_value;
    int encoder_3 = encoder_init_value;
    int encoder_4 = encoder_init_value;
    int encoder_5 = encoder_init_value;
    int encoder_6 = encoder_init_value;
    

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


    //Handling of encoder value overflow
        if      ((int)encoder_1_val - (int)encoder_1_old < -62000) encoder_1_ovf++;
        else if ((int)encoder_1_val - (int)encoder_1_old >  62000) encoder_1_ovf--;
        encoder_1_old = encoder_1_val;
        encoder_1 = (int)encoder_1_val + 65535*encoder_1_ovf;

        if      ((int)encoder_2_val - (int)encoder_2_old < -62000) encoder_2_ovf++;
        else if ((int)encoder_2_val - (int)encoder_2_old >  62000) encoder_2_ovf--;
        encoder_2_old = encoder_2_val;
        encoder_2 = (int)encoder_2_val + 65535*encoder_2_ovf;

        if      ((int)encoder_3_val - (int)encoder_3_old < -62000) encoder_3_ovf++;
        else if ((int)encoder_3_val - (int)encoder_3_old >  62000) encoder_3_ovf--;
        encoder_3_old = encoder_3_val;
        encoder_3 = (int)encoder_3_val + 65535*encoder_3_ovf;

        if      ((int)encoder_4_val - (int)encoder_4_old < -62000) encoder_4_ovf++;
        else if ((int)encoder_4_val - (int)encoder_4_old >  62000) encoder_4_ovf--;
        encoder_4_old = encoder_4_val;
        encoder_4 = (int)encoder_4_val + 65535*encoder_4_ovf;

        if      ((int)encoder_5_val - (int)encoder_5_old < -62000) encoder_5_ovf++;
        else if ((int)encoder_5_val - (int)encoder_5_old >  62000) encoder_5_ovf--;
        encoder_5_old = encoder_5_val;
        encoder_5 = (int)encoder_5_val + 65535*encoder_5_ovf;

        if      ((int)encoder_6_val - (int)encoder_6_old < -62000) encoder_6_ovf++;
        else if ((int)encoder_6_val - (int)encoder_6_old >  62000) encoder_6_ovf--;
        encoder_6_old = encoder_6_val;
        encoder_6 = (int)encoder_6_val + 65535*encoder_6_ovf;


    //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = (double)encoder_1/(gear_ratio*cpr);
        joint_state.position[1] =-(double)encoder_2/(gear_ratio*cpr);
      //joint_state.position[5] = (double)encoder_3_val;
        joint_state.position[2] = (double)encoder_4/(gear_ratio*cpr);
        joint_state.position[3] = (double)encoder_5/(gear_ratio*cpr);
        joint_state.position[4] = (double)encoder_6/(gear_ratio*cpr);

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    (void) dm6814_right.CloseBoard6814();
    (void) dm6814_left.CloseBoard6814();
    return 0;
}