#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dm6814_library.h>
#include "dm6604.h"

class SpaceRobot : public hardware_interface::RobotHW
{
public:
  void writeMotors()
  {
    dac.updateDAC(1, cmd[0]);
    dac.updateDAC(2, cmd[1]);
  }

  void readEncoders()
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

    pos[0]=(double)encoder_1;
    pos[1]=(double)encoder_2;
  }




  SpaceRobot() 
  { 
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_left_shoulder("left_shoulder", &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle state_handle_left_elbow("left_elbow", &pos[1], &vel[1], &eff[1]);

    jnt_state_interface.registerHandle(state_handle_left_shoulder);
    jnt_state_interface.registerHandle(state_handle_left_elbow);

    registerInterface(&jnt_state_interface);

    // connect and register the joint effort interface
    hardware_interface::JointHandle effort_handle_left_shoulder(jnt_state_interface.getHandle("left_shoulder"), &cmd[0]);
    hardware_interface::JointHandle effort_handle_left_elbow(jnt_state_interface.getHandle("left_elbow"), &cmd[1]);

    jnt_eff_interface.registerHandle(effort_handle_left_shoulder);
    jnt_eff_interface.registerHandle(effort_handle_left_elbow);

    registerInterface(&jnt_eff_interface);



    //ENCODER CARD


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

    encoder_1_ovf = 0;
    encoder_2_ovf = 0;
    encoder_3_ovf = 0;
    encoder_4_ovf = 0;
    encoder_5_ovf = 0;
    encoder_6_ovf = 0;

    encoder_1 = encoder_init_value;
    encoder_2 = encoder_init_value;
    encoder_3 = encoder_init_value;
    encoder_4 = encoder_init_value;
    encoder_5 = encoder_init_value;
    encoder_6 = encoder_init_value;


    dac = DM6604Device(640); //0x280 
    dac.configureIOPorts(OUTPUT,OUTPUT,OUTPUT);
    dac.setDACRange(20, BIPOLAR);
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  static DM6814Device dm6814_right;
  static DM6814Device dm6814_left;

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
  int encoder_1_ovf;
  int encoder_2_ovf;
  int encoder_3_ovf;
  int encoder_4_ovf;
  int encoder_5_ovf;
  int encoder_6_ovf;

  //ENCODER VALUES TO TRANSMIT
  int encoder_1;
  int encoder_2;
  int encoder_3;
  int encoder_4;
  int encoder_5;
  int encoder_6;


  //DAC
  DM6604Device dac;
};