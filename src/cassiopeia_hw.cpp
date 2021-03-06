#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <dm6814_library.h>
#include "cassiopeia_hw.h"
#include <math.h>


void CassiopeiaHW::writeMotors()
{
  dac.updateDAC(1, -cmd[0]);
  dac.updateDAC(2,  cmd[1]);
  dac.updateDAC(3, -cmd[3]);
  dac.updateDAC(4, -cmd[4]);
  dac.updateDAC(5, -cmd[5]);

  eff[0] = cmd[0];
  eff[1] = cmd[1];
  // eff[2] = cmd[2];
  eff[3] = cmd[3];
  eff[4] = cmd[4];
  eff[5] = cmd[5];
}

void CassiopeiaHW::readEncoders(ros::Duration dt)
{
// read robots joint state
  // right card
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



// Handling of encoder value overflow
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

  ROS_DEBUG("1: %d, 2: %d, 3: %d, 4: %d, 5: %d, 6: %d", encoder_1, encoder_2, encoder_3, encoder_4, encoder_5, encoder_6);

// Position Calculation
  pos[0]=  (double)encoder_1*2*M_PI/(4096*190) - offset_pos[0];
  pos[1]= -(double)encoder_2*2*M_PI/(4096*190) - offset_pos[1];
  pos[2]=  (double)encoder_3*2*M_PI/(4096*190) - offset_pos[2];//disconnected
  
  pos[3]=  (double)encoder_4*2*M_PI/(4096*190) - offset_pos[3];
  pos[4]=  (double)encoder_5*2*M_PI/(4096*190) - offset_pos[4];
  pos[5]=  (double)encoder_6*2*M_PI/ 4096      - offset_pos[5];

  ROS_DEBUG("encoder6: %d, pos (10^3): %d", encoder_6, (int)(pos[5]*1000));
  
  ROS_DEBUG("POS: 1: %f, 2: %f, 3: %f, 4: %f, 5: %f, 6: %f", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

// Speed Calculation
  for(int i=0; i<6; i++)
  {
    vel[i]=(pos[i] - prev_pos[i])/dt.toSec();
    prev_pos[i] = pos[i];
  }

}

bool CassiopeiaHW::checkLimits()
{
  uint8_t port_right, port_left;

  static uint8_t prev_port_right=255, prev_port_left=255;
  bool status=false;

  if (! dm6814_right.ReadEncoderDIO6814(1, &port_right))
    ROS_ERROR("ERROR: Right ReadEncoderDIO(1) FAILED");

  if (! dm6814_left.ReadEncoderDIO6814(1, &port_left))
    ROS_ERROR("ERROR: left ReadEncoderDIO(1) FAILED");
  
  if (port_right != prev_port_right)
  {
    if (0b00000100 == (~port_right & 0b00000100)) 
    {
      offset_pos[0] += pos[0] + 1.57;
      status |= true;
    }
    if (0b00001000 == (~port_right & 0b00001000)) 
    {
      offset_pos[1] += pos[1] + 1.1;
      status |= true;
    }
  }

  if (port_left != prev_port_left)
  {
    if (0b00000100 == (~port_left & 0b00000100)) 
    {
      offset_pos[3] += pos[3] - 1.57;
      status |= true;
    }
    if (0b00001000 == (~port_left & 0b00001000)) 
    {
      offset_pos[4] += pos[4] - 1.1;
      status |= true;
    }
  }
  
  prev_port_right = port_right;
  prev_port_left = port_left;

  return status;
}


CassiopeiaHW::CassiopeiaHW() 
{ 
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_right_shoulder("right_shoulder", &pos[0], &vel[0], &eff[0]);
  hardware_interface::JointStateHandle state_handle_right_elbow(   "right_elbow",    &pos[1], &vel[1], &eff[1]);

  hardware_interface::JointStateHandle state_handle_left_shoulder( "left_shoulder",  &pos[3], &vel[3], &eff[3]);
  hardware_interface::JointStateHandle state_handle_left_elbow(    "left_elbow",     &pos[4], &vel[4], &eff[4]);
  hardware_interface::JointStateHandle state_handle_reaction_wheel("reaction_wheel_joint", &pos[5], &vel[5], &eff[5]);

  jnt_state_interface.registerHandle(state_handle_left_shoulder);
  jnt_state_interface.registerHandle(state_handle_left_elbow);
  jnt_state_interface.registerHandle(state_handle_right_shoulder);
  jnt_state_interface.registerHandle(state_handle_right_elbow);
  jnt_state_interface.registerHandle(state_handle_reaction_wheel);

  registerInterface(&jnt_state_interface);

  // connect and register the joint effort interface
  hardware_interface::JointHandle effort_handle_right_shoulder(jnt_state_interface.getHandle("right_shoulder"),&cmd[0]);
  hardware_interface::JointHandle effort_handle_right_elbow(   jnt_state_interface.getHandle("right_elbow"),   &cmd[1]);

  hardware_interface::JointHandle effort_handle_left_shoulder( jnt_state_interface.getHandle("left_shoulder"), &cmd[3]);
  hardware_interface::JointHandle effort_handle_left_elbow(    jnt_state_interface.getHandle("left_elbow"),    &cmd[4]);
  hardware_interface::JointHandle effort_handle_reaction_wheel(jnt_state_interface.getHandle("reaction_wheel_joint"), &cmd[5]);

  jnt_eff_interface.registerHandle(effort_handle_left_shoulder);
  jnt_eff_interface.registerHandle(effort_handle_left_elbow);
  jnt_eff_interface.registerHandle(effort_handle_right_shoulder);
  jnt_eff_interface.registerHandle(effort_handle_right_elbow);
  jnt_eff_interface.registerHandle(effort_handle_reaction_wheel);

  registerInterface(&jnt_eff_interface);



  //ENCODER CARD


  // TRY ACCESS CARDS
  uint32_t  left_minor_number = 1;
  uint32_t right_minor_number = 0;

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

  encoder_1_old = encoder_init_value;
  encoder_2_old = encoder_init_value;
  encoder_3_old = encoder_init_value;
  encoder_4_old = encoder_init_value;
  encoder_5_old = encoder_init_value;
  encoder_6_old = encoder_init_value;

  for(int i=0; i<6; i++)
  {
    pos[i] = 0;
    prev_pos[i] = pos[i];
    offset_pos[i] = 0;
  }

  dac.init(640); //0x280 
  dac.configureIOPorts(OUTPUT,OUTPUT,OUTPUT);
  dac.setDACRange(20, BIPOLAR);
  
  for(int i=1; i<=8; i++) dac.updateDAC(i, 0.0);


}

