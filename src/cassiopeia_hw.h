#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dm6814_library.h>
#include "dm6604.h"

class CassiopeiaHW : public hardware_interface::RobotHW
{
public:
  void writeMotors();
  void readEncoders(ros::Duration);
  bool checkLimits();
  CassiopeiaHW(); 

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  double cmd[6];
  double pos[6], prev_pos[6], offset_pos[6];
  double vel[6];
  double eff[6];

  //CARDS
  DM6604Device dac;
  DM6814Device dm6814_right; //that was static
  DM6814Device dm6814_left;

  //ENCODER VALUES FROM CARD
  uint16_t encoder_1_val;
  uint16_t encoder_2_val;
  uint16_t encoder_3_val;
  uint16_t encoder_4_val;
  uint16_t encoder_5_val;
  uint16_t encoder_6_val;

  //ENCODER VALUES HISTORY FROM CARD
  uint16_t encoder_1_old;
  uint16_t encoder_2_old;
  uint16_t encoder_3_old;
  uint16_t encoder_4_old;
  uint16_t encoder_5_old;
  uint16_t encoder_6_old;

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


};