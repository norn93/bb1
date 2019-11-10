#ifndef BB1_HW_H
#define BB1_HW_H
#include "bb1_hw.h"
#include "wheel_driver.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <control_toolbox/pid.h>

namespace bb1
{

class BB1_HW : public hardware_interface::RobotHW
{
public:
  BB1_HW(std::string front_right_wheel_port, std::string back_right_wheel_port, 
    std::string front_left_wheel_port, std::string back_left_wheel_port,
    double tacho_pulses_per_revolution, int motor_poles, disp_pos_mode rotor_position_source, ros::NodeHandle nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  wheel_driver _front_right_wheel_driver;
  wheel_driver _back_right_wheel_driver;
  wheel_driver _front_left_wheel_driver;
  wheel_driver _back_left_wheel_driver;
  double _tacho_pulses_per_revolution;
  double _tacho_conversion_factor;
  double _rad_per_sec_to_erpm_conversion_factor;
  int _motor_poles;
  int _maxSpeed;
  hardware_interface::JointStateInterface _jnt_state_interface;
  hardware_interface::VelocityJointInterface _jnt_vel_interface;
  double _cmd[4];
  double _pos[4];
  double _vel[4];
  double _eff[4];

  control_toolbox::Pid _front_left_pid_controller;
  control_toolbox::Pid _back_left_pid_controller;
  control_toolbox::Pid _front_right_pid_controller;
  control_toolbox::Pid _back_right_pid_controller;
  
  ros::Time _front_left_last_time;
  ros::Time _back_left_last_time;
  ros::Time _front_right_last_time;
  ros::Time _back_right_last_time;

  double _front_left_wheel_low_pass_speed;
  double _back_left_wheel_low_pass_speed;
  double _front_right_wheel_low_pass_speed;
  double _back_right_wheel_low_pass_speed;

  double alpha;

  double pid_p;
  double pid_i;
  double pid_d;
};

}

#endif // BB1_HW_H
