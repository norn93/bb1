#include "bb1_hw.h"
#include "wheel_driver.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <math.h>

namespace bb1 {

BB1_HW::BB1_HW(std::string front_right_wheel_port, std::string back_right_wheel_port, 
    std::string front_left_wheel_port, std::string back_left_wheel_port,
    double tacho_pulses_per_revolution, int motor_poles, disp_pos_mode rotor_position_source, ros::NodeHandle nh) :
  _front_right_wheel_driver(front_right_wheel_port, nh, "front_right_wheel", rotor_position_source),
  _back_right_wheel_driver(back_right_wheel_port, nh, "back_right_wheel", rotor_position_source),
  _front_left_wheel_driver(front_left_wheel_port, nh, "front_left_wheel", rotor_position_source),
  _back_left_wheel_driver(back_left_wheel_port, nh, "back_left_wheel", rotor_position_source)
 {
    _pos[0]=0;
    _pos[1]=0;
    _pos[2]=0;
    _pos[3]=0;
    _vel[0]=0;
    _vel[1]=0;
    _vel[2]=0;
    _vel[3]=0;
    _cmd[0]=0;
    _cmd[1]=0;
    _cmd[2]=0;
    _cmd[3]=0;
    _eff[0]=0;
    _eff[1]=0;
    _eff[2]=0;
    _eff[3]=0;
    _tacho_pulses_per_revolution = tacho_pulses_per_revolution;
    _motor_poles = motor_poles;
    // Convert rad/s to RPM and multiply by number of poles to get ERPM
    _rad_per_sec_to_erpm_conversion_factor = (60/(2*M_PI))*_motor_poles;
    _tacho_conversion_factor = (2*M_PI)/_tacho_pulses_per_revolution;

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("front_left_wheel_joint", &_pos[0], &_vel[0], &_eff[0]);
    _jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("back_left_wheel_joint", &_pos[1], &_vel[1], &_eff[1]);
    _jnt_state_interface.registerHandle(state_handle_b);

    hardware_interface::JointStateHandle state_handle_c("front_right_wheel_joint", &_pos[2], &_vel[2], &_eff[2]);
    _jnt_state_interface.registerHandle(state_handle_c);

    hardware_interface::JointStateHandle state_handle_d("back_right_wheel_joint", &_pos[3], &_vel[3], &_eff[3]);
    _jnt_state_interface.registerHandle(state_handle_d);

    registerInterface(&_jnt_state_interface);

    // connect and register the joint velocity interfaces
    hardware_interface::JointHandle pos_handle_a(_jnt_state_interface.getHandle("front_left_wheel_joint"), &_cmd[0]);
    _jnt_vel_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(_jnt_state_interface.getHandle("back_left_wheel_joint"), &_cmd[1]);
    _jnt_vel_interface.registerHandle(pos_handle_b);

    hardware_interface::JointHandle pos_handle_c(_jnt_state_interface.getHandle("front_right_wheel_joint"), &_cmd[2]);
    _jnt_vel_interface.registerHandle(pos_handle_c);

    hardware_interface::JointHandle pos_handle_d(_jnt_state_interface.getHandle("back_right_wheel_joint"), &_cmd[3]);
    _jnt_vel_interface.registerHandle(pos_handle_d);

    registerInterface(&_jnt_vel_interface);

    //control_toolbox::Pid _front_left_pid_controller;
    //p, i, d, i_max, i_min (the max and min reduce integral windup)
    //_front_left_pid_controller.initPid(1, 1, 1, 0.3, -0.3); //commented out because I don't think it's used
    //set up the time
    //ros::Time _front_left_last_time = ros::Time::now();

    //_front_left_wheel_low_pass_speed = 0;

    //Parameter loading
    //alpha
    if(nh.getParam("bb1_motor_pid/alpha", alpha)) {
      ROS_INFO("Loaded 'bb1_motor_pid/alpha' OK!");
    } else {
      ROS_FATAL("Unable to load 'bb1_motor_pid/alpha'...");
    }

    //p
    if(nh.getParam("bb1_motor_pid/p", pid_p)) {
      ROS_INFO("Loaded 'bb1_motor_pid/p' OK!");
    } else {
      ROS_FATAL("Unable to load 'bb1_motor_pid/p'...");
    }

    //i
    if(nh.getParam("bb1_motor_pid/i", pid_i)) {
      ROS_INFO("Loaded 'bb1_motor_pid/i' OK!");
    } else {
      ROS_FATAL("Unable to load 'bb1_motor_pid/i'...");
    }

    //p
    if(nh.getParam("bb1_motor_pid/d", pid_d)) {
      ROS_INFO("Loaded 'bb1_motor_pid/d' OK!");
    } else {
      ROS_FATAL("Unable to load 'bb1_motor_pid/d'...");
    }

  }

  void BB1_HW::read(const ros::Time& time, const ros::Duration& period)
  {
    ROS_DEBUG("Reading from hardware...");

    _pos[0] = _front_left_wheel_driver.getDisplacement()*_tacho_conversion_factor;
    _pos[1] = _back_left_wheel_driver.getDisplacement()*_tacho_conversion_factor;
    _pos[2] = -_front_right_wheel_driver.getDisplacement()*_tacho_conversion_factor;
    _pos[3] = -_back_right_wheel_driver.getDisplacement()*_tacho_conversion_factor;
    _vel[0] = _front_left_wheel_driver.getSpeed()/_rad_per_sec_to_erpm_conversion_factor;
    _vel[1] = _back_left_wheel_driver.getSpeed()/_rad_per_sec_to_erpm_conversion_factor;
    _vel[2] = -_front_right_wheel_driver.getSpeed()/_rad_per_sec_to_erpm_conversion_factor;
    _vel[3] = -_back_right_wheel_driver.getSpeed()/_rad_per_sec_to_erpm_conversion_factor;

    _eff[0] = _front_left_wheel_driver.getDutyCycleIn();
    _eff[1] = _back_left_wheel_driver.getDutyCycleIn();
    _eff[2] = -_front_right_wheel_driver.getDutyCycleIn();
    _eff[3] = -_back_right_wheel_driver.getDutyCycleIn();

    double encoderDisplacementFrontLeft = _front_left_wheel_driver.getEncoderDisplacement();
    double encoderDisplacementBackLeft = _back_left_wheel_driver.getEncoderDisplacement();
    double encoderDisplacementFrontRight = _front_right_wheel_driver.getEncoderDisplacement();
    double encoderDisplacementBackRight = _back_right_wheel_driver.getEncoderDisplacement();

    ROS_DEBUG("Vars: %f : %f", _rad_per_sec_to_erpm_conversion_factor, _tacho_conversion_factor);

    ROS_DEBUG("Position: %f : %f : %f : %f", _pos[0], _pos[1], _pos[2], _pos[3]);
    ROS_DEBUG("Encoder position: %f : %f : %f : %f", encoderDisplacementFrontLeft, encoderDisplacementBackLeft, 
      encoderDisplacementFrontRight, encoderDisplacementBackRight);
    ROS_DEBUG("Velocity: %f : %f : %f : %f", _vel[0], _vel[1], _vel[2], _vel[3]);
    ROS_DEBUG("Commands: %f : %f : %f : %f", _cmd[0], _cmd[1], _cmd[2], _cmd[3]);
    ROS_DEBUG("Effort: %f : %f : %f : %f", _eff[0], _eff[1], _eff[2], _eff[3]);
  }

  void BB1_HW::write(const ros::Time& time, const ros::Duration& period)
  {
    /////////// front_left ///////////

    ROS_DEBUG("Entering front_left wheel control...");

    // Set the PID gains according to the parameters
    _front_left_pid_controller.setGains(pid_p, pid_i, pid_d, 10000, -10000);

    // Calculate filtered velocity
    _front_left_wheel_low_pass_speed = alpha * _vel[0] + (1 - alpha) * _front_left_wheel_low_pass_speed;
    ROS_INFO("Low pass velocity: %f", _front_left_wheel_low_pass_speed);

    ROS_INFO("Desired speed: %f, error: %f", _cmd[0], _cmd[0] - _front_left_wheel_low_pass_speed);

    // Compute the effort
    double front_left_effort = _front_left_pid_controller.computeCommand(_cmd[0] - _front_left_wheel_low_pass_speed, ros::Time::now() - _front_left_last_time);
    // Reset the time
    _front_left_last_time = ros::Time::now();
    ROS_INFO("Output: %f", front_left_effort);

    // Scaling for voltage
    double front_left_voltage_in = _front_left_wheel_driver.getVoltageIn();
    front_left_effort = front_left_effort/front_left_voltage_in;
    ROS_INFO("Adjusted output: %f", front_left_effort);

    // Clamping output
    if (front_left_effort > 0.95) {
      front_left_effort = 0.95;
    } else if (front_left_effort < -0.95) {
      front_left_effort = -0.95;
    }
    ROS_INFO("Clamped output: %f", front_left_effort);

    // Write to the wheel
    _front_left_wheel_driver.setDutyCycle(front_left_effort);

    ROS_DEBUG("Leaving front_left wheel control...");

    /////////// back_left ///////////

    ROS_DEBUG("Entering back_left wheel control...");

    // Set the PID gains according to the parameters
    _back_left_pid_controller.setGains(pid_p, pid_i, pid_d, 10000, -10000);

    // Calculate filtered velocity
    _back_left_wheel_low_pass_speed = alpha * _vel[0] + (1 - alpha) * _back_left_wheel_low_pass_speed;
    ROS_INFO("Low pass velocity: %f", _back_left_wheel_low_pass_speed);

    ROS_INFO("Desired speed: %f, error: %f", _cmd[0], _cmd[0] - _back_left_wheel_low_pass_speed);

    // Compute the effort
    double back_left_effort = _back_left_pid_controller.computeCommand(_cmd[0] - _back_left_wheel_low_pass_speed, ros::Time::now() - _back_left_last_time);
    // Reset the time
    _back_left_last_time = ros::Time::now();
    ROS_INFO("Output: %f", back_left_effort);

    // Scaling for voltage
    double back_left_voltage_in = _back_left_wheel_driver.getVoltageIn();
    back_left_effort = back_left_effort/back_left_voltage_in;
    ROS_INFO("Adjusted output: %f", back_left_effort);

    // Clamping output
    if (back_left_effort > 0.95) {
      back_left_effort = 0.95;
    } else if (back_left_effort < -0.95) {
      back_left_effort = -0.95;
    }
    ROS_INFO("Clamped output: %f", back_left_effort);

    // Write to the wheel
    _back_left_wheel_driver.setDutyCycle(back_left_effort);

    ROS_DEBUG("Leaving back_left wheel control...");

    /////////// front_right ///////////

    ROS_DEBUG("Entering front_right wheel control...");

    // Set the PID gains according to the parameters
    _front_right_pid_controller.setGains(pid_p, pid_i, pid_d, 10000, -10000);

    // Calculate filtered velocity
    _front_right_wheel_low_pass_speed = alpha * _vel[0] + (1 - alpha) * _front_right_wheel_low_pass_speed;
    ROS_INFO("Low pass velocity: %f", _front_right_wheel_low_pass_speed);

    ROS_INFO("Desired speed: %f, error: %f", _cmd[0], _cmd[0] - _front_right_wheel_low_pass_speed);

    // Compute the effort
    double front_right_effort = _front_right_pid_controller.computeCommand(_cmd[0] - _front_right_wheel_low_pass_speed, ros::Time::now() - _front_right_last_time);
    // Reset the time
    _front_right_last_time = ros::Time::now();
    ROS_INFO("Output: %f", front_right_effort);

    // Scaling for voltage
    double front_right_voltage_in = _front_right_wheel_driver.getVoltageIn();
    front_right_effort = front_right_effort/front_right_voltage_in;
    ROS_INFO("Adjusted output: %f", front_right_effort);

    // Clamping output
    if (front_right_effort > 0.95) {
      front_right_effort = 0.95;
    } else if (front_right_effort < -0.95) {
      front_right_effort = -0.95;
    }
    ROS_INFO("Clamped output: %f", front_right_effort);

    // Write to the wheel
    _front_right_wheel_driver.setDutyCycle(front_right_effort);

    ROS_DEBUG("Leaving front_right wheel control...");

    /////////// back_right ///////////

    ROS_DEBUG("Entering back_right wheel control...");

    // Set the PID gains according to the parameters
    _back_right_pid_controller.setGains(pid_p, pid_i, pid_d, 10000, -10000);

    // Calculate filtered velocity
    _back_right_wheel_low_pass_speed = alpha * _vel[0] + (1 - alpha) * _back_right_wheel_low_pass_speed;
    ROS_INFO("Low pass velocity: %f", _back_right_wheel_low_pass_speed);

    ROS_INFO("Desired speed: %f, error: %f", _cmd[0], _cmd[0] - _back_right_wheel_low_pass_speed);

    // Compute the effort
    double back_right_effort = _back_right_pid_controller.computeCommand(_cmd[0] - _back_right_wheel_low_pass_speed, ros::Time::now() - _back_right_last_time);
    // Reset the time
    _back_right_last_time = ros::Time::now();
    ROS_INFO("Output: %f", back_right_effort);

    // Scaling for voltage
    double back_right_voltage_in = _back_right_wheel_driver.getVoltageIn();
    back_right_effort = back_right_effort/back_right_voltage_in;
    ROS_INFO("Adjusted output: %f", back_right_effort);

    // Clamping output
    if (back_right_effort > 0.95) {
      back_right_effort = 0.95;
    } else if (back_right_effort < -0.95) {
      back_right_effort = -0.95;
    }
    ROS_INFO("Clamped output: %f", back_right_effort);

    // Write to the wheel
    _back_right_wheel_driver.setDutyCycle(back_right_effort);

    ROS_DEBUG("Leaving back_right wheel control...");
  }
}
