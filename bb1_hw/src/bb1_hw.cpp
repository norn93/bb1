#include "bb1_hw.h"
#include "wheel_driver.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <math.h>

//#include <control_toolbox/pid.h>

namespace bb1 {

BB1_HW::BB1_HW(std::string front_right_wheel_port, std::string back_right_wheel_port, 
    std::string front_left_wheel_port, std::string back_left_wheel_port,
    double front_right_wheel_ikv, double back_right_wheel_ikv,
    double front_left_wheel_ikv, double back_left_wheel_ikv,
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
    _front_right_wheel_ikv = front_right_wheel_ikv;
    _back_right_wheel_ikv = back_right_wheel_ikv;
    _front_left_wheel_ikv = front_left_wheel_ikv;
    _back_left_wheel_ikv = back_left_wheel_ikv;
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

    control_toolbox::Pid test_pid_controller;
    //p, i, d, i_max, i_min (the max and min reduce integral windup)
    test_pid_controller.initPid(1, 1, 1, 0.3, -0.3);
    //reset
    //test_pid_controller.reset();
    //set up the time
    ros::Time last_time = ros::Time::now();

    _front_left_wheel_low_pass_speed = 0;

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

    _pos[0] = _front_left_wheel_driver.getDisplacement()/90*(2*3.14);//*_tacho_conversion_factor;
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
    ROS_DEBUG("Writing to hardware...");

    // Get current parameters: can't because 'nh' isn't in scope
    // nh.getParam("bb1_motor_pid/alpha", alpha);
    // nh.getParam("bb1_motor_pid/p", pid_p);
    // nh.getParam("bb1_motor_pid/i", pid_i);
    // nh.getParam("bb1_motor_pid/d", pid_d);

    ROS_INFO("Entering PID control...");

    test_pid_controller.setGains(pid_p, pid_i, pid_d, 10000, -10000);

    //ROS_INFO("Last time %d", last_time);

    _front_left_wheel_low_pass_speed = alpha * _vel[0] + (1 - alpha) * _front_left_wheel_low_pass_speed;
    ROS_INFO("Low pass speed: %f", _front_left_wheel_low_pass_speed);

    //_vel[0] = _front_left_wheel_low_pass_speed;

    ROS_INFO("Current speed: %f, desired speed: %f, error: %f", _front_left_wheel_low_pass_speed, _cmd[0], _cmd[0] - _front_left_wheel_low_pass_speed);
    //ROS_INFO("Time: %d, Last: %d, Since update: %d", ros::Time::now().nsec, last_time.nsec, ros::Time::now().nsec - last_time.nsec);

    //ros::Duration dt = t_now - pid_last_time_;

    double effort = test_pid_controller.computeCommand(_cmd[0] - _front_left_wheel_low_pass_speed, ros::Time::now() - last_time);
    //double effort = test_pid_controller.computeCommand(_cmd[0] - _vel[0], ros::Duration(0.10));
    last_time = ros::Time::now();

    ROS_INFO("Output: %f", effort);

    double front_left_voltage_in = _front_left_wheel_driver.getVoltageIn();
    effort = effort/front_left_voltage_in;

    ROS_INFO("Adjusted output: %f", effort);

    if (effort > 0.95) {
      effort = 0.95;
    } else if (effort < -0.95) {
      effort = -0.95;
    } //else if (effort < 0.01 && effort >= 0 || effort > -0.01 && effort <= 0) {
      //effort = 0;
      //test_pid_controller.reset();
    //}

    ROS_INFO("Clamped output: %f", effort);

    //const double command = test_pid_controller.getCurrentCmd();

    //ROS_INFO("Current: %f", command);

    //TODO: _front_left_wheel_driver.setDutyCycle(effort);

    _front_left_wheel_driver.setDutyCycle(effort);

    double a = 0;
    double b = 0;
    double c = 0;

    test_pid_controller.getCurrentPIDErrors(&a, &b, &c);

    ROS_INFO("%f %f %f", a, b, c);

    double p = 0;
    double i = 0;
    double d = 0;
    double i_max = 0;
    double i_min = 0;

    test_pid_controller.getGains(p, i, d, i_max, i_min);

    ROS_INFO("%f %f %f %f %f", p, i, d, i_max, i_min);

    ROS_INFO("Exiting PID control.");
    
    //double front_left_voltage_in = _front_left_wheel_driver.getVoltageIn();
    double front_left_request_dutyCycle = 0.0;

    // if (_cmd[0] != 0.0)
    // {
    //   double requestedERPM = _rad_per_sec_to_erpm_conversion_factor * _cmd[0];
    //   ROS_DEBUG("Requested ERPM front left: %f", requestedERPM);
    //   front_left_request_dutyCycle = requestedERPM / (front_left_voltage_in * _front_left_wheel_ikv * _motor_poles * 2);
    //   ROS_DEBUG("Front left request dutycycle : %f", front_left_request_dutyCycle);
    //    //_front_left_wheel_driver.setDutyCycle(front_left_request_dutyCycle);
    // }
    // else
    // {
    //    _front_left_wheel_driver.releaseMotor();
    // }

    double back_left_voltage_in = _back_left_wheel_driver.getVoltageIn();
    double back_left_request_dutyCycle = 0.0;

    if (_cmd[1] != 0.0)
    {
      double requestedERPM = _rad_per_sec_to_erpm_conversion_factor * _cmd[1];
      ROS_DEBUG("Requested ERPM back left: %f", requestedERPM);
      back_left_request_dutyCycle = requestedERPM / (back_left_voltage_in * _back_left_wheel_ikv * _motor_poles * 2);
      ROS_DEBUG("Back left request dutycycle : %f", back_left_request_dutyCycle);
       _back_left_wheel_driver.setDutyCycle(back_left_request_dutyCycle);
    }
    else
    {
      _back_left_wheel_driver.releaseMotor();
    }

    double front_right_voltage_in = _front_right_wheel_driver.getVoltageIn();
    double front_right_request_dutyCycle = 0.0;
    if (_cmd[2] != 0.0)
    {
      double requestedERPM = -_rad_per_sec_to_erpm_conversion_factor * _cmd[2];
      ROS_DEBUG("Requested ERPM front right: %f", requestedERPM);
      front_right_request_dutyCycle = requestedERPM / (front_right_voltage_in * _front_right_wheel_ikv * _motor_poles * 2);
      ROS_DEBUG("Front right request dutycycle : %f", front_right_request_dutyCycle);
      //_front_right_wheel_driver.setDutyCycle(front_right_request_dutyCycle);
    }
    else
    {
      _front_right_wheel_driver.releaseMotor();
    }

    double back_right_voltage_in = _back_right_wheel_driver.getVoltageIn();
    double back_right_request_dutyCycle = 0.0;
    if (_cmd[3] != 0.0)
    {
      double requestedERPM = -_rad_per_sec_to_erpm_conversion_factor * _cmd[3];
      ROS_DEBUG("Requested ERPM back right: %f", requestedERPM);
      back_right_request_dutyCycle = requestedERPM / (back_right_voltage_in * _back_right_wheel_ikv * _motor_poles * 2);
      ROS_DEBUG("Back right request dutycycle : %f", back_right_request_dutyCycle);
      //_back_right_wheel_driver.setDutyCycle(back_right_request_dutyCycle);
    }
    else
    {
      _back_right_wheel_driver.releaseMotor();
    }
  }
}
