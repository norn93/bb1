#include "bb1_hw.h"
#include <iostream>
#include "controller_manager/controller_manager.h"
#include <hardware_interface/robot_hw.h>
#include "ros/callback_queue.h"

void controlLoop(ros::Time& last_time, 
    controller_manager::ControllerManager& cm, 
    bb1::BB1_HW& robot) {

  // Calculate monotonic time difference
  ros::Time this_time = ros::Time::now();
  //boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed = this_time - last_time;
  //last_time = this_time;
  ROS_WARN("%f", elapsed.toSec());
  last_time = this_time;
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bb1_robot");
  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string front_right_wheel_port;
  std::string back_right_wheel_port;
  std::string front_left_wheel_port;
  std::string back_left_wheel_port;
  double tacho_pulses_per_revolution;
  int motor_poles;

  ros::Rate r(100000); // 100Hz

  if (!private_nh.getParam("front_right_wheel_port", front_right_wheel_port)) {
    ROS_FATAL("VESC communication front_right_wheel_port parameter required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("back_right_wheel_port", back_right_wheel_port)) {
    ROS_FATAL("VESC communication back_right_wheel_port parameter required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("front_left_wheel_port", front_left_wheel_port)) {
    ROS_FATAL("VESC communication front_left_wheel_port parameter required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("back_left_wheel_port", back_left_wheel_port)) {
    ROS_FATAL("VESC communication beck_left_wheel_port parameter required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("tacho_pulses_per_revolution", tacho_pulses_per_revolution)) {
    ROS_FATAL("Parameter tacho_pulses_per_revolution required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("motor_poles", motor_poles)) {
    ROS_FATAL("Parameter motor_poles required.");
    ros::shutdown();
    return -1;
  }

  // decide whether or not we want to publish rotor positions
  // this is enabled with by sending a "set detect" packet with a mode
  // "encoder" is probably the most useful
  std::string rotor_position_source;
  disp_pos_mode mode = DISP_POS_MODE_NONE;
  if (private_nh.getParam("rotor_position_source", rotor_position_source)) {
    if (rotor_position_source.compare("inductance") == 0) {
      mode = DISP_POS_MODE_INDUCTANCE;
    }  else if (rotor_position_source.compare("observer") == 0) {
      mode = DISP_POS_MODE_OBSERVER;
    } else if (rotor_position_source.compare("encoder") == 0) {
      mode = DISP_POS_MODE_ENCODER;
    } else if (rotor_position_source.compare("pid_pos") == 0) {
      mode = DISP_POS_MODE_PID_POS;
    } else if (rotor_position_source.compare("pid_pos_error") == 0) {
      mode = DISP_POS_MODE_PID_POS_ERROR;
    } else if (rotor_position_source.compare("encoder_observer_error") == 0) {
      mode = DISP_POS_MODE_ENCODER_OBSERVER_ERROR;
    } else if (rotor_position_source.compare("none") != 0) {
      ROS_WARN("Invalid display mode parameter, defaulting to none");
    }
  }

  // Initialize robot hardware and link to controller manager
  bb1::BB1_HW robot(front_right_wheel_port, back_right_wheel_port, 
    front_left_wheel_port, back_left_wheel_port,
    tacho_pulses_per_revolution, motor_poles, mode, nh);
  controller_manager::ControllerManager cm(&robot);

  ros::Time last_time = ros::Time::now();
  ros::CallbackQueue bb1_queue;

  ros::Timer control_loop = nh.createTimer(ros::Duration(0.01), 
     boost::bind(controlLoop, last_time, boost::ref(cm), boost::ref(robot)));

  ROS_INFO_STREAM_NAMED("hardware_interface","Starting loop");

  //ros::spin();

  while (ros::ok())
  {
     robot.read();
     cm.update(ros::Time::now(), r.cycleTime());
     robot.write(r.cycleTime());
     r.sleep();
  }
  ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down");

  return 0;
}
