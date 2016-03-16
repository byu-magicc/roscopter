#ifndef RC_JOY_H
#define RC_JOY_H

#include <fcu_common/Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

struct Axes {
  int roll;
  int pitch;
  int thrust;
  int yaw;
  int roll_direction;
  int pitch_direction;
  int thrust_direction;
  int yaw_direction;
};

struct Button{
  int index;
  bool prev_value;
};

struct Buttons {
  Button fly;
};

namespace ros_copter{
class RCJoy {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

private:
  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;
  std::string command_topic_;

  Axes axes_;

  bool fly_mav_;

  fcu_common::Command command_msg_;
  sensor_msgs::Joy current_joy_;

  Buttons buttons_;

  double thrust_to_mass_ratio_;

  void StopMav();

  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void Publish();

public:
  RCJoy();
};
}

#endif // RC_JOY_H
