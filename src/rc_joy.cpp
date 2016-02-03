#include "rc_joy/rc_joy.h"

// #include <rotor_gazebo/default_topics.h>


namespace ros_copter {
  RCJoy::RCJoy(){
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("command_topic", command_topic_, "command");

  pnh.param<int>("axis_roll_", axes_.roll, 2);
  pnh.param<int>("axis_pitch_", axes_.pitch, 3);
  pnh.param<int>("axis_thrust_", axes_.thrust, 1);
  pnh.param<int>("axis_yaw_", axes_.yaw, 0);

  pnh.param<int>("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param<int>("axis_direction_pitch", axes_.pitch_direction, -1);
  pnh.param<int>("axis_direction_thrust", axes_.thrust_direction, 1);
  pnh.param<int>("axis_direction_yaw", axes_.yaw_direction, -1);

  pnh.param<double>("thrust_to_mass_ratio", thrust_to_mass_ratio_, 3.81);  // [N]

  pnh.param<int>("button_takeoff", buttons_.fly.index, 0);

  command_pub_ = nh_.advertise<fcu_io::Command>(command_topic_,10);

  ROS_ERROR_STREAM("thrust to mass ratio" << thrust_to_mass_ratio_);

  command_msg_.normalized_roll = 0;
  command_msg_.normalized_pitch = 0;
  command_msg_.normalized_yaw = 0;
  command_msg_.normalized_throttle = 0;

  joy_sub_ = nh_.subscribe("joy", 10, &RCJoy::JoyCallback, this);
  fly_mav_ = false;
}

void RCJoy::StopMav() {
  command_msg_.normalized_roll = 0;
  command_msg_.normalized_pitch = 0;
  command_msg_.normalized_yaw = 0;
  command_msg_.normalized_throttle = 0;
}

void RCJoy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  if(fly_mav_){
    current_joy_ = *msg;
    command_msg_.normalized_roll = msg->axes[axes_.roll] * axes_.roll_direction;
    command_msg_.normalized_pitch = msg->axes[axes_.pitch] * axes_.pitch_direction;
    command_msg_.normalized_yaw = msg->axes[axes_.yaw] * axes_.yaw_direction;
    if(msg->axes[axes_.thrust]*axes_.thrust_direction < 0){
      // some fraction of the mass
      command_msg_.normalized_throttle = (msg->axes[axes_.thrust]+1)*1.0/thrust_to_mass_ratio_;
    }else{
      // some fraction of remaining thrust
      command_msg_.normalized_throttle = msg->axes[axes_.thrust]*(1.0-(1.0/thrust_to_mass_ratio_))+1.0/thrust_to_mass_ratio_;
    }
  }else{
    StopMav();
  }
  if(msg->buttons[buttons_.fly.index]==0 && buttons_.fly.prev_value==1){ // button release
    fly_mav_ = !fly_mav_;
  }
  buttons_.fly.prev_value = msg->buttons[buttons_.fly.index];

  ros::Time update_time = ros::Time::now();
  Publish();
}

void RCJoy::Publish() {
  command_pub_.publish(command_msg_);
}

} // end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotor_sim_joy");
  ros_copter::RCJoy joy;

  ros::spin();

  return 0;
}
