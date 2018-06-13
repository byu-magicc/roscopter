#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>
#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <roscopter/ControllerConfig.h>
#include <eigen3/Eigen/Eigen>
#include <roscopter_common/common.h>

namespace controller
{

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;

  double throttle;
} state_t;

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double vel;
} max_t;

class Controller
{

public:

  Controller();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber is_flying_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher command_pub_;

  // Parameters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  double drag_constant_;
  bool is_flying_;

  // Controller Gains
  Eigen::Matrix3d K_p_; // position
  Eigen::Matrix3d K_v_; // velocity
  Eigen::Matrix3d K_d_; // disturbance acceleration

  // Dynamic Reconfigure Hooks
  dynamic_reconfigure::Server<roscopter::ControllerConfig> _server;
  dynamic_reconfigure::Server<roscopter::ControllerConfig>::CallbackType _func;
  void reconfigure_callback(roscopter::ControllerConfig &config, uint32_t level);

  // Memory for sharing information between functions
  state_t xhat_ = {}; // estimate
  state_t xc_ = {}; // command
  max_t max_ = {};
  rosflight_msgs::Command command_;
  double prev_time_;
  uint8_t control_mode_;
  Eigen::Vector3d dhat_; // disturbance acceleration

  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);
  void computeControl(double dt);
};
}

#endif
