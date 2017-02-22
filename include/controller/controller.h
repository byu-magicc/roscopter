#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <fcu_common/Command.h>
#include <fcu_common/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <stdint.h>
#include <dynamic_reconfigure/server.h>
#include <ros_copter/ControllerConfig.h>

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

  double ax;
  double ay;
  double az;

  double throttle;
}state_t;

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double u;
  double v;
  double w;
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

  // Paramters
  double thrust_eq_;
  double mass_;
  double max_thrust_;
  double drag_constant_;
  bool is_flying_;

  // PID Controllers
  fcu_common::SimplePID PID_u_;
  fcu_common::SimplePID PID_v_;
  fcu_common::SimplePID PID_w_;
  fcu_common::SimplePID PID_x_;
  fcu_common::SimplePID PID_y_;
  fcu_common::SimplePID PID_z_;
  fcu_common::SimplePID PID_psi_;

  // Dynamic Reconfigure Hooks
  dynamic_reconfigure::Server<ros_copter::ControllerConfig> _server;
  dynamic_reconfigure::Server<ros_copter::ControllerConfig>::CallbackType _func;
  void reconfigure_callback(ros_copter::ControllerConfig &config, uint32_t level);

  // Memory for sharing information between functions
  state_t xhat_ = {}; // estimate
  max_t max_ = {};
  fcu_common::Command command_;
  state_t xc_ = {}; // command
  double prev_time_;
  uint8_t control_mode_;

  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const fcu_common::CommandConstPtr &msg);

  void computeControl(double dt);
  void resetIntegrators();
  void publishCommand();
  double saturate(double x, double max, double min);
  double sgn(double x);
};
}

#endif
