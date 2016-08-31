#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <fcu_common/ExtendedCommand.h>
#include <fcu_common/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

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
}state_t;

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
  ros::Subscriber goal_sub_;

  ros::Publisher command_pub_;

  // Paramters
  double thrust_eq_;
  bool is_flying_;

  // PID Controllers
  fcu_common::SimplePID PID_u_;
  fcu_common::SimplePID PID_v_;
  fcu_common::SimplePID PID_x_;
  fcu_common::SimplePID PID_y_;
  fcu_common::SimplePID PID_z_;
  fcu_common::SimplePID PID_psi_;

  // Memory for sharing information between functions
  state_t xhat_; // estimate
  fcu_common::ExtendedCommand command_;
  state_t xc_; // command
  double prev_time_;


  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void goalCallback(const geometry_msgs::Vector3ConstPtr &msg);

  void computeControl();
  void resetIntegrators();
  void publishCommand();
};
}

#endif
