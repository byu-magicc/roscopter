#include <controller/controller.h>

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_("~/controller")
{
  // retrieve params
  thrust_eq_= nh_private_.param<double>("equilibrium_thrust", 0.5);
  is_flying_ = false;

  // Set PID Gains
  double P, I, D, tau;
  tau = nh_private_.param<double>("tau", 0.05);
  P = nh_private_.param<double>("u/P", 1.0);
  I = nh_private_.param<double>("u/I", 1.0);
  D = nh_private_.param<double>("u/D", 1.0);
  PID_u_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("v/P", 1.0);
  I = nh_private_.param<double>("v/I", 1.0);
  D = nh_private_.param<double>("v/D", 1.0);
  PID_v_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("x/P", 1.0);
  I = nh_private_.param<double>("x/I", 1.0);
  D = nh_private_.param<double>("x/D", 1.0);
  PID_x_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("y/P", 1.0);
  I = nh_private_.param<double>("y/I", 1.0);
  D = nh_private_.param<double>("y/D", 1.0);
  PID_y_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("z/P", 1.0);
  I = nh_private_.param<double>("z/I", 1.0);
  D = nh_private_.param<double>("z/D", 1.0);
  PID_z_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("psi/P", 1.0);
  I = nh_private_.param<double>("psi/I", 1.0);
  D = nh_private_.param<double>("psi/D", 1.0);
  PID_psi_.setGains(P, I, D, tau);



  // Set up Publishers and Subscriber
  state_sub_ = nh_.subscribe("estimate", 1, &Controller::stateCallback, this);
  is_flying_sub_ = nh_.subscribe("is_flying", 1, &Controller::isFlyingCallback, this);
  goal_sub_ = nh_.subscribe("waypoint", 1, &Controller::goalCallback, this);

  command_pub_ = nh_.advertise<fcu_common::ExtendedCommand>("extended_command", 1);
}

void Controller::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // This should already be coming in NED
  xhat_.pn = msg->pose.pose.position.x;
  xhat_.pe = msg->pose.pose.position.y;
  xhat_.pd = msg->pose.pose.position.z;

  xhat_.u = msg->twist.twist.linear.x;
  xhat_.v = msg->twist.twist.linear.y;
  xhat_.w = msg->twist.twist.linear.z;

  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(xhat_.phi, xhat_.theta, xhat_.psi);

  xhat_.p = msg->twist.twist.angular.x;
  xhat_.q = msg->twist.twist.angular.y;
  xhat_.r = msg->twist.twist.angular.z;

  if(is_flying_)
  {
    computeControl();
    publishCommand();
  }
  else
  {
    resetIntegrators();
    prev_time_ = ros::Time::now().toSec();
  }
}

void Controller::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
{
  is_flying_ = msg->data;
}

void Controller::goalCallback(const geometry_msgs::Vector3ConstPtr &msg)
{
  xc_.pn = msg->x;
  xc_.pe = msg->y;
  xc_.pd = msg->z;
}


void Controller::computeControl()
{
  double now = ros::Time::now().toSec();
  double dt = now - prev_time_;
  prev_time_ = now;

  // Figure out desired velocities (in inertial frame)
  // By running the position controllers
  double pndot_c = PID_x_.computePID(xc_.pn, xhat_.pn, dt);
  double pedot_c = PID_y_.computePID(xc_.pe, xhat_.pe, dt);
  double pddot_c = PID_z_.computePID(xc_.pd, xhat_.pd, dt);

  // Rotate into body frame
  /// TODO: Include pitch and roll in this mapping
  double u_c = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);
  double v_c = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);

  double phi_c = PID_u_.computePID(u_c, xhat_.u, dt);
  double theta_c = PID_u_.computePID(v_c, xhat_.v, dt);

  // For now, just command yaw to be in the direction of travel
  double psi_c = atan2(xc_.pn - xhat_.pn, xc_.pe - xhat_.pe);
  double r_c = PID_psi_.computePID(psi_c, xhat_.psi, dt);

  // Save message for publishing
  // Be sure to add the feed-forward term for thrust
  command_.mode = fcu_common::ExtendedCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  command_.F = pddot_c + thrust_eq_/(cos(phi_c)*cos(theta_c));
  command_.x = phi_c;
  command_.y = theta_c;
  command_.z = r_c;
}

void Controller::publishCommand()
{
  command_pub_.publish(command_);
}

void Controller::resetIntegrators()
{
  PID_u_.clearIntegrator();
  PID_v_.clearIntegrator();
  PID_x_.clearIntegrator();
  PID_y_.clearIntegrator();
  PID_z_.clearIntegrator();
  PID_psi_.clearIntegrator();
}

} // namespace controller

