#include <controller/controller.h>
#include <stdio.h>

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{
  // retrieve params
  thrust_eq_= nh_private_.param<double>("equilibrium_thrust", 0.34);
  is_flying_ = false;

  // Set PID Gains
  double P, I, D, tau;
  tau = nh_private_.param<double>("tau", 0.05);
  P = nh_private_.param<double>("u_P", 1.0);
  I = nh_private_.param<double>("u_I", 1.0);
  D = nh_private_.param<double>("u_D", 1.0);
  PID_u_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("v_P", 1.0);
  I = nh_private_.param<double>("v_I", 1.0);
  D = nh_private_.param<double>("v_D", 1.0);
  PID_v_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("x_P", 1.0);
  I = nh_private_.param<double>("x_I", 1.0);
  D = nh_private_.param<double>("x_D", 1.0);
  PID_x_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("y_P", 1.0);
  I = nh_private_.param<double>("y_I", 1.0);
  D = nh_private_.param<double>("y_D", 1.0);
  PID_y_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("z_P", 1.0);
  I = nh_private_.param<double>("z_I", 1.0);
  D = nh_private_.param<double>("z_D", 1.0);
  PID_z_.setGains(P, I, D, tau);

  P = nh_private_.param<double>("psi_P", 1.0);
  I = nh_private_.param<double>("psi_I", 1.0);
  D = nh_private_.param<double>("psi_D", 1.0);
  PID_psi_.setGains(P, I, D, tau);

  max_.roll = nh_private_.param<double>("max_roll", 0.785);
  max_.pitch = nh_private_.param<double>("max_pitch", 0.785);
  max_.yaw_rate = nh_private_.param<double>("max_yaw_rate", 3.14159);
  max_.throttle = nh_private_.param<double>("max_throttle", 1.0);
  max_.u = nh_private_.param<double>("max_u", 1.0);
  max_.v = nh_private_.param<double>("max_v", 1.0);

  _func = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

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
  xhat_.theta = xhat_.theta;
  xhat_.psi = xhat_.psi;

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

void Controller::reconfigure_callback(ros_copter::ControllerConfig &config, uint32_t level)
{
  double P, I, D, tau;
  tau = config.tau;
  P = config.u_P;
  I = config.u_I;
  D = config.u_D;
  PID_u_.setGains(P, I, D, tau);

  P = config.v_P;
  I = config.v_I;
  D = config.v_D;
  PID_v_.setGains(P, I, D, tau);

  P = config.x_P;
  I = config.x_I;
  D = config.x_D;
  PID_x_.setGains(P, I, D, tau);

  P = config.y_P;
  I = config.y_I;
  D = config.y_D;
  PID_y_.setGains(P, I, D, tau);

  P = config.z_P;
  I = config.z_I;
  D = config.z_D;
  PID_z_.setGains(P, I, D, tau);

  P = config.psi_P;
  I = config.psi_I;
  D = config.psi_D;
  PID_psi_.setGains(P, I, D, tau);

  max_.roll = config.max_roll;
  max_.pitch = config.max_pitch;
  max_.yaw_rate = config.max_yaw_rate;
  max_.throttle = config.max_throttle;
  max_.u = config.max_u;
  max_.v = config.max_v;
  ROS_INFO("new gains");

  resetIntegrators();
}


void Controller::computeControl()
{
  double now = ros::Time::now().toSec();
  double dt = now - prev_time_;
  prev_time_ = now;

  if(dt > 0.0)
  {
    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_x_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_y_.computePID(xc_.pe, xhat_.pe, dt);
    double az_c = saturate(PID_z_.computePID(xc_.pd, xhat_.pd, dt), 1.0, -1.0);

    // Rotate into body frame
    /// TODO: Include pitch and roll in this mapping
    double u_c = saturate(pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi), max_.u, -1.0*max_.u);
    double v_c = saturate(-pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi), max_.v, -1.0*max_.v);

//    double u_c = xc_.pn;
//    double v_c = xc_.pe;

    double ax_c = saturate(PID_u_.computePID(u_c, xhat_.u, dt), max_.roll, -max_.roll);
    double ay_c = saturate(PID_v_.computePID(v_c, xhat_.v, dt), max_.pitch, -max_.pitch);

    // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
    double total_acc_c = sqrt((1.0-az_c)*(1.0-az_c) + ax_c*ax_c + ay_c*ay_c); // (in g's)
    double thrust = total_acc_c*thrust_eq_; // calculate the total thrust in normalized units
    double phi_c = asin(ay_c / total_acc_c);
    double theta_c = -1.0*asin(ax_c / total_acc_c);

//    std::printf("z_c = %0.02f\t, z = %0.02f\t, az_c = %0.02f\t, total = %0.02f\t, thrust = %0.02f\n",
//                xc_.pd, xhat_.pd, az_c, total_acc_c, thrust);
//    std::printf("u_c: = %0.02f,\t u = %0.02f,\t out = %0.04f,\t theta_c = %0.04f, dt = %0.04f\n", u_c, xhat_.u, ax_c, theta_c, dt);

    // For now, just command yaw to be in the direction of travel
    double psi_c = atan2(xc_.pn - xhat_.pn, xc_.pe - xhat_.pe);
    double r_c = saturate(PID_psi_.computePID(psi_c, xhat_.psi, dt), max_.yaw_rate, -max_.yaw_rate);

    // Save message for publishing
    // Be sure to add the feed-forward term for thrust
    command_.mode = fcu_common::ExtendedCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.F = saturate(thrust, max_.throttle, 0.0);
    command_.x = phi_c;
    command_.y = theta_c;
    command_.z = r_c;
  }

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

double Controller::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

double Controller::sgn(double x)
{
  return (x >= 0.0) ? 1.0 : -1.0;
}

} // namespace controller
