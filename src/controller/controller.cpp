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
  cmd_sub_ = nh_.subscribe("high_level_command", 1, &Controller::cmdCallback, this);

  command_pub_ = nh_.advertise<fcu_common::Command>("command", 1);
}


void Controller::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  static double prev_time = 0;
  if(prev_time == 0)
  {
    prev_time = msg->header.stamp.toSec();
    return;
  }

  // Calculate time
  double now = msg->header.stamp.toSec();
  double dt = now - prev_time;
  prev_time = now;

  if(dt <= 0)
    return;

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
    computeControl(dt);
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


void Controller::cmdCallback(const fcu_common::CommandConstPtr &msg)
{
  switch(msg->mode)
  {
    case fcu_common::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      xc_.pn = msg->x;
      xc_.pe = msg->y;
      xc_.pd = msg->F;
      xc_.psi = msg->z;
      control_mode_ = msg->mode;
      break;
    case fcu_common::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
      xc_.u = msg->x;
      xc_.v = msg->y;
      xc_.pd = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    case fcu_common::Command::MODE_XACC_YACC_YAWRATE_AZ:
      xc_.ax = msg->x;
      xc_.ay = msg->y;
      xc_.az = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    default:
      ROS_ERROR("ros_copter/controller: Unhandled command message of type %d", msg->mode);
      break;
  }
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


void Controller::computeControl(double dt)
{
  if(dt <= 0.0)
  {
    // This messes up the derivative calculation in the PID controllers
    return;
  }

  uint8_t mode_flag = control_mode_;

  if(mode_flag == fcu_common::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
  {
    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_x_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_y_.computePID(xc_.pe, xhat_.pe, dt);

    // Calculate desired yaw rate
    // First, determine the shortest direction to the commanded psi
    if(fabs(xc_.psi + 2*M_PI - xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi += 2*M_PI;
    }
    else if (fabs(xc_.psi - 2*M_PI -xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi -= 2*M_PI;
    }
    xc_.r = saturate(PID_psi_.computePID(xc_.psi, xhat_.psi, dt), max_.yaw_rate, -max_.yaw_rate);

    // Rotate into body frame
    /// TODO: Include pitch and roll in this mapping
    xc_.u = saturate(pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi), max_.u, -1.0*max_.u);
    xc_.v = saturate(-pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi), max_.v, -1.0*max_.v);

    mode_flag = fcu_common::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE;
  }

  if(mode_flag == fcu_common::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE)
  {
    /// TODO: Maxes are wrong
    xc_.ax = saturate(PID_u_.computePID(xc_.u, xhat_.u, dt), max_.roll, -max_.roll);
    xc_.ay = saturate(PID_v_.computePID(xc_.v, xhat_.v, dt), max_.pitch, -max_.pitch);
    xc_.az = PID_z_.computePID(xc_.pd, xhat_.pd, dt);

    // saturate az so that we don't shoot off into the sky if we are too high above our setpoint
    if(xc_.az > 1.0)
    {
        xc_.az = 1.0;
    }
    mode_flag = fcu_common::Command::MODE_XACC_YACC_YAWRATE_AZ;
  }

  if(mode_flag == fcu_common::Command::MODE_XACC_YACC_YAWRATE_AZ)
  {
    // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
    // This model does not take into account drag.  If drag were being estimated, then we could
    // perform even better control.  It also tends to pop the MAV up in the air when a large change
    // in control is commanded as the MAV rotates to it's commanded attitude while also ramping up throttle.
    // It works quite well, but it is a little oversimplified.
    double total_acc_c = sqrt((1.0-xc_.az)*(1.0-xc_.az) + xc_.ax*xc_.ax + xc_.ay*xc_.ay); // (in g's)
    xc_.throttle = total_acc_c*thrust_eq_; // calculate the total thrust in normalized units
    xc_.phi = asin(xc_.ay / total_acc_c);
    xc_.theta = -1.0*asin(xc_.ax / total_acc_c);
    mode_flag = fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }

  if(mode_flag == fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    // Pack up and send the command
    command_.mode = fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.F = saturate(xc_.throttle, max_.throttle, 0.0);
    command_.x = xc_.phi;
    command_.y = xc_.theta;
    command_.z = xc_.r;
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
