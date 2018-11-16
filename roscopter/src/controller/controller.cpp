#include <controller/controller.h>
#include <stdio.h>

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{
  // Retrieve global MAV equilibrium throttle. This is the only MAV specific
  // parameter that is required
  ros::NodeHandle nh_mav(ros::this_node::getNamespace());
  if (!nh_private_.getParam("equilibrium_throttle", throttle_eq_))
    ROS_ERROR("[Controller] MAV equilibrium_throttle not found!");

  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
  max_accel_z_ = 1.0 / throttle_eq_;
  max_accel_xy_ = sin(acos(throttle_eq_)) / throttle_eq_;

  is_flying_ = false;

  nh_private_.getParam("max_roll", max_.roll);
  nh_private_.getParam("max_pitch", max_.pitch);
  nh_private_.getParam("max_yaw_rate", max_.yaw_rate);
  nh_private_.getParam("max_throttle", max_.throttle);
  nh_private_.getParam("max_n_dot", max_.n_dot);
  nh_private_.getParam("max_e_dot", max_.e_dot);
  nh_private_.getParam("max_z_dot", max_.z_dot);

  _func = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

  // Set up Publishers and Subscriber
  state_sub_ = nh_.subscribe("estimate", 1, &Controller::stateCallback, this);
  is_flying_sub_ = nh_.subscribe("is_flying", 1, &Controller::isFlyingCallback, this);
  cmd_sub_ = nh_.subscribe("high_level_command", 1, &Controller::cmdCallback, this);
  status_sub_ = nh_.subscribe("status", 1, &Controller::statusCallback, this);

  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);
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

  if(is_flying_ && armed_)
  {
    ROS_WARN_ONCE("CONTROLLER ACTIVE");
    computeControl(dt);
    publishCommand();
  }
  else
  {
    resetIntegrators();
    prev_time_ = msg->header.stamp.toSec();
  }
}


void Controller::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
{
  is_flying_ = msg->data;
}

void Controller::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
{
  armed_ = msg->armed;
}


void Controller::cmdCallback(const rosflight_msgs::CommandConstPtr &msg)
{
  switch(msg->mode)
  {
    case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      xc_.pn = msg->x;
      xc_.pe = msg->y;
      xc_.pd = msg->F;
      xc_.psi = msg->z;
      control_mode_ = msg->mode;
      break;
    case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
      xc_.x_dot = msg->x;
      xc_.y_dot = msg->y;
      xc_.pd = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
      xc_.ax = msg->x;
      xc_.ay = msg->y;
      xc_.az = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    default:
      ROS_ERROR("roscopter/controller: Unhandled command message of type %d", msg->mode);
      break;
  }
}


void Controller::reconfigure_callback(roscopter::ControllerConfig &config, uint32_t level)
{
  double P, I, D, tau;
  tau = config.tau;
  P = config.x_dot_P;
  I = config.x_dot_I;
  D = config.x_dot_D;
  PID_x_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = config.y_dot_P;
  I = config.y_dot_I;
  D = config.y_dot_D;
  PID_y_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = config.z_dot_P;
  I = config.z_dot_I;
  D = config.z_dot_D;
  PID_z_dot_.setGains(P, I, D, tau, 1.0, -max_accel_z_);

  P = config.north_P;
  I = config.north_I;
  D = config.north_D;
  max_.n_dot = config.max_n_dot;
  PID_n_.setGains(P, I, D, tau, max_.n_dot, -max_.n_dot);

  P = config.east_P;
  I = config.east_I;
  D = config.east_D;
  max_.e_dot = config.max_e_dot;
  PID_e_.setGains(P, I, D, tau, max_.e_dot, -max_.e_dot);

  P = config.z_P;
  I = config.z_I;
  D = config.z_D;
  max_.z_dot = config.max_z_dot;
  PID_z_.setGains(P, I, D, tau, max_.z_dot, -max_.z_dot);

  P = config.psi_P;
  I = config.psi_I;
  D = config.psi_D;
  PID_psi_.setGains(P, I, D, tau);

  max_.roll = config.max_roll;
  max_.pitch = config.max_pitch;
  max_.yaw_rate = config.max_yaw_rate;
  max_.throttle = config.max_throttle;

  ROS_INFO("new gains");

  resetIntegrators();
}


void Controller::computeControl(double dt)
{
  if(dt <= 0.0000001)
  {
    // This messes up the derivative calculation in the PID controllers
    return;
  }

  uint8_t mode_flag = control_mode_;

  if(mode_flag == rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
  {
    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_n_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_e_.computePID(xc_.pe, xhat_.pe, dt);

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
    xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);

    xc_.x_dot = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);
    xc_.y_dot = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);

    mode_flag = rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE)
  {
    // xc_.ax = saturate(PID_u_.computePID(xc_.u, xhat_.u, dt) + drag_constant_*xhat_.u /(9.80665 * mass_), max_a_, -max_a_);
    // xc_.ay = saturate(PID_v_.computePID(xc_.v, xhat_.v, dt) + drag_constant_*xhat_.v /(9.80665 * mass_), max_a_, -max_a_);
    //TODO: rotate xhat into vehicle 1 frame
    xc_.ax = PID_x_dot_.computePID(xc_.x_dot, xhat_.u, dt);
    xc_.ay = PID_y_dot_.computePID(xc_.y_dot, xhat_.v, dt);

    // Nested Loop for Altitude
    double pddot = -sin(xhat_.theta) * xhat_.u +
                   sin(xhat_.phi) * cos(xhat_.theta) * xhat_.v +
                   cos(xhat_.phi) * cos(xhat_.theta) * xhat_.w;
    double pddot_c = PID_z_dot_.computePID(xc_.pd, xhat_.pd, dt, pddot);
    xc_.az = PID_z_.computePID(pddot_c, pddot, dt);
    // ROS_INFO("pddot = %f, pddot_c = %f, az_c = %f", pddot, pddot_c, xc_.az);
    mode_flag = rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ)
  {
    // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
    // This model tends to pop the MAV up in the air when a large change
    // in control is commanded as the MAV rotates to it's commanded attitude while also ramping up throttle.
    // It works quite well, but it is a little oversimplified.
    double total_acc_c = sqrt((1.0-xc_.az)*(1.0-xc_.az) + xc_.ax*xc_.ax + xc_.ay*xc_.ay); // (in g's)
    // ROS_INFO("total_acc = %f", total_acc_c);
    if (total_acc_c > 0.001)
    {
      xc_.phi = asin(xc_.ay / total_acc_c);
      xc_.theta = -1.0*asin(xc_.ax / total_acc_c);
    }
    else
    {
      xc_.phi = 0;
      xc_.theta = 0;
    }

    // Calculate actual throttle (saturate az to be falling at 1 g)

    xc_.az = xc_.az;
    total_acc_c = sqrt((1.0-xc_.az)*(1.0-xc_.az) + xc_.ax*xc_.ax + xc_.ay*xc_.ay); // (in g's)
    xc_.throttle = total_acc_c*throttle_eq_; // calculate the total thrust in normalized units

    // ROS_INFO("xc_.az = %f, max_az = %f, total_acc_c = %f, throttle = %f", xc_.az, max_az_, total_acc_c, xc_.throttle);

    mode_flag = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    // Pack up and send the command
    command_.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.F = saturate(xc_.throttle, max_.throttle, 0.0);
    command_.x = saturate(xc_.phi, max_.roll, -max_.roll);
    command_.y = saturate(xc_.theta, max_.pitch, -max_.pitch);
    command_.z = saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);
  }
}

void Controller::publishCommand()
{
  command_pub_.publish(command_);
}

void Controller::resetIntegrators()
{
  PID_x_dot_.clearIntegrator();
  PID_y_dot_.clearIntegrator();
  PID_n_.clearIntegrator();
  PID_e_.clearIntegrator();
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
