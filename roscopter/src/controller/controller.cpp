#include <controller/controller.h>

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_("controller")
{
  // retrieve parameters from ROS parameter server
  roscopter_common::rosImportScalar<double>(nh_private_, "equilibrium_throttle", throttle_eq_, 0.5);
  roscopter_common::rosImportScalar<double>(nh_private_, "max_roll", max_.roll, 0.3);
  roscopter_common::rosImportScalar<double>(nh_private_, "max_pitch", max_.pitch, 0.3);
  roscopter_common::rosImportScalar<double>(nh_private_, "max_yaw_rate", max_.yaw_rate, 1.57);
  roscopter_common::rosImportScalar<double>(nh_private_, "max_throttle", max_.throttle, 1);
  roscopter_common::rosImportScalar<double>(nh_private_, "max_vel", max_.vel, 5);
  is_flying_ = false;
  prev_time_ = 0;
  dhat_.setZero();

  _func = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

  // Set up Publishers and Subscriber
  state_sub_ = nh_.subscribe("estimate", 1, &Controller::stateCallback, this);
  is_flying_sub_ = nh_.subscribe("is_flying", 1, &Controller::isFlyingCallback, this);
  cmd_sub_ = nh_.subscribe("high_level_command", 1, &Controller::cmdCallback, this);

  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);
}


void Controller::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // Calculate time step
  double dt;
  double now = msg->header.stamp.toSec();
  if(prev_time_ == 0)
  {
    prev_time_ = now;
    return;
  }
  else
  {
    dt = now - prev_time_;
    prev_time_ = now;
  }

  // This should already be coming in NED
  xhat_.pn = msg->pose.pose.position.x;
  xhat_.pe = msg->pose.pose.position.y;
  xhat_.pd = msg->pose.pose.position.z;

  xhat_.u = msg->twist.twist.linear.x;
  xhat_.v = msg->twist.twist.linear.y;
  xhat_.w = msg->twist.twist.linear.z;

  // Convert Quaternion to RPY
  roscopter_common::Quaternion quat(msg->pose.pose.orientation.w,
                                    msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z);
  xhat_.phi = quat.roll();
  xhat_.theta = quat.pitch();
  xhat_.psi = quat.yaw();

  xhat_.p = msg->twist.twist.angular.x;
  xhat_.q = msg->twist.twist.angular.y;
  xhat_.r = msg->twist.twist.angular.z;

  if(is_flying_)
  {
    computeControl(dt);
    command_pub_.publish(command_);
  }
}


void Controller::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
{
  is_flying_ = msg->data;
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
      xc_.u = msg->x;
      xc_.v = msg->y;
      xc_.pd = msg->F;
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
  K_p_ << config.kp_x,           0,           0,
                    0, config.kp_y,           0,
                    0,           0, config.kp_z;
  K_v_ << config.kv_x,           0,           0,
                    0, config.kv_y,           0,
                    0,           0, config.kv_z;
  K_d_ << config.kd_x,           0,           0,
                    0, config.kd_y,           0,
                    0,           0, config.kd_z;
  max_.roll = config.max_roll;
  max_.pitch = config.max_pitch;
  max_.yaw_rate = config.max_yaw_rate;
  max_.throttle = config.max_throttle;
  max_.vel = config.max_vel;
  ROS_INFO("New gains!");
}


void Controller::computeControl(double dt)
{
  // copy so it can be modified
  uint8_t mode_flag = control_mode_;

  // get data that applies to both position and velocity control
  Eigen::Matrix3d R_v_to_v1 = roscopter_common::R_v_to_v1(xhat_.psi); // rotation from vehicle to vehicle-1 frame
  Eigen::Matrix3d R_v1_to_b = roscopter_common::R_v_to_b(xhat_.phi,xhat_.theta,0); // rotation from vehicle-1 to body frame
  static Eigen::Vector3d k(0,0,1); // general unit vector in z-direction
  static double gravity = 9.80665; // m/s^2

  if(mode_flag == rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
  {
    Eigen::Vector3d phat(xhat_.pn,xhat_.pe,xhat_.pd); // position estimate
    Eigen::Vector3d pc(xc_.pn,xc_.pe,xc_.pd); // position command
    Eigen::Vector3d vc = R_v_to_v1*K_p_*(pc-phat); // velocity command

    // store velocity command
    xc_.u = vc(0);
    xc_.v = vc(1);

    // get yaw rate direction and allow it to saturate
    xc_.r = xc_.psi - xhat_.psi;
    
    // change flag to compute needed velocities
    mode_flag = rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE)
  {
    // get velocity command and enforce max velocity
    Eigen::Vector3d vc(xc_.u,xc_.v,K_p_(2,2)*(xc_.pd-xhat_.pd));
    double vmag = vc.norm();
    if (vmag > max_.vel)
      vc = vc*max_.vel/vmag;

    Eigen::Vector3d vhat_b(xhat_.u,xhat_.v,xhat_.w); // body velocity estimate
    Eigen::Vector3d vhat = R_v1_to_b.transpose()*vhat_b; // vehicle-1 velocity estimate
    dhat_ = dhat_ - K_d_*(vc-vhat)*dt; // update disturbance estimate
    Eigen::Vector3d k_tilde = throttle_eq_/gravity*(k-(K_v_*(vc-vhat)-dhat_));

    // pack up throttle command 
    xc_.throttle = k.transpose()*R_v1_to_b*k_tilde;

    Eigen::Vector3d kd = (1.0/xc_.throttle)*k_tilde; // desired body z direction
    kd = kd/kd.norm(); // need direction only
    double tilt_angle = acos(k.transpose()*kd); // desired tilt

    // get shortest rotation to desired tilt
    roscopter_common::Quaternion q_c;
    if (tilt_angle < 1e-6)
    {
      q_c = roscopter_common::Quaternion();
    }
    else
    {
      Eigen::Vector3d k_cross_kd = roscopter_common::skew(k)*kd;
      q_c = roscopter_common::exp_q(tilt_angle*k_cross_kd/k_cross_kd.norm());
    }

    // pack up attitude commands
    xc_.phi = q_c.roll();
    xc_.theta = q_c.pitch();
    
    mode_flag = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    // Pack up and send the command
    command_.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.F = roscopter_common::saturate(xc_.throttle, max_.throttle, 0.0);
    command_.x = roscopter_common::saturate(xc_.phi, max_.roll, -max_.roll);
    command_.y = roscopter_common::saturate(xc_.theta, max_.pitch, -max_.pitch);
    command_.z = roscopter_common::saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);
  }
}

} // namespace controller

