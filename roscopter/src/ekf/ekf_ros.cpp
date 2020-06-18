// BSD 3-Clause License
//
// Copyright (c) 2017, James Jackson, BYU MAGICC Lab, Provo UT
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ros/package.h>

#include "ekf/state.h"
#include "ekf/ekf_ros.h"
#include "roscopter_utils/yaml.h"
#include "roscopter_utils/gnss.h"

using namespace Eigen;

namespace roscopter::ekf
{

EKF_ROS::EKF_ROS() :
  nh_(), nh_private_("~")
{}

EKF_ROS::~EKF_ROS()
{}

///////entry point
//called from ekf_node.cpp int main
//calls EKF_ROS::init
//calls callbacks in EKF_ROS through subscription
void EKF_ROS::initROS()
{

  //sets a variable to the parameter file path and name
  std::string roscopter_path = ros::package::getPath("roscopter");
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", roscopter_path + "/params/ekf.yaml");

  //This gets sensor and propagation noise paramters and assigns them R variables example imu_R_
  init(parameter_filename);

  //sets up publishing. Number referes to queue size.  All publishers are called in ekf_ros.cpp
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  euler_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("euler_degrees", 1);
  imu_bias_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_bias", 1);
  gps_ned_cov_pub_ = nh_.advertise<geometry_msgs::PoseWithCovariance>("gps_ned_cov", 1);
  gps_ecef_cov_pub_ = nh_.advertise<geometry_msgs::PoseWithCovariance>("gps_ecef_cov", 1);
  is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1);

  //sets up subscriptions.  Number referes to queue size
  //with subscription, the callbacks are "listening for messages, and they are the next step in the code"
  //all subscriptions are set up to be used in ekf_ros.cpp
  //callbacks from range and status are set up but not subscribed to here.
  imu_sub_ = nh_.subscribe("imu", 100, &EKF_ROS::imuCallback, this);
  baro_sub_ = nh_.subscribe("baro", 100, &EKF_ROS::baroCallback, this);
  pose_sub_ = nh_.subscribe("pose", 10, &EKF_ROS::poseCallback, this);
  odom_sub_ = nh_.subscribe("reference", 10, &EKF_ROS::odomCallback, this);
  gnss_sub_ = nh_.subscribe("gnss", 10, &EKF_ROS::gnssCallback, this);

// These are defined if found when the project is built.  This can be seen in the CMakeLists.txt
// Subscribes if found
#ifdef UBLOX
  ublox_gnss_sub_ = nh_.subscribe("ublox_gnss", 10, &EKF_ROS::gnssCallbackUblox, this);
  ublox_relpos_sub_ = nh_.subscribe("ublox_relpos", 10, &EKF_ROS::gnssCallbackRelPos, this);
  ublox_posvelecef_sub_ = nh_.subscribe("ublox_posvelecef", 10, &EKF_ROS::gnssCallbackBasevel, this);
  base_relPos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("base_relPos", 1);
  base_Vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("base_vel", 1);
  std::cerr << "UBLOX is defined \n";
#endif
#ifdef INERTIAL_SENSE
  is_gnss_sub_ = nh_.subscribe("is_gnss", 10, &EKF_ROS::gnssCallbackInertialSense, this);
#endif

  ros_initialized_ = true;
}

///////loads noise parameters for propagation and sensor updates, sets start time
//called from EKF_ROS::initROS
//calls ekf_.load
//gets paramters from ekf.yaml
void EKF_ROS::init(const std::string &param_file)
{

  //ekf_ object comes from EKF class in ekf.cpp.  Instatiated in header file
  //../../params/ekf.yaml
  ekf_.load(param_file);

  // Load Sensor Noise Parameters
  double acc_stdev, gyro_stdev;
  //get_yaml_node(stuff in quotes from the param file gets assigned to final variable) not sure where it is defined
  get_yaml_node("accel_noise_stdev", param_file, acc_stdev);
  get_yaml_node("gyro_noise_stdev", param_file, gyro_stdev);

  //R variables are defined in ekf_ros.h and can be accessed by the entire class
  //R matrix for sensor (in this case imu is actually propagation) noise
  imu_R_.setZero();
  imu_R_.topLeftCorner<3,3>() = acc_stdev * acc_stdev * I_3x3;
  imu_R_.bottomRightCorner<3,3>() = gyro_stdev * gyro_stdev * I_3x3;

  // R matrix for mocap sensor noise (Sigma)
  double pos_stdev, att_stdev;
  get_yaml_node("position_noise_stdev", param_file, pos_stdev);
  get_yaml_node("attitude_noise_stdev", param_file, att_stdev);
  mocap_R_ << pos_stdev * pos_stdev * I_3x3,   Matrix3d::Zero(),
      Matrix3d::Zero(),   att_stdev * att_stdev * I_3x3;

  // R matrix for barometer sensor noise (Sigma)
  double baro_pressure_stdev;
  get_yaml_node("baro_pressure_noise_stdev", param_file, baro_pressure_stdev);
  baro_R_ = baro_pressure_stdev * baro_pressure_stdev;

  // R matrix for range sensor noise (Sigma)
  double range_stdev;
  get_yaml_node("range_noise_stdev", param_file, range_stdev);
  range_R_ = range_stdev * range_stdev;

  // R matrix for manual gps sensor noise (Sigma) if not using the values reported from the message topic
  // This can be set false or true in the param file
  get_yaml_node("manual_gps_noise", param_file, manual_gps_noise_);
  if (manual_gps_noise_)
  {
    get_yaml_node("gps_horizontal_stdev", param_file, gps_horizontal_stdev_);
    get_yaml_node("gps_vertical_stdev", param_file, gps_vertical_stdev_);
    get_yaml_node("gps_speed_stdev", param_file, gps_speed_stdev_);
  }

  start_time_.fromSec(0.0);
}

//////This function is called each time imu is received and publishes odom, 
//called by EKF_ROS::imuCallback
//calls EKF::x, EKF::isflying
//publishes odom, euler, imu_bias, is_flying
void EKF_ROS::publishEstimates(const sensor_msgs::ImuConstPtr &msg)
{

  // Pub Odom
  odom_msg_.header = msg->header;

  //ekf_.x can be found in ekf.h.  It is a function that returns xbuf.x.  state_est is used throughout the function
  //xbuf_.x is all the information about the state that we use, it comes from ekf.cpp
  const State state_est = ekf_.x();
  odom_msg_.pose.pose.position.x = state_est.p(0);
  odom_msg_.pose.pose.position.y = state_est.p(1);
  odom_msg_.pose.pose.position.z = state_est.p(2);

  odom_msg_.pose.pose.orientation.w = state_est.q.w();
  odom_msg_.pose.pose.orientation.x = state_est.q.x();
  odom_msg_.pose.pose.orientation.y = state_est.q.y();
  odom_msg_.pose.pose.orientation.z = state_est.q.z();

  odom_msg_.twist.twist.linear.x = state_est.v(0);
  odom_msg_.twist.twist.linear.y = state_est.v(1);
  odom_msg_.twist.twist.linear.z = state_est.v(2);

  odom_msg_.twist.twist.angular.x = state_est.w(0);
  odom_msg_.twist.twist.angular.y = state_est.w(1);
  odom_msg_.twist.twist.angular.z = state_est.w(2);

  odometry_pub_.publish(odom_msg_);

  // Pub Euler Attitude
  euler_msg_.header = msg->header;
  //grabs state est.q from above, then converts to euler
  const Eigen::Vector3d euler_angles = state_est.q.euler() * 180. / M_PI;
  euler_msg_.vector.x = euler_angles(0);
  euler_msg_.vector.y = euler_angles(1);
  euler_msg_.vector.z = euler_angles(2);

  euler_pub_.publish(euler_msg_);

  // Pub Imu Bias estimate
  imu_bias_msg_.header = msg->header;

  imu_bias_msg_.angular_velocity.x = state_est.bg(0);
  imu_bias_msg_.angular_velocity.y = state_est.bg(1);
  imu_bias_msg_.angular_velocity.z = state_est.bg(2);

  imu_bias_msg_.linear_acceleration.x = state_est.ba(0);
  imu_bias_msg_.linear_acceleration.y = state_est.ba(1);
  imu_bias_msg_.linear_acceleration.z = state_est.ba(2);

  // linear acceleration covariance
  imu_bias_msg_.linear_acceleration_covariance[0] = imu_R_(0,0);
  imu_bias_msg_.linear_acceleration_covariance[1] = imu_R_(0,1);
  imu_bias_msg_.linear_acceleration_covariance[2] = imu_R_(0,2);
  imu_bias_msg_.linear_acceleration_covariance[3] = imu_R_(1,0);
  imu_bias_msg_.linear_acceleration_covariance[4] = imu_R_(1,1);
  imu_bias_msg_.linear_acceleration_covariance[5] = imu_R_(1,2);
  imu_bias_msg_.linear_acceleration_covariance[6] = imu_R_(2,0);
  imu_bias_msg_.linear_acceleration_covariance[7] = imu_R_(2,1);
  imu_bias_msg_.linear_acceleration_covariance[8] = imu_R_(2,2);

  imu_bias_pub_.publish(imu_bias_msg_);

  // Only publish is_flying is true once
  if (!is_flying_) //set to false in header file.  returns value in isflying variable.  This variable is set in EKF::checkIsFlying 
  {
    is_flying_ = ekf_.isFlying();
    if (is_flying_)
    {
      is_flying_msg_.data = is_flying_;
      is_flying_pub_.publish(is_flying_msg_);
    }
  }
}

void EKF_ROS::publishGpsCov(Matrix6d sigma_ecef, Vector6d sigma_ned, Vector6d z)
{
  // const Eigen::Vector3d& ecef
  // z_ned = x_ecef2ned(const Eigen::Vector3d& ecef)

  //gps ecef covariance
  gps_ecef_cov_msg_.covariance[0] = sigma_ecef(0,0);
  gps_ecef_cov_msg_.covariance[1] = sigma_ecef(0,1);
  gps_ecef_cov_msg_.covariance[2] = sigma_ecef(0,2);
  gps_ecef_cov_msg_.covariance[3] = sigma_ecef(1,0);
  gps_ecef_cov_msg_.covariance[4] = sigma_ecef(1,1);
  gps_ecef_cov_msg_.covariance[5] = sigma_ecef(1,2);
  gps_ecef_cov_msg_.covariance[6] = sigma_ecef(2,0);
  gps_ecef_cov_msg_.covariance[7] = sigma_ecef(2,1);
  gps_ecef_cov_msg_.covariance[8] = sigma_ecef(2,2);

  // gps ned covariance
  gps_ned_cov_msg_.covariance[0] = sigma_ned[0];
  gps_ned_cov_msg_.covariance[1] = sigma_ned[1];
  gps_ned_cov_msg_.covariance[2] = sigma_ned[2];
  gps_ned_cov_msg_.covariance[3] = sigma_ned[3];
  gps_ned_cov_msg_.covariance[4] = sigma_ned[4];
  gps_ned_cov_msg_.covariance[5] = sigma_ned[5];

  //convert z to ned frame
  Vector3d z_lla = ecef2lla(z.head<3>());
  Vector3d ref_lla(ekf_.ref_lat_radians_, ekf_.ref_lon_radians_, ekf_.x().ref);
  Vector3d z_ned = lla2ned(ref_lla, z_lla);

  //get gps ned position
  gps_ned_cov_msg_.pose.position.x = z_ned[0];
  gps_ned_cov_msg_.pose.position.y = z_ned[1];
  gps_ned_cov_msg_.pose.position.z = z_ned[2];

  gps_ned_cov_pub_.publish(gps_ned_cov_msg_);
  gps_ecef_cov_pub_.publish(gps_ecef_cov_msg_);
}

void EKF_ROS::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  //initializes time on first imu callback
  if (start_time_.sec == 0)
  {
    start_time_ = msg->header.stamp;
  }

  //I'd assume Vector 6d is a type defined somewhere, but I couldn't find it
  Vector6d z;
  //Assigning elements of the imu message to z.
  z << msg->linear_acceleration.x,
       msg->linear_acceleration.y,
       msg->linear_acceleration.z,
       msg->angular_velocity.x,
       msg->angular_velocity.y,
       msg->angular_velocity.z;

  double t = (msg->header.stamp - start_time_).toSec();
  ekf_.imuCallback(t, z, imu_R_); //time, measurment, and noise

  if(ros_initialized_) //flag set true in EKF_ROS::initRos
    publishEstimates(msg);
}

//////similar to imuCallback
//Called from EKF_ROS::initRos through subscription
//calls EKF::groundTempPressSet, EKF::setGroundTempPressure, EKF::baroCallback
void EKF_ROS::baroCallback(const rosflight_msgs::BarometerConstPtr& msg)
{

  const double pressure_meas = msg->pressure;
  const double temperature_meas = msg->temperature;

  // if statement sets ground pressure and temperature if not already set
  if (!ekf_.groundTempPressSet()) //function contained in ekf.h, returns true if ground pressure and groudn temperature are not equal to 0.
  {
    std::cout << "Set ground pressure and temp" << std::endl;
    std::cout << "press: " << pressure_meas << std::endl;
    //sets ground temperature and ground pressure on first baroCallback
    ekf_.setGroundTempPressure(temperature_meas, pressure_meas);
  }

  if (start_time_.sec == 0)
    return;

  const double t = (msg->header.stamp - start_time_).toSec();
  ekf_.baroCallback(t, pressure_meas, baro_R_, temperature_meas);
}

//no subscription to range_Callback
void EKF_ROS::rangeCallback(const sensor_msgs::RangeConstPtr& msg)
{
  if (start_time_.sec == 0)
    return;

  const double range_meas = msg->range;
  if (range_meas < msg->max_range && range_meas > msg->min_range)
  {
    const double t = (msg->header.stamp - start_time_).toSec();
    ekf_.rangeCallback(t, range_meas, range_R_);
  }
}

//////similar to imuCallback
//Called from EKF_ROS::initRos through subscription
//calls mocapCallback
void EKF_ROS::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  //xform is from geometry/xform.h
  xform::Xformd z;
  z.arr_ << msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z,
          msg->pose.orientation.w,
          msg->pose.orientation.x,
          msg->pose.orientation.y,
          msg->pose.orientation.z;

  mocapCallback(msg->header.stamp, z);
}

//////just like poseCallLback
//Called from EKF_ROS::initRos through subscription
//calls mocapCallback
void EKF_ROS::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  
  xform::Xformd z;
  z.arr_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z;

  mocapCallback(msg->header.stamp, z);
}

//both odometry and pose can be used for mocap
//Called from EKF_ROS::poseCallback and EKF_ROS::odomCallback
//calls EKF::mocapCallback
void EKF_ROS::mocapCallback(const ros::Time &time, const xform::Xformd &z)
{
  if (start_time_.sec == 0)
    return;

  double t = (time - start_time_).toSec();
  ekf_.mocapCallback(t, z, mocap_R_);
}

//no subscription 
void EKF_ROS::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
{
  if (msg->armed)
  {
    ekf_.setArmed();
  }
  else
  {
    ekf_.setDisarmed();
  }
}

//similar to EKF_ROS::imuCallback, but it also sets the original lla and converts to ecef
//Called from EKF_ROS::initRos through subscription, EKF_ROS::gnssCallbackUBLOX, EKF_ROS::gnssCallbackInertialSense
//calls EKF::refLlaSet, EKF::setRefLlla, EKF::gnssCallback
void EKF_ROS::gnssCallback(const rosflight_msgs::GNSSConstPtr &msg)
{

  Vector6d z;
  z << msg->position[0],
       msg->position[1],
       msg->position[2],
       msg->velocity[0],
       msg->velocity[1],
       msg->velocity[2];

  // rotate covariance into the ECEF frame
  Vector6d Sigma_diag_NED;
  if (manual_gps_noise_) //comes from ekf.yaml
  {
    Sigma_diag_NED << gps_horizontal_stdev_,
                      gps_horizontal_stdev_,
                      gps_vertical_stdev_,
                      gps_speed_stdev_,
                      gps_speed_stdev_,
                      gps_speed_stdev_;
  }
  else //using reported values from message
  {
    Sigma_diag_NED << msg->horizontal_accuracy,
                  msg->horizontal_accuracy,
                  msg->vertical_accuracy,
                  msg->speed_accuracy,
                  msg->speed_accuracy,
                  msg->speed_accuracy;
  }
  //cwiseProduct returns an expression of the Schur product (coefficient wise product) of *this and other
  //looks like it is just element wise matrix multiplication.  So this is just squaring each element of Sigma_diag_NED. https://eigen.tuxfamily.org/dox/classEigen_1_1MatrixBase.html#title37
  Sigma_diag_NED = Sigma_diag_NED.cwiseProduct(Sigma_diag_NED);

  Matrix3d R_e2n = q_e2n(ecef2lla(z.head<3>())).R();

  //using the rotation obtained above from z, rotate the covariance into the ecef frame.  The transposes flip the rotation
  Matrix6d Sigma_ecef;
  Sigma_ecef << R_e2n.transpose() * Sigma_diag_NED.head<3>().asDiagonal() * R_e2n, Matrix3d::Zero(),
                Matrix3d::Zero(), R_e2n.transpose() *  Sigma_diag_NED.tail<3>().asDiagonal() * R_e2n;

  if (!ekf_.refLlaSet())
  {
    // set ref lla to first gps position
    Eigen::Vector3d ref_lla = ecef2lla(z.head<3>());
    // Convert radians to degrees
    ref_lla.head<2>() *= 180. / M_PI;
    ekf_.setRefLla(ref_lla);
  }

  publishGpsCov(Sigma_ecef, Sigma_diag_NED, z); //delete this later

  if (start_time_.sec == 0)
    return;

  double t = (msg->header.stamp - start_time_).toSec();
  ekf_.gnssCallback(t, z, Sigma_ecef);
}

/////Grabs needed values from ublox ecef message and puts it in a rosflight GNSS message
//Called from EKF_ROS::initRos through subscription if UBLOX found
//calls EKF_Ros::gnssCallback
//???it looks like the reference lla is not set in here.
#ifdef UBLOX
void EKF_ROS::gnssCallbackUblox(const ublox::PosVelEcefConstPtr &msg)
{
  //only uses the data if gnss is fixed as 2D or 3D
  if (msg->fix == ublox::PosVelEcef::FIX_TYPE_2D
      || msg->fix == ublox::PosVelEcef::FIX_TYPE_3D)
  {
    rosflight_msgs::GNSS rf_msg;
    rf_msg.header.stamp = msg->header.stamp;
    rf_msg.position = msg->position;
    rf_msg.velocity = msg->velocity;
    rf_msg.horizontal_accuracy = msg->horizontal_accuracy;
    rf_msg.vertical_accuracy = msg->vertical_accuracy;
    rf_msg.speed_accuracy = msg->speed_accuracy;
    gnssCallback(boost::make_shared<rosflight_msgs::GNSS>(rf_msg));
  }
  else
  {
    ROS_WARN_THROTTLE(1., "Ublox GPS not in fix");
  }
}

void EKF_ROS::gnssCallbackRelPos(const ublox::RelPosConstPtr &msg)
{
  //TODO:: put in logic to only use measurements if a flag of 311, 279, 271, or ... xxx, is found
  //TODO:: maybe put in logic to only move forward if in a landing state
  base_relPos_msg_.header = msg->header;
  // negate relPos message to go from rover to base rather than base to rover
  base_relPos_msg_.point.x = -msg->relPosNED[0];
  base_relPos_msg_.point.y = -msg->relPosNED[1];
  base_relPos_msg_.point.z = -msg->relPosNED[2];  
  //TODO:: could add in the high precision (portion less than a mm)
  //TODO:: could add in the accuracy of the NED measurment to update covariance
  base_relPos_pub_.publish(base_relPos_msg_);

}

void EKF_ROS::gnssCallbackBasevel(const ublox::PosVelEcefConstPtr &msg)
{
  base_Vel_msg_.twist.linear.x = msg->velocity[0];
  base_Vel_msg_.twist.linear.y = msg->velocity[1];
  base_Vel_msg_.twist.linear.z = msg->velocity[2];

  base_Vel_pub_.publish(base_Vel_msg_);
}
#endif

/////simialr to EKF::ROS::gnssCallbackUblox
//Called from EKF_ROS::initRos through subscription if Inertial Sense found
//calls EKF_Ros::gnssCallback
//???it looks like the reference lla is not set in here.
#ifdef INERTIAL_SENSE
void EKF_ROS::gnssCallbackInertialSense(const inertial_sense::GPSConstPtr &msg)
{
  if (msg->fix_type == inertial_sense::GPS::GPS_STATUS_FIX_TYPE_2D_FIX
      || msg->fix_type == inertial_sense::GPS::GPS_STATUS_FIX_TYPE_3D_FIX)
  {
    rosflight_msgs::GNSS rf_msg;
    rf_msg.header.stamp = msg->header.stamp;
    rf_msg.position[0] = msg->posEcef.x;
    rf_msg.position[1] = msg->posEcef.y;
    rf_msg.position[2] = msg->posEcef.z;
    rf_msg.velocity[0] = msg->velEcef.x;
    rf_msg.velocity[1] = msg->velEcef.y;
    rf_msg.velocity[2] = msg->velEcef.z;
    rf_msg.horizontal_accuracy = msg->hAcc;
    rf_msg.vertical_accuracy = msg->vAcc;
    rf_msg.speed_accuracy = 0.3;
    gnssCallback(boost::make_shared<rosflight_msgs::GNSS>(rf_msg));
  }
  else
  {
    ROS_WARN_THROTTLE(1., "Inertial Sense GPS not in fix");
  }
}
#endif




}
