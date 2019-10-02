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

void EKF_ROS::initROS()
{
  std::string roscopter_path = ros::package::getPath("roscopter");
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", roscopter_path + "/params/ekf.yaml");

  init(parameter_filename);

  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  imu_bias_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_bias", 1);
  is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1);

  imu_sub_ = nh_.subscribe("imu", 100, &EKF_ROS::imuCallback, this);
  pose_sub_ = nh_.subscribe("pose", 10, &EKF_ROS::poseCallback, this);
  odom_sub_ = nh_.subscribe("reference", 10, &EKF_ROS::odomCallback, this);
  gnss_sub_ = nh_.subscribe("gnss", 10, &EKF_ROS::gnssCallback, this);

  ros_initialized_ = true;

#ifdef INERTIAL_SENSE
  is_gnss_sub_ = nh_.subscribe("is_gnss", 10, &EKF_ROS::gnssCallbackInertialSense, this);
#endif
}

void EKF_ROS::init(const std::string &param_file)
{
  ekf_.load(param_file);

  // Load Sensor Noise Parameters
  double acc_stdev, gyro_stdev;
  get_yaml_node("accel_noise_stdev", param_file, acc_stdev);
  get_yaml_node("gyro_noise_stdev", param_file, gyro_stdev);
  imu_R_.setZero();
  imu_R_.topLeftCorner<3,3>() = acc_stdev * acc_stdev * I_3x3;
  imu_R_.bottomRightCorner<3,3>() = gyro_stdev * gyro_stdev * I_3x3;

  double pos_stdev, att_stdev;
  get_yaml_node("position_noise_stdev", param_file, pos_stdev);
  get_yaml_node("attitude_noise_stdev", param_file, att_stdev);
  mocap_R_ << pos_stdev * pos_stdev * I_3x3,   Matrix3d::Zero(),
      Matrix3d::Zero(),   att_stdev * att_stdev * I_3x3;

  double alt_stdev;
  get_yaml_node("alt_noise_stdev", param_file, alt_stdev);
  alt_R_ << alt_stdev*alt_stdev;
  start_time_.fromSec(0.0);
}

void EKF_ROS::publishEstimates(const sensor_msgs::ImuConstPtr &msg)
{
  // Pub Odom
  odom_msg_.header = msg->header;

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

  odometry_pub_.publish(odom_msg_);

  // Pub Imu Bias estimate
  imu_bias_msg_.header = msg->header;

  imu_bias_msg_.angular_velocity.x = state_est.bg(0);
  imu_bias_msg_.angular_velocity.y = state_est.bg(1);
  imu_bias_msg_.angular_velocity.z = state_est.bg(2);

  imu_bias_msg_.linear_acceleration.x = state_est.ba(0);
  imu_bias_msg_.linear_acceleration.y = state_est.ba(1);
  imu_bias_msg_.linear_acceleration.z = state_est.ba(2);

  imu_bias_pub_.publish(imu_bias_msg_);

  // Only publish is_flying is true once
  if (!is_flying_)
  {
    is_flying_ = ekf_.isFlying();
    if (is_flying_)
    {
      is_flying_msg_.data = is_flying_;
      is_flying_pub_.publish(is_flying_msg_);
    }
  }
}

void EKF_ROS::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  if (start_time_.sec == 0)
  {
    start_time_ = msg->header.stamp;
    return;
  }

  Vector6d z;
  z << msg->linear_acceleration.x,
       msg->linear_acceleration.y,
       msg->linear_acceleration.z,
       msg->angular_velocity.x,
       msg->angular_velocity.y,
       msg->angular_velocity.z;

  double t = (msg->header.stamp - start_time_).toSec();
  ekf_.imuCallback(t, z, imu_R_);

  if(ros_initialized_)
    publishEstimates(msg);
}

void EKF_ROS::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
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

void EKF_ROS::mocapCallback(const ros::Time &time, const xform::Xformd &z)
{
  if (start_time_.sec == 0)
    return;

  double t = (time - start_time_).toSec();
  ekf_.mocapCallback(t, z, mocap_R_);
}

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

void EKF_ROS::gnssCallback(const rosflight_msgs::GNSSConstPtr &msg)
{
  if (start_time_.sec == 0)
    return;

  Vector6d z;
  z << msg->position[0],
       msg->position[1],
       msg->position[2],
       msg->velocity[0],
       msg->velocity[1],
       msg->velocity[2];

  // rotate covariance into the ECEF frame
  Vector6d Sigma_diag_NED;
  Sigma_diag_NED << msg->horizontal_accuracy,
                msg->horizontal_accuracy,
                msg->vertical_accuracy,
                msg->speed_accuracy,
                msg->speed_accuracy,
                msg->speed_accuracy;
  Sigma_diag_NED = Sigma_diag_NED.cwiseProduct(Sigma_diag_NED);
  Matrix3d R_e2n = q_e2n(ecef2lla(z.head<3>())).R();
  Matrix6d Sigma_ecef;
  Sigma_ecef << R_e2n.transpose() * Sigma_diag_NED.head<3>().asDiagonal() * R_e2n, Matrix3d::Zero(),
                Matrix3d::Zero(), R_e2n.transpose() *  Sigma_diag_NED.tail<3>().asDiagonal() * R_e2n;

  double t = (msg->header.stamp - start_time_).toSec();
  ekf_.gnssCallback(t, z, Sigma_ecef);
}

#ifdef UBLOX
void EKF_ROS::gnssCallbackUblox(const ublox::PosVelEcefConstPtr &msg)
{
  if (msg->fix == ublox::PosVelEcef::FIX_TYPE_2D
      || msg->fix == ublox::PosVelEcef::FIX_TYPE_3D)
  {
    if (!ekf_.refLlaSet())
    {
      // set ref lla to first gps position
      Eigen::Vector3d ref_lla;
      ref_lla(0) = msg->lla[0];
      ref_lla(1) = msg->lla[1];
      ref_lla(2) = msg->lla[2];
      ekf_.setRefLla(ref_lla);
    }

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
#endif

#ifdef INERTIAL_SENSE
void EKF_ROS::gnssCallbackInertialSense(const inertial_sense::GPSConstPtr &msg)
{
  if (msg->fix_type == inertial_sense::GPS::GPS_STATUS_FIX_TYPE_2D_FIX
      || msg->fix_type == inertial_sense::GPS::GPS_STATUS_FIX_TYPE_3D_FIX)
  {
    if (!ekf_.refLlaSet())
    {
      // set ref lla to first gps position
      Eigen::Vector3d ref_lla;
      ref_lla(0) = msg->latitude;
      ref_lla(1) = msg->longitude;
      ref_lla(2) = msg->altitude;
      ekf_.setRefLla(ref_lla);
    }

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
