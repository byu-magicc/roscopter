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

  imu_sub_ = nh_.subscribe("imu", 100, &EKF_ROS::imuCallback, this);
  pose_sub_ = nh_.subscribe("pose", 10, &EKF_ROS::poseCallback, this);
  odom_sub_ = nh_.subscribe("reference", 10, &EKF_ROS::odomCallback, this);
  gnss_sub_ = nh_.subscribe("gnss", 10, &EKF_ROS::gnssCallback, this);

  init(parameter_filename);
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

void EKF_ROS::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  if (start_time_.sec == 0)
    start_time_ = msg->header.stamp;

  Vector6d z;
  z << msg->linear_acceleration.x,
       msg->linear_acceleration.y,
       msg->linear_acceleration.z,
       msg->angular_velocity.x,
       msg->angular_velocity.y,
       msg->angular_velocity.z;

  double t = (msg->header.stamp - start_time_).toSec();
  ekf_.imuCallback(t, z, imu_R_);
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




}
