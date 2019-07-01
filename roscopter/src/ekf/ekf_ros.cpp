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

namespace roscopter::ekf
{

EKF_ROS::EKF_ROS() :
  nh_(), nh_private_("~")
{}

void EKF_ROS::initROS()
{
  std::string roscopter_path = ros::package::getPath("roscopter");
  std::string parameter_filename = nh_private_.param<std::string>("param_filename", roscopter_path + "/params/ekf.yaml");

  imu_sub_ = nh_.subscribe("imu", 100, &EKF_ROS::imuCallback, this);
  pose_sub_ = nh_.subscribe("pose", 10, &EKF_ROS::poseCallback, this);
  transform_sub_ = nh_.subscribe("transform", 10, &EKF_ROS::transformCallback, this);
  gnss_sub_ = nh_.subscribe("gnss", 10, &EKF_ROS::gnssCallback, this);

  init(parameter_filename);
}

void EKF_ROS::init(const std::string &param_file)
{
  ekf_.load(param_file);

  get_yaml_diag("imu_R", param_file, imu_R_);
  get_yaml_diag("mocap_R", param_file, mocap_R_);
  get_yaml_diag("gnss_R", param_file, gnss_R_);
  get_yaml_diag("alt_R", param_file, alt_R_);
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

void EKF_ROS::transformCallback(const geometry_msgs::TransformStampedConstPtr &msg)
{
  xform::Xformd z;
  z.arr_ << msg->transform.translation.x,
            msg->transform.translation.y,
            msg->transform.translation.z,
            msg->transform.rotation.w,
            msg->transform.rotation.x,
            msg->transform.rotation.y,
            msg->transform.rotation.z;

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
  Vector6d R_diag_NED;
  R_diag_NED << msg->horizontal_accuracy,
                msg->horizontal_accuracy,
                msg->vertical_accuracy,
                msg->speed_accuracy,
                msg->speed_accuracy,
                msg->speed_accuracy;
  Matrix6d R_ecef =

  double t = (msg->header.stamp - start_time_).toSec();
  ekf_.gnssCallback();
}




}
