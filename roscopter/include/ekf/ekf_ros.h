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


#pragma once


#include "ekf.h"

#include <mutex>
#include <deque>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/GNSS.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#ifdef INERTIAL_SENSE
#include "inertial_sense/GPS.h"
#endif

#ifdef F9P_GNSS
#include "blah"
#endif

namespace roscopter::ekf
{
class EKF_ROS
{
public:

  EKF_ROS();
  ~EKF_ROS();
  void init(const std::string& param_file);
  void initROS();

  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void gnssCallback(const rosflight_msgs::GNSSConstPtr& msg);
  void mocapCallback(const ros::Time& time, const xform::Xformd &z);
  void statusCallback(const rosflight_msgs::StatusConstPtr& msg);

#ifdef INERTIAL_SENSE
  void gnssCallbackInertialSense(const inertial_sense::GPSConstPtr& msg);
#endif

#ifdef F9P_GNSS
  void gnssCallbackF9P(const f9p_gnss::GPSConstPtr& msg);
#endif


private:
  EKF ekf_;

  ros::Time last_imu_update_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber gnss_sub_;
  ros::Subscriber status_sub_;

  ros::Publisher odometry_pub_;
  ros::Publisher bias_pub_;
  ros::Publisher is_flying_pub_;
  nav_msgs::Odometry odom_msg_;

#ifdef INERTIAL_SENSE
  ros::Subscriber is_gnss_sub_;
#endif

  std::mutex ekf_mtx_;

  bool imu_init_ = false;
  bool truth_init_ = false;

  bool use_odom_;
  bool use_pose_;

  bool is_flying_ = false;
  bool armed_ = false;
  ros::Time time_took_off_;
  ros::Time start_time_;

  Vector6d imu_;

  Matrix6d imu_R_;
  Matrix6d mocap_R_;
  Eigen::Matrix<double, 1, 1> alt_R_;
};

}
