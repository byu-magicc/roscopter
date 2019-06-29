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
  pose_sub_ = nh_.subscribe("pose", 10, &EKF_ROS::poseTruthCallback, this);
  transform_sub_ = nh_.subscribe("transform", 10, &EKF_ROS::transformTruthCallback, this);
  gnss_sub_ = nh_.subscribe("gnss", 10, &EKF_ROS::gnssCallback, this);
}

void EKF_ROS::init(const std::string &param_file)
{

}


}
