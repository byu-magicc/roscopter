/*
 * Copyright 2016 James Jackson, Brigham Young University, Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROSCOPTER_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H
#define ROSCOPTER_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H

#include <stdio.h>

#include <vector>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Eigen>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/Attitude.h>
#include <rosflight_utils/simple_pid.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include "roscopter_sim/common.h"
#include "roscopter_sim/gz_compat.h"

namespace gazebo {


class MultiRotorForcesAndMoments : public ModelPlugin {
public:
  MultiRotorForcesAndMoments();

  ~MultiRotorForcesAndMoments();

  void InitializeParams();
  void SendForces();

protected:
  void UpdateForcesAndMoments();
  void Reset();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  std::string command_topic_;
  std::string wind_topic_;
  std::string attitude_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string parent_frame_id_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::EntityPtr parent_link_;
  event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

  // physical parameters
  double linear_mu_;
  double angular_mu_;
  struct GE_constants{
    double a;
    double b;
    double c;
    double d;
    double e;
  } ground_effect_;
  double mass_; // for static thrust offset when in altitude mode (kg)

  // Container for an Actuator
  struct Actuator{
    double max;
    double tau_up;
    double tau_down;
  };

  // Struct of Actuators
  // This organizes the physical limitations of the abstract torques and Force
  struct Actuators{
    Actuator l;
    Actuator m;
    Actuator n;
    Actuator F;
  } actuators_;

  // container for forces
  struct ForcesAndTorques{
    double Fx;
    double Fy;
    double Fz;
    double l;
    double m;
    double n;
  } applied_forces_, actual_forces_, desired_forces_;

  // container for PID controller
  rosflight_utils::SimplePID roll_controller_;
  rosflight_utils::SimplePID pitch_controller_;
  rosflight_utils::SimplePID yaw_controller_;
  rosflight_utils::SimplePID alt_controller_;

  rosflight_msgs::Command command_;

  // Time Counters
  double sampling_time_;
  double prev_sim_time_;

  ros::NodeHandle* nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_sub_;
  ros::Publisher attitude_pub_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void WindCallback(const geometry_msgs::Vector3& wind);
  void CommandCallback(const rosflight_msgs::Command msg);
  void ComputeControl(void);
  double sat(double x, double max, double min);
  double max(double x, double y);

  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
  GazeboVector W_wind_;
};
}

#endif // ROSCOPTER_SIM_MULTIROTOR_FORCES_AND_MOMENTS_H
