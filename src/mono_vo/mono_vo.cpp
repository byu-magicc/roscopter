#include "mono_vo/mono_vo.h"

namespace mono_vo
{

monoVO::monoVO() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~/mono_vo"))
{
  // Get Parameters from Server
  string camera_parameter_filename;
  nh_private_.param<string>("camera_parameter_filename", camera_parameter_filename, 400);

  // Initialize Camera
  FileStorage file(camera_parameter_filename, FileStorage::WRITE);
  file["I"] >> I_;
  file["D"] >> D_;

  // Setup publishers and subscribers
  camera_sub_ = nh_.subscribe("camera", 1, &monoVO::cameraCallback, this);
  estimate_sub_ = nh_.subscribe("estimate", 1, &monoVO::estimateCallback, this);
  velocity_pub_ = nh_.advertise<geometry_msgs::Vector3>("velocity", 1);
  return;
}


void monoVO::cameraCallback(const sensor_msgs::Image msg)
{
  // Do stuff
  return;
}

void monoVO::estimateCallback(const nav_msgs::Odometry)
{
  // Save off estimate for vision processing
  return;
}

void monoVO::publishVelocity()
{
  velocity_pub_.publish(velocity_measurement_);
}

} // namespace ekf


