#include <ros/ros.h>
#include "ekf/ekf_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "landing_estimator");

  roscopter::ekf::EKF_ROS estimator;

  ros::spin();

  return 0;
}

