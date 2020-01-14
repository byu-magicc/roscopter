#include <ros/ros.h>
#include "ekf/ekf_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  roscopter::ekf::EKF_ROS estimator;
  estimator.initROS();

  ros::spin();

  return 0;
}

