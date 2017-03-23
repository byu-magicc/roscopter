#include <ros/ros.h>
#include "mekf/mekf.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mekf_node");
  mekf::kalmanFilter Thing;
  ros::spin();
  return 0;
}
