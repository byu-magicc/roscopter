#include <ros/ros.h>
#include "mocap_filter/mocap_filter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mocap_filter_node");
  ros::NodeHandle nh;

  mocap_filter::mocapFilter Thing;

  ros::spin();

  return 0;
}
