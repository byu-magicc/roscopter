#include <ros/ros.h>
#include "mono_vo/mono_vo.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mono_vo");
  ros::NodeHandle nh;

  mono_vo::monoVO Thing;

  ros::spin();

  return 0;
}
