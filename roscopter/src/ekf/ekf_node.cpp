#include "ekf/ekf_ros.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vi_ekf_node");
  roscopter::EKF_ROS ekf;
  
  ros::NodeHandle nh("~");
  int num_threads = nh.param<int>("num_threads", 0);
  if (num_threads == 1)
  {
    ros::spin();
  }
  else
  {
    ros::AsyncSpinner spinner(num_threads);
    spinner.start();
    ros::waitForShutdown();
  }
  return 0;
}
