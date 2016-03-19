#ifndef MONO_VO_H
#define MONO_VO_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace mono_vo
{

class monoVO
{

public:

  monoVO();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber camera_sub_;
  ros::Subscriber estimate_sub_;
  ros::Publisher velocity_pub_;

  // Parameters
  Mat I_, D_; // intrinsic parameters for camera

  nav_msgs::Odometry current_state_;
  geometry_msgs::Vector3 velocity_measurement_;

  // Functions
  void cameraCallback(const sensor_msgs::ImageConstPtr msg);
  void estimateCallback(const nav_msgs::Odometry msg);
  void publishVelocity();
};

} // namespace ekf

#endif // MONO_VO_H
