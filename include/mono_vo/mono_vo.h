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
#include <cmath>

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

  // Parameters (Things we will want to tweak)
  // put an underscore after class variables,
  // local variables do not have underscores following them
  // all variables are lowercase, (except acronyms) functions are camelCase (ROS standard)
  // structs and classes are also camelCase
  struct goodFeaturesToTrackParameters{
    int maxCorners;
    double qualityLevel;
    double minDist;
    int blockSize;
  }GFTT_params_;

  struct calcOpticalFlowPyrLKParams{
    int winSize;
    int maxLevel;
    int iters;
    double accuracy;
  }LK_params_;

  struct findHomographyParams{
    double ransacReprojThreshold;
    int maxIters;
    double confidence;
  }FH_params_;

  struct lineCircleParams{
    int radius;
    int thickness;
  }LC_params_;

  // Class Variables (for memory between loops and functions)
  nav_msgs::Odometry current_state_;
  geometry_msgs::Vector3 velocity_measurement_;
  Mat srcPrev_, optFlowVel_, N_;
  vector<Point2f> corners_, cornersLK_, cornersPrev_;
  bool no_normal_estimate_;

  // Functions (feel free to add more helper functions if needed)
  void cameraCallback(const sensor_msgs::ImageConstPtr msg);
  void estimateCallback(const nav_msgs::Odometry msg);
  void publishVelocity();
};

} // namespace ekf

#endif // MONO_VO_H
