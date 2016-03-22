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

  // Parameters (Things we will want to tweak)
  // put an underscore after class variables,
  // local variables do not have underscores following them
  // all variables are lowercase, (except acronyms) functions are camelCase (ROS standard)
  // structs and classes are also camelCase
  struct goodFeaturesToTrackParameters{
    int max_corners;
    double quality_level;
    double min_dist;
    int block_size;
  }GFTT_params_;

  struct calcOpticalFlowPyrLKParams{
    Size winSize;
    int maxLevel;
    TermCriteria criteria;
  }LK_params_;

  struct findHomographyParams{
    int method;
    double ransac_reproj_threshold;
    int max_iters;
    double confidence;
  };

  struct lineCircleParams{
    int radius;
    int thickness;
    Scalar lineColor;
    Scalar circColor;
  };

  // Class Variables (for memory between loops and functions)
  nav_msgs::Odometry current_state_;
  geometry_msgs::Vector3 velocity_measurement_;
  vector<uchar> status_; // point mask
  vector<float> err_; // (NOT SURE)
  int wait_time_; // milliseconds
  Mat frame_prev_;
  vector<Point2f> corners_, corners_LK_, corners_prev_;

  // Functions (feel free to add more helper functions if needed)
  void cameraCallback(const sensor_msgs::ImageConstPtr msg);
  void estimateCallback(const nav_msgs::Odometry msg);
  void publishVelocity();
};

} // namespace ekf

#endif // MONO_VO_H
