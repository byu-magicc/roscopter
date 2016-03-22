#include "mono_vo/mono_vo.h"

namespace mono_vo
{

monoVO::monoVO() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~/mono_vo"))
{
  // Get Parameters from Server
  // arguments are "name", "variable to put the value into", "default value"
  nh_private_.param<int>("max_corners", max_corners_, 70);

  // Setup publishers and subscribers
  camera_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &monoVO::cameraCallback, this);
  estimate_sub_ = nh_.subscribe("estimate", 1, &monoVO::estimateCallback, this);
  velocity_pub_ = nh_.advertise<geometry_msgs::Vector3>("velocity", 1);
  return;
  
  
  // Initialize Filters and other class variables
}


void monoVO::cameraCallback(const sensor_msgs::ImageConstPtr msg)
{
  // Convert ROS message to opencv Mat
  cv_bridge::CvImageConstPtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Mat src = cv_ptr->image;

  // Show image to show it's working
  imshow("image", src);
  waitKey(33);

  // At this point, src holds the color image you should use for the rest
  // of the main processing loop.

  // The current state can be found in the current_state_ data member
  // positions/orientations are in pose, angular and linear velocity
  // estimates are in the twist data member.
  // covariances are also available (from the ekf)
  double phi = current_state_.pose.pose.orientation.x;
  double theta = current_state_.pose.pose.orientation.y;
  double psi = current_state_.pose.pose.orientation.z;
  double p = current_state_.twist.twist.angular.x;
  double q = current_state_.twist.twist.angular.y;
  double r = current_state_.twist.twist.angular.z;

  //Store the resulting measurement in the geometry_msgs::Vector3 velocity_measurement.
  velocity_measurement_.x = 0.0;
  velocity_measurement_.y = 0.0;
  velocity_measurement_.z = 0.0;

  // publish the velocity measurement whenever you're finished processing
  publishVelocity();
  return;
}

void monoVO::estimateCallback(const nav_msgs::Odometry msg)
{
  current_state_ = msg;
  return;
}

void monoVO::publishVelocity()
{
  velocity_pub_.publish(velocity_measurement_);
}

} // namespace ekf



