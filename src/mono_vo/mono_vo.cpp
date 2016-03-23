#include "mono_vo/mono_vo.h"
#include <iostream>

namespace mono_vo
{

monoVO::monoVO() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~/mono_vo"))
{
  // Get Parameters from Server
  // arguments are "name", "variable to put the value into", "default value"
  nh_private_.param<int>("GFTT_maxCorners", GFTT_params_.max_corners, 70);
  nh_private_.param<double>("GFTT_qualityLevel", GFTT_params_.quality_level, 0.01);
  nh_private_.param<double>("GFTT_minDist", GFTT_params_.min_dist, 5);
  nh_private_.param<int>("GFTT_blockSize", GFTT_params_.block_size, 3);
  nh_private_.param<int>("LK_winSize", LK_params_.win_size, 31);
  nh_private_.param<int>("LK_maxLevel", LK_params_.win_level, 3);
  nh_private_.param<int>("LK_iterations", LK_params_.iters, 30);
  nh_private_.param<double>("LK_accuracy", LK_params_.accuracy, 0.01);
  nh_private_.param<int>("LC_radius", LC_params_.radius, 5);
  nh_private_.param<int>("LC_thickness", LC_params_.thickness, 3);
  nh_private_.param<double>("FH_ransacReprojThreshold", FH_params_.rancsace_reproj_threshold, 3);
  nh_private_.param<int>("FH_maxIters", FH_params_.max_iters, 2000);
  nh_private_.param<double>("FH_confidence", FH_params_.confidence, 0.995);
  nh_private_.param<bool>("no_normal_estimate", no_normal_estimate_, false);

  // Setup publishers and subscribers
  camera_sub_ = nh_.subscribe("/image_mono", 1, &monoVO::cameraCallback, this);
  estimate_sub_ = nh_.subscribe("/shredder/ground_truth/odometry", 1, &monoVO::estimateCallback, this);
  velocity_pub_ = nh_.advertise<geometry_msgs::Vector3>("velocity", 1);
  flow_image_pub_ = nh_.advertise<sensor_msgs::Image>("optical_flow", 1);

  // Initialize Filters and other class variables
  optical_flow_velocity_ = (Mat_<double>(3,1) << 0, 0, 0);
  return;
}


void monoVO::cameraCallback(const sensor_msgs::ImageConstPtr msg)
{
  // Convert ROS message to opencv Mat
  cv_bridge::CvImageConstPtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Mat src = cv_ptr->image;

  // At this point, src holds the gray, rectified image you should use for the rest
  // of the main processing loop.

  // The current state can be found in the current_state_ data member
  // positions/orientations are in pose, angular and linear velocity
  // estimates are in the twist data member.
  // covariances are also available (from the ekf)
  double pd = current_state_.pose.pose.position.z;
  double phi = current_state_.pose.pose.orientation.x;
  double theta = current_state_.pose.pose.orientation.y;
  double psi = current_state_.pose.pose.orientation.z;
  double p = current_state_.twist.twist.angular.x;
  double q = current_state_.twist.twist.angular.y;
  double r = current_state_.twist.twist.angular.z;

  /*-------------------------------------------------------------------------------------
    ----------------------------------- Optical Flow ------------------------------------
    -------------------------------------------------------------------------------------*/

  // build angular velocity skew symmetric matrix
  Mat omega_hat = (Mat_<double>(3,3) <<   0, -r,  q,
                   r,  0, -p,
                   -q,  p,  0 );


  // compute ground normal (w.r.t. camera frame) from gravity vector
  if (no_normal_estimate_ == false) {
    Mat N_inertial = (Mat_<double>(3,1) <<  0, 0, -1);
    Mat R_v1_to_v2 = (Mat_<double>(3,3) <<  cos(theta),  0, -sin(theta),
                      0     ,  1,      0     ,
                      sin(theta),  0,  cos(theta) );
    Mat R_v2_to_b = (Mat_<double>(3,3) <<  1,      0   ,      0   ,
                     0,  cos(phi),  sin(phi),
                     0, -sin(phi),  cos(phi) );
    Mat R_b_to_c = (Mat_<double>(3,3) <<  0,  1,  0,
                    -1,  0,  0,
                    0,  0,  1 );
    N_ = R_b_to_c*R_v2_to_b*R_v1_to_v2*N_inertial; // rotate inertial into the camera frame
  }

  // find good features
  Mat mask; // just to run function but not actually needed/used
  goodFeaturesToTrack(src, corners_, GFTT_params_.max_corners, GFTT_params_.quality_level, GFTT_params_.min_dist, mask, GFTT_params_.block_size, false, 0.04);

  // compute optical flow after 2 sets of data is stored
  if (!prev_src_.empty()) {
    // run Lucas-Kanade to match features to previous frame
    vector<uchar> status; // stores inlier indicators
    vector<float> err; // only needed to run function, not used
    corners_LK_ = corners_; // use these for LK to not lose original corners
    calcOpticalFlowPyrLK( prev_src_, src, prev_corners_, corners_LK_, status, err, Size(LK_params_.win_size,LK_params_.win_size), LK_params_.win_level, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, LK_params_.iters, LK_params_.accuracy), 0, 1e-4 );

    // get matching features
    vector<Point2f> cornersPrevGood, cornersGood;
    for (int i = 0; i < status.size(); i++) {
      int prev_x(prev_corners_[i].x), prev_y(prev_corners_[i].y);
      int new_x(corners_LK_[i].x), new_y(corners_LK_[i].y);
      double diff = pow((prev_x-new_x)*(prev_x-new_x) + (prev_y-new_y)*(prev_y-new_y),0.5);
      if ( status[i] && diff < 20) {
        cornersPrevGood.push_back(prev_corners_[i]);
        cornersGood.push_back(corners_LK_[i]);
      }
    }
    // compute homography matrix
    Mat mask; // just to run function but not actually needed/used
    Mat H = findHomography(cornersPrevGood, cornersGood, CV_RANSAC, FH_params_.rancsace_reproj_threshold, mask, FH_params_.max_iters, FH_params_.confidence);

    // remove angular velocity from homography
    Mat H_no_omega = H - omega_hat;

    // DO THIS IF NO GROUND NORMAL VECTOR ESTIMATE IS AVAILABLE
    // decompose H_no_omega into velocity and normal components
    if (no_normal_estimate_ == true) {
      Mat Sigma, U, V, Vt;
      SVD::compute(H_no_omega, Sigma, U, Vt);
      N_ = (Mat_<double>(3,1) << Vt.at<double>(0,0), Vt.at<double>(0,1), Vt.at<double>(0,2))*(-1);
    }

    // compute estimated velocity vector
    Mat v_over_d = H_no_omega*N_;
    optical_flow_velocity_ = v_over_d*(-pd);

    Mat prev_src_color;
    cvtColor(prev_src_, prev_src_color, COLOR_GRAY2BGR);

    // draw matching features
    for (int i = 0; i < status.size(); i++) {
      circle(prev_src_color, cornersPrevGood[i], LC_params_.radius, Scalar(0,255,0), LC_params_.thickness, LINE_AA);
      line(prev_src_color, cornersPrevGood[i], cornersGood[i], Scalar(0,0,255), LC_params_.thickness, LINE_AA);
    }

    // publish the image
    cv_bridge::CvImage out_msg;
    out_msg.header = cv_ptr->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = prev_src_color;

    flow_image_pub_.publish(out_msg.toImageMsg());

  }else{
    ROS_INFO("srcPrev is empty");
  }

  // store previous frame and corners
  prev_src_ = src.clone();
  prev_corners_ = corners_;


  /*-----------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------*/


  //Store the resulting measurement in the geometry_msgs::Vector3 velocity_measurement.
  velocity_measurement_.x = optical_flow_velocity_.at<double>(0,0);
  velocity_measurement_.y = optical_flow_velocity_.at<double>(1,0);
  velocity_measurement_.z = optical_flow_velocity_.at<double>(2,0);

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

} // namespace mono_vo



