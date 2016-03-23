#include "mono_vo/mono_vo.h"

namespace mono_vo
{

monoVO::monoVO() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~/mono_vo"))
{
  // Get Parameters from Server
  // arguments are "name", "variable to put the value into", "default value"
  nh_private_.param<int>("GFTT_maxCorners", GFTT_params_.maxCorners, 70);
  nh_private_.param<double>("GFTT_qualityLevel", GFTT_params_.qualityLevel, 0.01);
  nh_private_.param<double>("GFTT_minDist", GFTT_params_.minDist, 5);
  nh_private_.param<int>("GFTT_blockSize", GFTT_params_.blockSize, 3);
  nh_private_.param<int>("LK_winSize", LK_params_.winSize, 31);
  nh_private_.param<int>("LK_maxLevel", LK_params_.maxLevel, 3);
  nh_private_.param<int>("LK_iterations", LK_params_.iters, 30);
  nh_private_.param<double>("LK_accuracy", LK_params_.accuracy, 0.01);
  nh_private_.param<int>("LC_radius", LC_params_.radius, 2);
  nh_private_.param<int>("LC_thickness", LC_params_.thickness, 1);
  nh_private_.param<double>("FH_ransacReprojThreshold", FH_params_.ransacReprojThreshold, 3);
  nh_private_.param<int>("FH_maxIters", FH_params_.maxIters, 2000);
  nh_private_.param<double>("FH_confidence", FH_params_.confidence, 0.995);
  nh_private_.param<bool>("no_normal_estimate", no_normal_estimate_, false);

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
  goodFeaturesToTrack(src, corners_, GFTT_params_.maxCorners, GFTT_params_.qualityLevel, GFTT_params_.minDist, mask, GFTT_params_.blockSize, false, 0.04);

  // compute optical flow after 2 sets of data is stored
  if (!srcPrev_.empty()) {

    // run Lucas-Kanade to match features to previous frame
    vector<uchar> status; // stores inlier indicators
    vector<float> err; // only needed to run function, not used
    cornersLK_ = corners_; // use these for LK to not lose original corners
    calcOpticalFlowPyrLK( srcPrev_, src, cornersPrev_, cornersLK_, status, err, Size(LK_params_.winSize,LK_params_.winSize), LK_params_.maxLevel, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, LK_params_.iters, LK_params_.accuracy), 0, 1e-4 );

    // get matching features
    vector<Point2f> cornersPrevGood, cornersGood;
    for (int i = 0; i < status.size(); i++) {
        if ( status[i] ) {
            cornersPrevGood.push_back(cornersPrev_[i]);
            cornersGood.push_back(cornersLK_[i]);
        }
    }

    // compute homography matrix
    Mat mask; // just to run function but not actually needed/used
    Mat H = findHomography(cornersPrevGood, cornersGood, CV_RANSAC, FH_params_.ransacReprojThreshold, mask, FH_params_.maxIters, FH_params_.confidence);

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
    optFlowVel_ = v_over_d*(-pd);

    // draw matching features
    for (int i = 0; i < status.size(); i++) {
        circle(srcPrev_, cornersPrevGood[i], LC_params_.radius, Scalar(0,255,0), LC_params_.thickness, LINE_AA);
        line(srcPrev_, cornersPrevGood[i], cornersGood[i], Scalar(0,0,255), LC_params_.thickness, LINE_AA);
    }

    // show image
    imshow("srcPrev", srcPrev_);
    waitKey(33);
  }

  // store previous frame and corners
  srcPrev_ = src.clone();
  cornersPrev_ = corners_;
  

  /*-----------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------
    -----------------------------------------------------------------------------------------*/


  //Store the resulting measurement in the geometry_msgs::Vector3 velocity_measurement.
  velocity_measurement_.x = optFlowVel_.at<double>(0,0);
  velocity_measurement_.y = optFlowVel_.at<double>(1,0);
  velocity_measurement_.z = optFlowVel_.at<double>(2,0);

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



