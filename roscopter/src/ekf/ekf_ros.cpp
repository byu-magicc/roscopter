#include "ekf/ekf_ros.h"
#include "roscopter/eigen_helpers.h"

namespace roscopter {

EKF_ROS::EKF_ROS() :
  nh_private_("~")
{
  imu_sub_ = nh_.subscribe("imu", 500, &EKF_ROS::imu_callback, this);
  pose_sub_ = nh_.subscribe("truth/pose", 10, &EKF_ROS::pose_truth_callback, this);
  transform_sub_ = nh_.subscribe("truth/transform", 10, &EKF_ROS::transform_truth_callback, this);
  status_sub_ = nh_.subscribe("status", 1, &EKF_ROS::status_callback, this);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  bias_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/bias", 1);
  is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1, true);
  
  std::string log_directory, feature_mask;
  std::string default_log_folder = ros::package::getPath("roscopter") + "/logs/" + to_string(ros::Time::now().sec);
  nh_private_.param<std::string>("log_directory", log_directory, default_log_folder );
  nh_private_.param<std::string>("feature_mask", feature_mask, "");

  Eigen::Matrix<double, roscopter::EKF::xZ, 1> x0;
  Eigen::Matrix<double, roscopter::EKF::dxZ, 1> P0diag, Qxdiag, lambda;
  uVector Qudiag;
  Vector4d q_b_IMU, q_I_truth;
  Vector2d acc_r_drag_diag;
  Vector3d att_r_diag, pos_r_diag, vel_r_diag, acc_r_grav_diag;
  importMatrixFromParamServer(nh_private_, x0, "x0");
  importMatrixFromParamServer(nh_private_, P0diag, "P0");
  importMatrixFromParamServer(nh_private_, Qxdiag, "Qx");
  importMatrixFromParamServer(nh_private_, lambda, "lambda");
  importMatrixFromParamServer(nh_private_, Qudiag, "Qu");
  importMatrixFromParamServer(nh_private_, acc_r_drag_diag, "acc_R_drag");
  importMatrixFromParamServer(nh_private_, acc_r_grav_diag, "acc_R_grav");
  importMatrixFromParamServer(nh_private_, att_r_diag, "att_R");
  importMatrixFromParamServer(nh_private_, pos_r_diag, "pos_R");
  importMatrixFromParamServer(nh_private_, vel_r_diag, "vel_R");
  importMatrixFromParamServer(nh_private_, q_b_IMU, "q_b_IMU");
  importMatrixFromParamServer(nh_private_, q_I_truth, "q_I_truth");
  q_b_IMU_.arr_ = q_b_IMU;
  q_I_truth_.arr_ = q_I_truth;
  double alt_r, gating_threshold;
  bool partial_update;
  int cov_prop_skips;
  ROS_FATAL_COND(!nh_private_.getParam("alt_R", alt_r), "you need to specify the 'alt_R' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("imu_LPF", IMU_LPF_), "you need to specify the 'imu_LPF' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("truth_LPF", truth_LPF_), "you need to specify the 'truth_LPF' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("partial_update", partial_update), "you need to specify the 'partial_update' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("drag_term", use_drag_term_), "you need to specify the 'drag_term' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("cov_prop_skips", cov_prop_skips), "you need to specify the 'cov_prop_skips' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("sync_time_samples", num_sync_time_samples_), "you need to specify the 'sync_time_samples' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("gating_threshold", gating_threshold), "you need to specify the 'gating_threshold' parameter");
  
  ekf_.init(x0, P0diag, Qxdiag, lambda, Qudiag, log_directory, use_drag_term_, partial_update, cov_prop_skips, gating_threshold);
  
  is_flying_ = false; // Start out not flying
  ekf_.set_drag_term(false); // Start out not using the drag term
  
  // Initialize keyframe variables
  init_pos_.setZero();
  
  // Initialize the measurement noise covariance matrices
  acc_R_drag_ = acc_r_drag_diag.asDiagonal();
  acc_R_grav_ = acc_r_grav_diag.asDiagonal();
  att_R_ = att_r_diag.asDiagonal();
  pos_R_ = pos_r_diag.asDiagonal();
  vel_R_ = vel_r_diag.asDiagonal();
  alt_R_ << alt_r;
  
  // Turn on the specified measurements
  ROS_FATAL_COND(!nh_private_.getParam("use_truth", use_truth_), "you need to specify the 'use_truth' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("use_acc", use_acc_), "you need to specify the 'use_acc' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("use_imu_att", use_imu_att_), "you need to specify the 'use_imu_att' parameter");
  ROS_FATAL_COND(!nh_private_.getParam("use_alt", use_alt_), "you need to specify the 'use_alt' parameter");
  
  cout << "\nlog file: " << ((log_directory.compare("~") != 0) ? log_directory : "N/A") << "\n";
  cout << "\nMEASUREMENTS\tFEATURES\n==============================\n";
  cout << "truth: " << use_truth_ << "\t";
  cout << "partial update: " << partial_update << "\n";
  cout << "drag_term: " << use_drag_term_ << "\n";
  cout << "acc: " << use_acc_ << "\n";
  cout << "imu_att: " << use_imu_att_ << "\n";
  cout << "imu_alt: " << use_alt_ << "\n";

  
  // Wait for truth to initialize pose
  imu_init_ = false;
  truth_init_ = false;
  time_took_off_.fromSec(1e9);
  
  odom_msg_.header.frame_id = "body";
  
}

EKF_ROS::~EKF_ROS()
{}

void EKF_ROS::imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
  
  u_ << msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z,
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z;
  u_.block<3,1>(0,0) = q_b_IMU_.rotp(u_.block<3,1>(0,0));
  u_.block<3,1>(3,0) = q_b_IMU_.rotp(u_.block<3,1>(3,0));
  
  imu_ = (1. - IMU_LPF_) * u_ + IMU_LPF_ * imu_;
  imu_time_ = msg->header.stamp;

  if (use_truth_ && !truth_init_)
    return;
  else if (!imu_init_)
  {
    imu_ = u_;
    imu_init_ = true;
    start_time_ = msg->header.stamp;
    return;
  }
  if (use_truth_ && !time_sync_complete_)
    return;

  double t = (msg->header.stamp - start_time_).toSec();
  

  // Propagate filter
  ekf_mtx_.lock();
  ekf_.propagate_state(imu_, t);
  ekf_mtx_.unlock();

  
  // update accelerometer measurement
  ekf_mtx_.lock();
  if (ekf_.get_drag_term() == true)
  {
    z_acc_drag_ = imu_.block<2,1>(0, 0);
    ekf_.add_measurement(t, z_acc_drag_, roscopter::EKF::ACC, acc_R_drag_, use_acc_ && is_flying_);
  }
  else
  {
    z_acc_grav_ = imu_.block<3,1>(0, 0);
    double norm = z_acc_grav_.norm();
    if (norm < 9.80665 * 1.15 && norm > 9.80665 * 0.85)
      ekf_.add_measurement(t, z_acc_grav_, roscopter::EKF::ACC, acc_R_grav_, use_acc_);
  }
  ekf_mtx_.unlock();
  
  // update attitude measurement
  z_att_ << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  ekf_mtx_.lock();
  if (use_imu_att_)
    ekf_.add_measurement(t, z_att_, roscopter::EKF::ATT, att_R_, (use_truth_) ? true : use_imu_att_);
  ekf_mtx_.unlock();

  if (odometry_pub_.getNumSubscribers() > 0)
  {
    odom_msg_.header.stamp = msg->header.stamp;
    odom_msg_.pose.pose.position.x = ekf_.get_state()(roscopter::EKF::xPOS,0);
    odom_msg_.pose.pose.position.y = ekf_.get_state()(roscopter::EKF::xPOS+1,0);
    odom_msg_.pose.pose.position.z = ekf_.get_state()(roscopter::EKF::xPOS+2,0);
    odom_msg_.pose.pose.orientation.w = ekf_.get_state()(roscopter::EKF::xATT,0);
    odom_msg_.pose.pose.orientation.x = ekf_.get_state()(roscopter::EKF::xATT+1,0);
    odom_msg_.pose.pose.orientation.y = ekf_.get_state()(roscopter::EKF::xATT+2,0);
    odom_msg_.pose.pose.orientation.z = ekf_.get_state()(roscopter::EKF::xATT+3,0);
    odom_msg_.twist.twist.linear.x = ekf_.get_state()(roscopter::EKF::xVEL,0);
    odom_msg_.twist.twist.linear.y = ekf_.get_state()(roscopter::EKF::xVEL+1,0);
    odom_msg_.twist.twist.linear.z = ekf_.get_state()(roscopter::EKF::xVEL+2,0);
    odom_msg_.twist.twist.angular.x = imu_(3);
    odom_msg_.twist.twist.angular.y = imu_(4);
    odom_msg_.twist.twist.angular.z = imu_(5);
    odometry_pub_.publish(odom_msg_);
  }
  
  if (bias_pub_.getNumSubscribers() > 0)
  {
    bias_msg_.header = msg->header;
    bias_msg_.linear_acceleration.x = ekf_.get_state()(roscopter::EKF::xB_A, 0);
    bias_msg_.linear_acceleration.y = ekf_.get_state()(roscopter::EKF::xB_A+1, 0);
    bias_msg_.linear_acceleration.z = ekf_.get_state()(roscopter::EKF::xB_A+2, 0);
    bias_msg_.angular_velocity.x = ekf_.get_state()(roscopter::EKF::xB_G, 0);
    bias_msg_.angular_velocity.y = ekf_.get_state()(roscopter::EKF::xB_G+1, 0);
    bias_msg_.angular_velocity.z = ekf_.get_state()(roscopter::EKF::xB_G+2, 0);
    bias_pub_.publish(bias_msg_);
  }

}

void EKF_ROS::pose_truth_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  z_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  z_att_ << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
  truth_callback(z_pos_, z_att_, msg->header.stamp);
}

void EKF_ROS::transform_truth_callback(const geometry_msgs::TransformStampedConstPtr &msg)
{
  z_pos_ << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;
  z_att_ << msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z;
  truth_callback(z_pos_, z_att_, msg->header.stamp);
}

void EKF_ROS::truth_callback(Vector3d& z_pos, Vector4d& z_att, ros::Time time)
{
  static int counter = 0;
  if (counter++ < 2)
  {
    return;
  }
  else counter = 0;
  // Rotate measurements into the proper frame
  z_pos = q_I_truth_.rotp(z_pos);
  z_att.block<3,1>(1,0) = q_I_truth_.rotp(z_att.block<3,1>(1,0));
  
  // Make sure that the truth quaternion is the right sign (for plotting)
  if (sign(z_att(0,0)) != sign(ekf_.get_state()(roscopter::EKF::xATT, 0)))
  {
    z_att *= -1.0;
  }
  
  truth_pos_ = z_pos;

  double t = (time - start_time_).toSec();
  
  // Initialize Truth
  if (!truth_init_)
  {

    truth_att_ = Quatd(z_att);
    
    // Initialize the EKF to the origin in the Motion Capture Frame
    ekf_mtx_.lock();
    Matrix<double, roscopter::EKF::xZ, 1> x0 = ekf_.get_state().topRows(roscopter::EKF::xZ);
    x0.block<3,1>((int)roscopter::EKF::xPOS,0) = z_pos;
    x0.block<4,1>((int)roscopter::EKF::xATT,0) = z_att;
    ekf_.set_x0(x0);
    ekf_mtx_.unlock();

    init_pos_ = z_pos;
    truth_init_ = true;
  }

  if (!time_sync_complete_)
  {
    if (!imu_init_)
      return;
    if (++time_sync_samples_ < num_sync_time_samples_)
    {
      double offset = (time - imu_time_).toSec();
      min_offset_ = (fabs(offset) < min_offset_) ? offset : min_offset_;
    }
    else
    {
      time_sync_complete_ = true;
      if (!isfinite(min_offset_))
      {
        min_offset_ = 0;
      }
    }
    return;
  }

  t -= min_offset_;
  
  // Decide whether we are flying or not
  if (!is_flying_)
  {
    Vector3d error = truth_pos_ - init_pos_;
    // If we have moved 3 centimeters, then assume we are flying
    if (error.norm() >= 3e-2)
    {
      if (armed_)
      {
        is_flying_ = true;
        std_msgs::Bool is_flying_msg;
        is_flying_msg.data = true;
        is_flying_pub_.publish(is_flying_msg);
        time_took_off_ = time;
      }
      else
      {
        // reset the initial position - in case we lift up and move the MAV around by hand
        init_pos_ = truth_pos_;
      }
    }
  }
  
  if (time > time_took_off_ + ros::Duration(1.0))
  {
    // After 1 second of flying, turn on the drag term if we are supposed to
    if (use_drag_term_ == true && ekf_.get_drag_term() ==  false)
      ekf_.set_drag_term(true);
  }
  
  // Low-pass filter Attitude (use manifold)
  Quatd y_t(z_att);
  truth_att_ = y_t + (truth_LPF_ * (truth_att_ - y_t));
  
  z_alt_ << -z_pos(2,0);
  
  bool truth_active = (use_truth_ || !is_flying_);

  ekf_mtx_.lock();
  ekf_.add_measurement(t, z_pos, roscopter::EKF::POS, pos_R_, truth_active);
  ekf_.add_measurement(t, z_att, roscopter::EKF::ATT, att_R_, truth_active);
  ekf_.handle_measurements();
  ekf_mtx_.unlock();

  // Apply a zero-velocity update if we haven't started flying (helps accelerometer and gyro biases converge)
  if (!is_flying_)
  {
    ekf_mtx_.lock();
    Vector3d meas = Vector3d::Zero();
    Matrix3d R = Matrix3d::Identity() * 1e-8;
    ekf_.add_measurement(t, meas, roscopter::EKF::VEL, R, true);
    ekf_mtx_.unlock();
  }
  
  // Perform Altitude Measurement
  ekf_mtx_.lock();
  ekf_.add_measurement(t, z_alt_, roscopter::EKF::ALT, alt_R_, use_alt_ && !truth_active);
  ekf_mtx_.unlock();
}

void EKF_ROS::status_callback(const rosflight_msgs::StatusConstPtr &msg)
{
  if (armed_ && !msg->armed)
    is_flying_ = false;

  armed_ = msg->armed;
}

}


