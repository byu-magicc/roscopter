#include "ekf.h"

#include <mutex>
#include <deque>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <rosflight_msgs/Status.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>


using namespace Eigen;
using namespace quat;

typedef Matrix<double, 1, 1> Matrix1d;
typedef Matrix<double, 6, 1> Vector6d;


namespace roscopter {
class EKF_ROS
{
public:

  EKF_ROS();
  ~EKF_ROS();
  void pose_truth_callback(const geometry_msgs::PoseStampedConstPtr &msg);
  void transform_truth_callback(const geometry_msgs::TransformStampedConstPtr &msg);
  void truth_callback(Vector3d &z_pos_, Vector4d &z_att_, ros::Time time);
  void imu_callback(const sensor_msgs::ImuConstPtr& msg);
  void status_callback(const rosflight_msgs::StatusConstPtr& msg);
  EKF ekf_;
  
private:

  ros::Time last_imu_update_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber imu_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber transform_sub_;
  ros::Subscriber status_sub_;
  ros::Publisher odometry_pub_;
  ros::Publisher bias_pub_;
  ros::Publisher is_flying_pub_;
  nav_msgs::Odometry odom_msg_;

  std::mutex ekf_mtx_;

  bool imu_init_ = false;
  bool truth_init_ = false;
  
  bool use_drag_term_ = false;
  bool is_flying_ = false;
  bool armed_ = false;
  bool use_truth_;
  bool use_acc_;
  bool use_imu_att_;
  bool use_alt_;
  double IMU_LPF_;
  double truth_LPF_;
  ros::Time time_took_off_;
  ros::Time start_time_;

  Vector6d imu_;
  Vector3d init_pos_;
  Vector3d truth_pos_;
  Quatd truth_att_;
  
  uVector u_;
  Vector2d z_acc_drag_;
  Vector3d z_acc_grav_;
  Vector4d z_att_;
  Matrix1d z_alt_;
  Vector3d z_pos_;
  sensor_msgs::Imu bias_msg_;
    
  Quatd q_b_IMU_;
  Quatd q_I_truth_;
  
  Matrix2d acc_R_drag_;
  Matrix3d acc_R_grav_;
  Matrix3d att_R_;
  Matrix<double, 1, 1> alt_R_;
  Matrix3d pos_R_;
  Matrix3d vel_R_;

  // Mocap Time Sync
  int time_sync_samples_ = 0;
  int num_sync_time_samples_;
  double min_offset_ = INFINITY;
  bool time_sync_complete_ = false;
  ros::Time imu_time_;
};

}




