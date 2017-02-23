#ifndef EKF_H
#define EKF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <deque>
#include <lib/eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <fcu_common/GPS.h>

// state numbers
#define PN 0
#define PE 1
#define PD 2
#define U 3
#define V 4
#define W 5
#define PHI 6
#define THETA 7
#define PSI 8
#define AX 9
#define AY 10
#define AZ 11
#define BX 12
#define BY 13
#define BZ 14

#define NUM_STATES 15

#define G 9.80

namespace ekf
{

class mocapFilter
{

public:

  mocapFilter();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber mocap_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;
  ros::Publisher estimate_pub_;
  ros::Publisher bias_pub_;
  ros::Publisher is_flying_pub_;
  ros::Timer predict_timer_;
  ros::Timer publish_timer_;

  // Parameters
  double inner_loop_rate_;
  double publish_rate_;
  double mass_;
  struct inertia{
    double x;
    double y;
    double z;
  } J_;


  // Local Variables
  Eigen::Matrix<double, 3, 3> R_IMU_;
  Eigen::Matrix<double, 6, 6> R_Mocap_;
  Eigen::Matrix<double, 3, 3> R_GPS_;

  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q_;
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> P_;
  Eigen::Matrix<double, NUM_STATES, 1> x_hat_;

  ros::Time current_time_;
  ros::Time previous_predict_time_;
  double prev_p_, prev_q_, prev_r_;
  double gx_, gy_, gz_, az_, ax_, ay_;
  double lat_, lon_, alt_, vg_, chi_;
  double lat0_, lon0_, alt0_, gps_count_;
  double alpha_;
  bool flying_;

  // Functions
  void mocapCallback(const geometry_msgs::TransformStamped msg);
  void imuCallback(const sensor_msgs::Imu msg);
  void gpsCallback(const fcu_common::GPS msg);
  void predictStep();
  void updateStep();
  void updateIMU(sensor_msgs::Imu msg);
  void updateMocap(geometry_msgs::TransformStamped msg);
  void updateGPS(fcu_common::GPS msg);
  void initializeX(geometry_msgs::TransformStamped msg);
  void publishEstimate();
  void GPS_to_m(double* dlat, double* dlon, double* dx, double* dy);
  double LPF(double yn, double un);
  Eigen::Matrix<double, NUM_STATES, 1> f(const Eigen::Matrix<double, NUM_STATES, 1> x);
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> dfdx(const Eigen::Matrix<double, NUM_STATES, 1> x);

};

} // namespace ekf

#endif // EKF_H
