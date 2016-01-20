#ifndef MOCAP_FILTER_H
#define MOCAP_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <relative_nav/FilterState.h>
#include <tf/tf.h>
#include <deque>
#include <mocap_filter/message.h>
#include <relative_nav/eigen.h>
#include <eigen_conversions/eigen_msg.h>

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


#define NUM_STATES 9

#define G 10.6160

namespace mocap_filter
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
  ros::Publisher estimate_pub_;
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

  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q_;
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> P_;
  Eigen::Matrix<double, NUM_STATES, 1> x_hat_;

  ros::Time previous_predict_time_;
  double prev_p_, prev_q_, prev_r_;
  double p_, q_, r_, az_, ax_, ay_;
  double alpha_;
  bool first_mocap_, flying_;

  // Functions
  void mocapCallback(const geometry_msgs::TransformStamped msg);
  void imuCallback(const sensor_msgs::Imu msg);
  void predictStep();
  void updateStep();
  void updateIMU(sensor_msgs::Imu msg);
  void updateMocap(geometry_msgs::TransformStamped msg);
  void initializeX(geometry_msgs::TransformStamped msg);
  void predictTimerCallback(const ros::TimerEvent& event);
  void publishTimerCallback(const ros::TimerEvent& event);
  void publishEstimate();
  double LPF(double yn, double un);
  Eigen::Matrix<double, NUM_STATES, 1> f(const Eigen::Matrix<double, NUM_STATES, 1> x);
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> dfdx(const Eigen::Matrix<double, NUM_STATES, 1> x);

};

} // namespace mocap_filter

#endif // mocapFilter_H
