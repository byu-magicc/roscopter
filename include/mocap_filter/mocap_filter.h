#ifndef MOCAP_FILTER_H
#define MOCAP_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread.hpp>
#include <deque>
#include <mocap_filter/message.h>
#include <relative_nav/eigen.h>

// state numbers
#define PN 0
#define PE 1
#define PD 2
#define PHI 3
#define THETA 4
#define PSI 5
#define U 6
#define V 7
#define W 8
#define P 9
#define Q 10
#define R 11

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

  // Parameters
  double inner_loop_rate_;
  double mass_;
  struct inertia{
    double x;
    double y;
    double z;
  } J_;

  // Local Variables
  Eigen::Matrix<double, 6, 1> R_IMU_;
  Eigen::Matrix<double, 6, 1> R_Mocap_;
  Eigen::Matrix<double, 12, 6> H_IMU_;
  Eigen::Matrix<double, 12, 6> H_Mocap_;

  Eigen::Matrix<double, 12, 12> Q_;
  Eigen::Matrix<double, 12, 12> P_;
  Eigen::Matrix<double, 12, 1> x_hat_;

  ros::Time previous_predict_time_;

  std::deque<Message> message_queue_;

  // Functions
  void mocapCallback(const geometry_msgs::PoseStamped msg);
  void imuCallback(const sensor_msgs::Imu msg);
  void predictStep();
  void updateStep();
  Eigen::Matrix<double, 12, 1> f(const Eigen::Matrix<double, 12, 1> x);
};

} // namespace mocap_filter

#endif // mocapFilter_H
