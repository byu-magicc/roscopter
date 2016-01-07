#include "mocap_filter/mocap_filter.h"

namespace mocap_filter
{

mocapFilter::mocapFilter() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // retrieve params
  nh_private_.param<double>("inner_loop_rate", inner_loop_rate_, 400);
  relative_nav::importMatrixFromParamServer(nh_private_, x_hat_, "x0");
  relative_nav::importMatrixFromParamServer(nh_private_, P_, "P0");
  relative_nav::importMatrixFromParamServer(nh_private_, Q_, "Q0");
  relative_nav::importMatrixFromParamServer(nh_private_, R_IMU_, "R_IMU");
  relative_nav::importMatrixFromParamServer(nh_private_, R_Mocap_, "R_Mocap");


  // Setup publishers and subscribers
  imu_sub_ = nh_.subscribe("imu/data", 1, &mocapFilter::imuCallback, this);
  mocap_sub_ = nh_.subscribe("mocap", 1, &mocapFilter::mocapCallback, this);
  estimate_pub_ = nh_.advertise<geometry_msgs::PoseWithCovariance>("truth", 1);
}


void mocapFilter::imuCallback(const sensor_msgs::Imu msg)
{
  ImuMessage imu_message(msg);
  message_queue_.push_back(imu_message);
}


void mocapFilter::mocapCallback(const geometry_msgs::PoseStamped msg)
{
  PoseMessage pose_message(msg);
  message_queue_.push_back(pose_message);
}


void mocapFilter::predictStep()
{
  ros::Time now = ros::Time::now();
  double dt = (previous_predict_time_ - now).toSec();
  x_hat_ = x_hat_ + dt*f(x_hat_);
  previous_predict_time_ = now;
}


Eigen::Matrix<double, 12, 1> mocapFilter::f(const Eigen::Matrix<double, 12, 1> x)
{
  // this implementation ignores the inputs, u.  If these were given the filter
  // would do better
  double phi(x(PHI)), theta(x(THETA)), psi(x(PSI));
  double u(x(U)), v(x(V)), w(x(W));
  double p(x(P)), q(x(Q)), r(x(R));

  double ct = cos(theta);
  double cs = cos(psi);
  double cp = cos(phi);
  double st = sin(theta);
  double ss = sin(psi);
  double sp = sin(phi);
  double tt = tan(theta);

  Eigen::Matrix<double, 3, 1> xdot1, xdot2, xdot3, xdot4;
  xdot1 << ct*cs*u  + sp*st*cs-cp*ss*v + cp*st*cs+sp*ss*w,
          cp*ss*u  + sp*st*ss+cp*cs*v + cp*st*ss-sp*cs*w,
          st*u     - sp*ct*v          - cp*ct*w;
  xdot2 << r*v - q*w,
           p*w - r*u,
           q*u - p*v;
  xdot3 << p + sp*tt*q + cp*tt*r,
                  cp*q    - sp*r,
                  sp/ct*q + cp/ct*r;
  xdot4 << (J_.y-J_.z)/J_.z*q*r,
           (J_.z-J_.x)/J_.y*p*r,
           (J_.x-J_.y)/J_.z*p*q;
}




} // namespace mocap_filter

