#include "ekf/ekf.h"

namespace ekf
{

mocapFilter::mocapFilter() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~/ekf"))
{
  // retrieve params
  nh_private_.param<double>("inner_loop_rate", inner_loop_rate_, 400);
  nh_private_.param<double>("publish_rate", publish_rate_, 400);
  nh_private_.param<double>("alpha", alpha_, 0.2);
  ros_copter::importMatrixFromParamServer(nh_private_, x_hat_, "x0");
  ros_copter::importMatrixFromParamServer(nh_private_, P_, "P0");
  ros_copter::importMatrixFromParamServer(nh_private_, Q_, "Q0");
  ros_copter::importMatrixFromParamServer(nh_private_, R_IMU_, "R_IMU");
  ros_copter::importMatrixFromParamServer(nh_private_, R_Mocap_, "R_Mocap");

  // Setup publishers and subscribers
  imu_sub_ = nh_.subscribe("imu/data", 1, &mocapFilter::imuCallback, this);
  mocap_sub_ = nh_.subscribe("mocap", 1, &mocapFilter::mocapCallback, this);
  estimate_pub_ = nh_.advertise<nav_msgs::Odometry>("estimate", 1);
  is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1);
  predict_timer_ = nh_.createTimer(ros::Duration(1.0/inner_loop_rate_), &mocapFilter::predictTimerCallback, this);
  publish_timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), &mocapFilter::publishTimerCallback, this);

  x_hat_.setZero();
  flying_ = false;
  ROS_INFO("Done");
  return;
}

void mocapFilter::publishTimerCallback(const ros::TimerEvent &event)
{
  publishEstimate();
  return;
}

void mocapFilter::predictTimerCallback(const ros::TimerEvent &event)
{
  if(flying_){
    predictStep();
  }
  return;
}


void mocapFilter::imuCallback(const sensor_msgs::Imu msg)
{
  if(!flying_){
    if(fabs(msg.linear_acceleration.z) > 11.0){
      ROS_WARN("Now flying");
      flying_ = true;
      std_msgs::Bool flying;
      flying.data = true;
      is_flying_pub_.publish(flying);
      previous_predict_time_ = ros::Time::now();
    }
  }
  if(flying_){
    updateIMU(msg);
  }
  return;
}


void mocapFilter::mocapCallback(const geometry_msgs::TransformStamped msg)
{
  if(!flying_){
    ROS_INFO_THROTTLE(1,"Not flying, but motion capture received, estimate is copy of mocap");
    initializeX(msg);
  }else{
    updateMocap(msg);
  }
  return;
}

void mocapFilter::initializeX(geometry_msgs::TransformStamped msg)
{
  ROS_INFO_ONCE("ekf initialized");
  tf::Transform measurement;
  double roll, pitch, yaw;
  tf::transformMsgToTF(msg.transform, measurement);
  tf::Matrix3x3(measurement.getRotation()).getRPY(roll, pitch, yaw);
  x_hat_(PN) =  measurement.getOrigin().getX(); // NWU to NED
  x_hat_(PE) = -measurement.getOrigin().getY();
  x_hat_(PD) = -measurement.getOrigin().getZ();
  x_hat_(PHI) = roll;
  x_hat_(THETA) = -pitch;
  x_hat_(PSI) = -yaw;
  return;
}


void mocapFilter::predictStep()
{
  ros::Time now = ros::Time::now();
  double dt = (now-previous_predict_time_).toSec();
  x_hat_ = x_hat_ + dt*f(x_hat_);
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> A = dfdx(x_hat_);
  P_ = P_ + dt*(A*P_ + P_*A.transpose() + Q_);
  previous_predict_time_ = now;
  return;
}


void mocapFilter::updateIMU(sensor_msgs::Imu msg)
{
  double phi(x_hat_(PHI)), theta(x_hat_(THETA)), psi(x_hat_(PSI));
  double ct = cos(theta);
  double cp = cos(phi);
  double st = sin(theta);
  double sp = sin(phi);
  ax_ = msg.linear_acceleration.x;
  ay_ = msg.linear_acceleration.y;
  az_ = msg.linear_acceleration.z;
  gx_ = msg.angular_velocity.x;
  gy_ = msg.angular_velocity.y;
  gz_ = msg.angular_velocity.z;

  Eigen::Matrix<double, 3, 1> y;
  y << ax_, ay_, az_;

  Eigen::Matrix<double, 3, NUM_STATES> C;
  C.setZero();
  C(0,THETA) = G*ct;
  C(0,AX) = -1;

  C(0,PHI) = -G*ct*cp;
  C(0,THETA) = -G*st*sp;
  C(0,AY) = -1;

  C(0,PHI) = G*ct*sp;
  C(0,THETA) = G*st*sp;
  C(0,AZ) = -1;

  Eigen::Matrix<double, NUM_STATES, 3> L;
  L.setZero();
  L = P_*C.transpose()*(R_IMU_ + C*P_*C.transpose()).inverse();
  P_ = (Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES) - L*C)*P_;
  x_hat_ = x_hat_ + L*(y - C*x_hat_);
}

void mocapFilter::updateMocap(geometry_msgs::TransformStamped msg)
{
  tf::Transform measurement;
  double roll, pitch, yaw;
  tf::transformMsgToTF(msg.transform, measurement);
  tf::Matrix3x3(measurement.getRotation()).getRPY(roll, pitch, yaw);
  Eigen::Matrix<double, 6, 1> y;
  y << measurement.getOrigin().getX(), // NWU to NED
       -measurement.getOrigin().getY(),
       -measurement.getOrigin().getZ(),
       roll, -pitch, -yaw;
  Eigen::Matrix<double, 6, NUM_STATES> C = Eigen::Matrix<double, 6, NUM_STATES>::Zero();
  C(0,PN) = 1;
  C(1,PE) = 1;
  C(2,PD) = 1;
  C(3,PHI) = 1;
  C(4,THETA) = 1;
  C(5,PSI) = 1;
  Eigen::Matrix<double, NUM_STATES, 6> L;
  L.setZero();
  L = P_*C.transpose()*(R_Mocap_ + C*P_*C.transpose()).inverse();
  P_ = (Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES) - L*C)*P_;
  x_hat_ = x_hat_ + L*(y - C*x_hat_);
}


Eigen::Matrix<double, NUM_STATES, 1> mocapFilter::f(const Eigen::Matrix<double, NUM_STATES, 1> x)
{
  double u(x(U)), v(x(V)), w(x(W));
  double phi(x(PHI)), theta(x(THETA)), psi(x(PSI));
  double alpha_z(x(AZ));
  double bx(x(BX)), by(x(BY)), bz(x(BZ));

  double ct = cos(theta);
  double cs = cos(psi);
  double cp = cos(phi);
  double st = sin(theta);
  double ss = sin(psi);
  double sp = sin(phi);
  double tt = tan(theta);

  // calculate forces - see eqs. 16-19, and 35-37.
  Eigen::Matrix<double, NUM_STATES, 1> xdot;
  xdot.setZero();
  xdot(PN) = ct*cs*u + (sp*st*cs-cp*ss)*v + (cp*st*cs+sp*ss)*w;
  xdot(PE) = cp*ss*u  + (sp*st*ss+cp*cs)*v + (cp*st*ss-sp*cs)*w;
  xdot(PD) = st*u     - sp*ct*v            - cp*ct*w;

  xdot(U) = (gz_+bz)*v-(gy_+by)*w - G*st;
  xdot(V) = (gx_+bx)*w-(gz_+bz)*u + G*ct*sp;
  xdot(W) = (gy_+by)*u-(gx_+bx)*v + G*ct*cp + (az_+alpha_z);

  xdot(PHI) = (gx_+bx) + sp*tt*(gy_+by) + cp*tt*(gz_+bz);
  xdot(THETA) = (gy_+by)*ct - (gz_+bz)*sp;
  xdot(PSI) = (gy_+by)*sp/ct + (gz_+bz)*cp/ct;

  // all other states (biases) are constant
  return xdot;
}



Eigen::Matrix<double, NUM_STATES, NUM_STATES> mocapFilter::dfdx(const Eigen::Matrix<double, NUM_STATES, 1> x)
{
  double u(x(U)), v(x(V)), w(x(W));
  double phi(x(PHI)), theta(x(THETA)), psi(x(PSI));
  double bx(x(BX)), by(x(BY)), bz(x(BZ));

  double ct = cos(theta);
  double cs = cos(psi);
  double cp = cos(phi);
  double st = sin(theta);
  double ss = sin(psi);
  double sp = sin(phi);
  double tt = tan(theta);

  Eigen::Matrix<double, NUM_STATES, NUM_STATES> A;
  A.setZero();
  A(PN,U) = ct*cs;
  A(PN,V) = sp*st*cs-cp*ss;
  A(PN,W) = cp*st*cs+sp*ss;
  A(PN,PHI) = (cp*st*cs+sp*ss)*v + (-sp*st*cs+cp*ss)*w;
  A(PN,THETA) = -st*cs*u + (sp*ct*cs)*v + (cp*ct*ss)*w;
  A(PN,PSI) = -ct*ss*u + (-sp*st*ss-cp*cs)*v + (-cp*st*ss+sp*cs)*w;

  A(PE,U) = cp*ss;
  A(PE,V) = sp*st*ss+cp*cs;
  A(PE,W) = cp*st*ss-sp*cs;
  A(PE,PHI) = (cp*st*ss-sp*cs)*v + (-sp*st*ss-cp*cs)*w;
  A(PE,THETA) = -st*ss*u + (sp*ct*ss)*v + (cp*ct*ss)*w;
  A(PE,PSI) = ct*cs*u  + (sp*st*cs-cp*ss)*v + (cp*st*cs+sp*ss)*w;

  A(PD,U) = st;
  A(PD,V) = -sp*ct;
  A(PD,W) = -cp*ct;
  A(PD,PHI) = cp*ct*v - sp*ct*w;
  A(PD,THETA) = -ct*u - sp*st*v - cp*st*w;

  A(U,V) = gz_+bz;
  A(U,W) = -(gy_+by);
  A(U,THETA) = -G*ct;
  A(U,BY) = -w;
  A(U,BZ) = v;

  A(V,U) = -(gz_+bz);
  A(V,W) = gx_+bx;
  A(V,PHI) = G*ct*cp;
  A(V,THETA) = -G*ct*sp;
  A(V,BX) = w;
  A(V,BZ) = -u;

  A(W,U) = gy_+by;
  A(W,V) = -(gx_+bx);
  A(W,PHI) = -G*ct*sp;
  A(W,THETA) = -G*st*cp;
  A(W,BX) = -v;
  A(W,BY) = u;

  A(PHI,PHI) = cp*tt*(gy_+by) - sp*tt*(gz_+bz);
  A(PHI,THETA) = (sp*(gy_+by) + cp*(gz_+bz))/(ct*ct);
  A(PHI,BX) = 1;
  A(PHI,BY) = sp*tt;
  A(PHI,BZ) = cp*tt;

  A(THETA,PHI) = -sp*(gy_+by) -cp*(gz_+bz);
  A(THETA,BY) = cp;
  A(THETA,BZ) = -sp;

  A(PSI,PHI) = ((gy_+by)*cp-(gz_+bz)*sp)/ct;
  A(PSI,THETA) = -((gy_+by)*sp+(gz_+bz)*cp)*tt/ct;
  A(PSI,BY) = sp/ct;
  A(PSI,BZ) = cp/ct;

  // all other states are zero

  return A;
}

void mocapFilter::publishEstimate()
{
  nav_msgs::Odometry estimate;
  double pn(x_hat_(PN)), pe(x_hat_(PE)), pd(x_hat_(PD));
  double u(x_hat_(U)), v(x_hat_(V)), w(x_hat_(W));
  double phi(x_hat_(PHI)), theta(x_hat_(THETA)), psi(x_hat_(PSI));
  double bx(x_hat_(BX)), by(x_hat_(BY)), bz(x_hat_(BZ));

  tf::Quaternion q;
  q.setRPY(phi, theta, psi);
  geometry_msgs::Quaternion qmsg;
  tf::quaternionTFToMsg(q, qmsg);
  estimate.pose.pose.orientation = qmsg;

  estimate.pose.pose.position.x = pn;
  estimate.pose.pose.position.y = pe;
  estimate.pose.pose.position.z = pd;

  estimate.pose.covariance[0*6+0] = P_(PN, PN);
  estimate.pose.covariance[1*6+1] = P_(PE, PE);
  estimate.pose.covariance[2*6+2] = P_(PD, PD);
  estimate.pose.covariance[3*6+3] = P_(PHI, PHI);
  estimate.pose.covariance[4*6+4] = P_(THETA, THETA);
  estimate.pose.covariance[5*6+5] = P_(PSI, PSI);

  estimate.twist.twist.linear.x = u;
  estimate.twist.twist.linear.y = v;
  estimate.twist.twist.linear.z = w;
  estimate.twist.twist.angular.x = gx_+bx;
  estimate.twist.twist.angular.y = gy_+by;
  estimate.twist.twist.angular.z = gz_+bz;

  estimate.twist.covariance[0*6+0] = P_(U, U);
  estimate.twist.covariance[1*6+1] = P_(V, V);
  estimate.twist.covariance[2*6+2] = P_(W, W);
  estimate.twist.covariance[3*6+3] = 0.05;  // not being estimated
  estimate.twist.covariance[4*6+4] = 0.05;
  estimate.twist.covariance[5*6+5] = 0.05;

  estimate.header.frame_id = "body_link";
  estimate.header.stamp = ros::Time::now();
  estimate_pub_.publish(estimate);

  double ax(x_hat_(AX)), ay(x_hat_(AY)),az(x_hat_(AZ));
  ROS_INFO_STREAM("ax: " << ax << " ay:" << ay << " az: " << az << " bx: " << bx << " by: " << by << " bz: " << bz);
}


double mocapFilter::LPF(double yn, double un)
{
  return alpha_*yn+(1-alpha_)*un;
}



} // namespace ekf


