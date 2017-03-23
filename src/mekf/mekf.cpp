// This is an implementation of a multiplicative extended kalman filter (MEKF) based on
// the "Derivation of the Relative Multiplicative Kalman Filter" by David Wheeler and 
// Daniel Koch

#include "mekf/mekf.h"

namespace mekf
{

kalmanFilter::kalmanFilter() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~/mekf"))
{
  // retrieve params
  nh_private_.param<double>("alpha", alpha_, 0.2);
  nh_private_.param<int>("euler_integration_steps", N_, 20);
  ros_copter::importMatrixFromParamServer(nh_private_, x_hat_, "x0");
  ros_copter::importMatrixFromParamServer(nh_private_, P_, "P0");
  ros_copter::importMatrixFromParamServer(nh_private_, Qu_, "Qu");
  ros_copter::importMatrixFromParamServer(nh_private_, Qx_, "Qx");
  ros_copter::importMatrixFromParamServer(nh_private_, R_IMU_, "R_IMU");

  // setup publishers and subscribers
  imu_sub_   = nh_.subscribe("imu/data", 1, &kalmanFilter::imuCallback, this);

  estimate_pub_  = nh_.advertise<nav_msgs::Odometry>("estimate", 1);
  bias_pub_      = nh_.advertise<sensor_msgs::Imu>("estimate/bias", 1);
  is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1);

  // initialize variables
  x_hat_.setZero();
  flying_ = false;
}


// the IMU serves as the heartbeat of the filter
void kalmanFilter::imuCallback(const sensor_msgs::Imu msg)
{
  // wait for sufficient acceleration to start the filter
  if(!flying_){
    if(fabs(msg.linear_acceleration.z) > 10.0){
      // filter initialization stuff
      ROS_WARN("Now flying!");
      flying_ = true;
      std_msgs::Bool flying;
      flying.data = true;
      is_flying_pub_.publish(flying);
      previous_time_ = msg.header.stamp;
    }
  }
  else if(flying_)
  {
    // run the filter
    current_time_ = msg.header.stamp;
    predictStep();
    updateIMU(msg);
    publishEstimate();
  }
  return;
}


// this function propagates filter dynamics foward in time
void kalmanFilter::predictStep()
{
  // get the current time step and build error state propagation Jacobians
  double dt = (current_time_-previous_time_).toSec();
  Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> F = dfdx(x_hat_);
  Eigen::Matrix<double, NUM_ERROR_STATES, 6> Gu = dfdu(x_hat_);

  // propagate the state estimate and error state covariance via Euler integration
  for (unsigned i = 0; i < N_; i++)
  {
    x_hat_ += (dt/N_)*f(x_hat_); // eq. 70
    P_ += (dt/N_)*(F*P_ + P_*F.transpose() + Gu*Qu_*Gu.transpose() + Qx_); // eq. 79
  }
  
  // store time for time step computation on the next iteration
  previous_time_ = current_time_;
  
  return;
}


// state estimate dynamics
Eigen::Matrix<double, NUM_STATES, 1> kalmanFilter::f(const Eigen::Matrix<double, NUM_STATES, 1> x)
{
  // unpack the input
  double qx(x(QX)), qy(x(QY)), qz(x(QZ)), qw(x(QW));
  double u(x(U)), v(x(V)), w(x(W));
  double p_hat(ygx_-x(GX)), q_hat(ygy_-x(GY)), r_hat(ygz_-x(GZ));
  double az_hat(yaz_-x(AZ));

  // eq. 105
  Eigen::Matrix<double, NUM_STATES, 1> xdot;
  xdot <<  u*(2*qw*qw + 2*qx*qx - 1) - 2*v*(qw*qz - qx*qy) + 2*w*(qw*qy + qx*qz),
           2*u*(qw*qz + qx*qy) + v*(2*qw*qw + 2*qy*qy - 1) - 2*w*(qw*qx - qy*qz),
          -2*u*(qw*qy - qx*qz) + 2*v*(qw*qx + qy*qz) + w*(2*qw*qw + 2*qz*qz - 1),
                                      0.5*p_hat*qw - 0.5*q_hat*qz + 0.5*qy*r_hat,
                                      0.5*p_hat*qz + 0.5*q_hat*qw - 0.5*qx*r_hat,
                                     -0.5*p_hat*qy + 0.5*q_hat*qx + 0.5*qw*r_hat,
                                     -0.5*p_hat*qx - 0.5*q_hat*qy - 0.5*qz*r_hat,
                                        -2*G*(qw*qy - qx*qz) - q_hat*w + r_hat*v,
                                         2*G*(qw*qx + qy*qz) + p_hat*w - r_hat*u,
                          G*(2*qw*qw + 2*qz*qz - 1) + az_hat - p_hat*v + q_hat*u,
                                                                               0,
                                                                               0,
                                                                               0,
                                                                               0,
                                                                               0,
                                                                               0;

  // all other state derivatives are zero
  return xdot;
}


// error state propagation Jacobian
Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> kalmanFilter::dfdx(const Eigen::Matrix<double, NUM_STATES, 1> x)
{
  // unpack the input
  double qx(x(QX)), qy(x(QY)), qz(x(QZ)), qw(x(QW));
  double u(x(U)), v(x(V)), w(x(W));
  double p_hat(ygx_-x(GX)), q_hat(ygy_-x(GY)), r_hat(ygz_-x(GZ));

  // eq. 107
  Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> A;
  A << 0, 0, 0,        2*v*(qw*qy + qx*qz) + 2*w*(qw*qz - qx*qy), -2*u*(qw*qy + qx*qz) + w*(2*qw*qw + 2*qx*qx - 1), -2*u*(qw*qz - qx*qy) - v*(2*qw*qw + 2*qx*qx - 1), 2*qw*qw + 2*qx*qx - 1,    -2*qw*qz + 2*qx*qy,     2*qw*qy + 2*qx*qz,  0,  0,  0, 0, 0,  0,
       0, 0, 0, -2*v*(qw*qx - qy*qz) - w*(2*qw*qw + 2*qy*qy - 1),        2*u*(qw*qx - qy*qz) + 2*w*(qw*qz + qx*qy),  u*(2*qw*qw + 2*qy*qy - 1) - 2*v*(qw*qz + qx*qy),     2*qw*qz + 2*qx*qy, 2*qw*qw + 2*qy*qy - 1,    -2*qw*qx + 2*qy*qz,  0,  0,  0, 0, 0,  0,
       0, 0, 0,  v*(2*qw*qw + 2*qz*qz - 1) - 2*w*(qw*qx + qy*qz), -u*(2*qw*qw + 2*qz*qz - 1) - 2*w*(qw*qy - qx*qz),        2*u*(qw*qx + qy*qz) + 2*v*(qw*qy - qx*qz),    -2*qw*qy + 2*qx*qz,     2*qw*qx + 2*qy*qz, 2*qw*qw + 2*qz*qz - 1,  0,  0,  0, 0, 0,  0,
       0, 0, 0,                                                0,                                            r_hat,                                           -q_hat,                     0,                     0,                     0, -1,  0,  0, 0, 0,  0,
       0, 0, 0,                                           -r_hat,                                                0,                                            p_hat,                     0,                     0,                     0,  0, -1,  0, 0, 0,  0,
       0, 0, 0,                                            q_hat,                                           -p_hat,                                                0,                     0,                     0,                     0,  0,  0, -1, 0, 0,  0,
       0, 0, 0,                                                0,                       G*(-2*qw*qw - 2*qz*qz + 1),                              2*G*(qw*qx + qy*qz),                     0,                 r_hat,                -q_hat,  0,  w, -v, 0, 0,  0,
       0, 0, 0,                        G*(2*qw*qw + 2*qz*qz - 1),                                                0,                              2*G*(qw*qy - qx*qz),                -r_hat,                     0,                 p_hat, -w,  0,  u, 0, 0,  0,
       0, 0, 0,                             -2*G*(qw*qx + qy*qz),                             2*G*(-qw*qy + qx*qz),                                                0,                 q_hat,                -p_hat,                     0,  v, -u,  0, 0, 0, -1,
       0, 0, 0,                                                0,                                                0,                                                0,                     0,                     0,                     0,  0,  0,  0, 0, 0,  0,
       0, 0, 0,                                                0,                                                0,                                                0,                     0,                     0,                     0,  0,  0,  0, 0, 0,  0,
       0, 0, 0,                                                0,                                                0,                                                0,                     0,                     0,                     0,  0,  0,  0, 0, 0,  0,
       0, 0, 0,                                                0,                                                0,                                                0,                     0,                     0,                     0,  0,  0,  0, 0, 0,  0,
       0, 0, 0,                                                0,                                                0,                                                0,                     0,                     0,                     0,  0,  0,  0, 0, 0,  0,
       0, 0, 0,                                                0,                                                0,                                                0,                     0,                     0,                     0,  0,  0,  0, 0, 0,  0;

  // all other states are zero
  return A;
}


// error state propagation Jacobian
Eigen::Matrix<double, NUM_ERROR_STATES, 6> kalmanFilter::dfdu(const Eigen::Matrix<double, NUM_STATES, 1> x)
{
  double u(x(U)), v(x(V)), w(x(W));

  // eq. 108
  Eigen::Matrix<double, NUM_ERROR_STATES, 6> A;
  A <<  0,  0,  0, 0, 0,  0,
        0,  0,  0, 0, 0,  0,
        0,  0,  0, 0, 0,  0,
       -1,  0,  0, 0, 0,  0,
        0, -1,  0, 0, 0,  0,
        0,  0, -1, 0, 0,  0,
        0,  w, -v, 0, 0,  0,
       -w,  0,  u, 0, 0,  0,
        v, -u,  0, 0, 0, -1,
        0,  0,  0, 0, 0,  0,
        0,  0,  0, 0, 0,  0,
        0,  0,  0, 0, 0,  0,
        0,  0,  0, 0, 0,  0,
        0,  0,  0, 0, 0,  0,
        0,  0,  0, 0, 0,  0;

  // all other states are zero
  return A;
}


// build and publish estimate messages
void kalmanFilter::publishEstimate()
{
  nav_msgs::Odometry estimate;
  // double pn(x_hat_(PN)), pe(x_hat_(PE)), pd(x_hat_(PD));
  // double u(x_hat_(U)), v(x_hat_(V)), w(x_hat_(W));
  // double phi(x_hat_(PHI)), theta(x_hat_(THETA)), psi(x_hat_(PSI));
  // double bx(x_hat_(BX)), by(x_hat_(BY)), bz(x_hat_(BZ));

  // tf::Quaternion q;
  // q.setRPY(phi, theta, psi);
  // geometry_msgs::Quaternion qmsg;
  // tf::quaternionTFToMsg(q, qmsg);
  // estimate.pose.pose.orientation = qmsg;

  // estimate.pose.pose.position.x = pn;
  // estimate.pose.pose.position.y = pe;
  // estimate.pose.pose.position.z = pd;

  // estimate.pose.covariance[0*6+0] = P_(PN, PN);
  // estimate.pose.covariance[1*6+1] = P_(PE, PE);
  // estimate.pose.covariance[2*6+2] = P_(PD, PD);
  // estimate.pose.covariance[3*6+3] = P_(PHI, PHI);
  // estimate.pose.covariance[4*6+4] = P_(THETA, THETA);
  // estimate.pose.covariance[5*6+5] = P_(PSI, PSI);

  // estimate.twist.twist.linear.x = u;
  // estimate.twist.twist.linear.y = v;
  // estimate.twist.twist.linear.z = w;
  // estimate.twist.twist.angular.x = gx_+bx;
  // estimate.twist.twist.angular.y = gy_+by;
  // estimate.twist.twist.angular.z = gz_+bz;

  // estimate.twist.covariance[0*6+0] = P_(U, U);
  // estimate.twist.covariance[1*6+1] = P_(V, V);
  // estimate.twist.covariance[2*6+2] = P_(W, W);
  // estimate.twist.covariance[3*6+3] = 0.05;  // not being estimated
  // estimate.twist.covariance[4*6+4] = 0.05;
  // estimate.twist.covariance[5*6+5] = 0.05;

  // estimate.header.frame_id = "body_link";
  // estimate.header.stamp = current_time_;
  // estimate_pub_.publish(estimate);

  // sensor_msgs::Imu bias;
  // bias.linear_acceleration.x = x_hat_(AX);
  // bias.linear_acceleration.y = x_hat_(AY);
  // bias.linear_acceleration.z = x_hat_(AZ);
  // bias.linear_acceleration_covariance[0*3+0] = P_(AX,AX);
  // bias.linear_acceleration_covariance[1*3+1] = P_(AY,AY);
  // bias.linear_acceleration_covariance[2*3+2] = P_(AZ,AZ);

  // bias.angular_velocity.x = x_hat_(BX);
  // bias.angular_velocity.y = x_hat_(BY);
  // bias.angular_velocity.z = x_hat_(BZ);
  // bias.angular_velocity_covariance[0*3+0] = P_(BX,BX);
  // bias.angular_velocity_covariance[1*3+1] = P_(BY,BY);
  // bias.angular_velocity_covariance[2*3+2] = P_(BZ,BZ);

  // bias_pub_.publish(bias);
  // double ax(x_hat_(AX)), ay(x_hat_(AY)),az(x_hat_(AZ));
}


double kalmanFilter::LPF(double yn, double un)
{
  return alpha_*yn + (1 - alpha_)*un;
}


void kalmanFilter::updateIMU(sensor_msgs::Imu msg)
{
  //ROS_INFO_STREAM("Update IMU");
  // double phi(x_hat_(PHI)), theta(x_hat_(THETA)), psi(x_hat_(PSI));
  // double alpha_x(x_hat_(AX)), alpha_y(x_hat_(AY)), alpha_z(x_hat_(AZ));
  // double ct = cos(theta);
  // double cp = cos(phi);
  // double st = sin(theta);
  // double sp = sin(phi);
  ygx_ = msg.angular_velocity.x;
  ygy_ = msg.angular_velocity.y;
  ygz_ = msg.angular_velocity.z;
  yax_ = msg.linear_acceleration.x;
  yay_ = msg.linear_acceleration.y;
  yaz_ = msg.linear_acceleration.z;

  // Eigen::Matrix<double, 3, 1> y;
  // y << yax_, yay_, yaz_;

  // Eigen::Matrix<double, 3, NUM_STATES> C;
  // C.setZero();
  // C(0,THETA) = -G*ct;
  // C(0,AX) = -1.0;

  // C(1,PHI) = G*ct*cp;
  // C(1,THETA) = -G*st*sp;
  // C(1,AY) = -1.0;

  // C(2,PHI) = G*ct*sp;
  // C(2,THETA) = G*st*cp;
  // C(2,AZ) = -1.0;

  // Eigen::Matrix<double, NUM_STATES, 3> L;
  // L.setZero();
  // L = P_*C.transpose()*(R_IMU_ + C*P_*C.transpose()).inverse();
  // P_ = (Eigen::MatrixXd::Identity(NUM_STATES,NUM_STATES) - L*C)*P_;
  // x_hat_ = x_hat_ + L*(y - C*x_hat_);
}


} // namespace mekf


