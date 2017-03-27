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

  // setup publishers and subscribers
  imu_sub_   = nh_.subscribe("imu/data", 1, &kalmanFilter::imuCallback, this);

  estimate_pub_  = nh_.advertise<nav_msgs::Odometry>("estimate", 1);
  bias_pub_      = nh_.advertise<sensor_msgs::Imu>("estimate/bias", 1);
  is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1);

  // initialize variables
  flying_ = false;
}


// the IMU serves as the heartbeat of the filter
void kalmanFilter::imuCallback(const sensor_msgs::Imu msg)
{
  // wait for sufficient acceleration to start the filter
  if(!flying_)
  {
    if(fabs(msg.linear_acceleration.z) > 10.0)
    {
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

  // normalize quaternion portion of state
  double x_hat_q_norm = sqrt(x_hat_(QX)*x_hat_(QX) + x_hat_(QY)*x_hat_(QY) + x_hat_(QZ)*x_hat_(QZ) + x_hat_(QW)*x_hat_(QW));
  x_hat_(QX) /= x_hat_q_norm;
  x_hat_(QY) /= x_hat_q_norm;
  x_hat_(QZ) /= x_hat_q_norm;
  x_hat_(QW) /= x_hat_q_norm;
  
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


// update state using accelerometer x and y measurements, Section 4.3.1
void kalmanFilter::updateIMU(sensor_msgs::Imu msg)
{
  // collect IMU measurements
  ygx_ = msg.angular_velocity.x;
  ygy_ = msg.angular_velocity.y;
  ygz_ = msg.angular_velocity.z;
  yax_ = msg.linear_acceleration.x;
  yay_ = msg.linear_acceleration.y;
  yaz_ = msg.linear_acceleration.z;
  
  // // build measurement vector (only use x and y)
  // Eigen::Matrix<double, 2, 1> y;
  // y << yax_, yay_;

  // // unpack the input
  // double u(x_hat_(U)), v(x_hat_(V)), w(x_hat_(W));
  // double p_hat(ygx_-x_hat_(GX)), q_hat(ygy_-x_hat_(GY)), r_hat(ygz_-x_hat_(GZ));
  // double bax(x_hat_(AX)), bay(x_hat_(AY));

  // // measurement Jacobian, eq. 115
  // Eigen::Matrix<double, 2, NUM_ERROR_STATES> H;
  // H << 0, 0, 0, 0, 0, 0,      0,  r_hat, -q_hat,  0,  w, -v, 1, 0, 0,
  //      0, 0, 0, 0, 0, 0, -r_hat,      0,  p_hat, -w,  0,  u, 0, 1, 0;

  // // input noise selector, eq. 118
  // Eigen::Matrix<double, 2, 6> J;
  // J <<  0,  w, -v, 1, 0, 0,
  //      -w,  0,  u, 0, 1, 0;

  // // accelerometer noise matrix for x and y
  // // input noise selector, eq. 118
  // Eigen::Matrix<double, 2, 2> R;
  // R << Qu_(3,3), 0, 0, Qu_(4,4);

  // // compute Kalman gain, eq. 117
  // Eigen::Matrix<double, NUM_ERROR_STATES, 2> K;
  // K = P_*H.transpose()*(H*P_*H.transpose() + J*Qu_*J.transpose() + R).inverse();

  // // compute measurement error below, eq. 114
  // Eigen::Matrix<double, 2, 1> r;
  // r << yax_ - (bax - q_hat*w + r_hat*v),
  //      yay_ - (bay + p_hat*w - r_hat*u);

  // // compute delta_x, eq. 88
  // Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x;
  // delta_x = K*r;

  // // update state and covariance
  // stateUpdate(delta_x);
  // P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K*H)*P_;
}


// build and publish estimate messages
void kalmanFilter::publishEstimate()
{
  nav_msgs::Odometry estimate;
  double pn(x_hat_(PN)), pe(x_hat_(PE)), pd(x_hat_(PD));
  double qx(x_hat_(QX)), qy(x_hat_(QY)), qz(x_hat_(QZ)), qw(x_hat_(QW));
  double u(x_hat_(U)), v(x_hat_(V)), w(x_hat_(W));
  double gx(x_hat_(GX)), gy(x_hat_(GY)), gz(x_hat_(GZ));
  double ax(x_hat_(AX)), ay(x_hat_(AY)), az(x_hat_(AZ));

  estimate.pose.pose.position.x = pn;
  estimate.pose.pose.position.y = pe;
  estimate.pose.pose.position.z = pd;

  estimate.pose.pose.orientation.x = qx;
  estimate.pose.pose.orientation.y = qy;
  estimate.pose.pose.orientation.z = qz;
  estimate.pose.pose.orientation.w = qw;

  estimate.pose.covariance[0*6+0] = P_(dPN, dPN);
  estimate.pose.covariance[1*6+1] = P_(dPE, dPE);
  estimate.pose.covariance[2*6+2] = P_(dPD, dPD);
  estimate.pose.covariance[3*6+3] = P_(dPHI, dPHI);
  estimate.pose.covariance[4*6+4] = P_(dTHETA, dTHETA);
  estimate.pose.covariance[5*6+5] = P_(dPSI, dPSI);

  estimate.twist.twist.linear.x = u;
  estimate.twist.twist.linear.y = v;
  estimate.twist.twist.linear.z = w;
  estimate.twist.twist.angular.x = ygx_-gx;
  estimate.twist.twist.angular.y = ygy_-gy;
  estimate.twist.twist.angular.z = ygz_-gz;

  estimate.twist.covariance[0*6+0] = P_(dU, dU);
  estimate.twist.covariance[1*6+1] = P_(dV, dV);
  estimate.twist.covariance[2*6+2] = P_(dW, dW);
  estimate.twist.covariance[3*6+3] = 0.05;  // not being estimated
  estimate.twist.covariance[4*6+4] = 0.05;
  estimate.twist.covariance[5*6+5] = 0.05;

  estimate.header.frame_id = "body_link";
  estimate.header.stamp = current_time_;
  estimate_pub_.publish(estimate);

  sensor_msgs::Imu bias;
  bias.linear_acceleration.x = x_hat_(AX);
  bias.linear_acceleration.y = x_hat_(AY);
  bias.linear_acceleration.z = x_hat_(AZ);
  bias.linear_acceleration_covariance[0*3+0] = P_(dAX,dAX);
  bias.linear_acceleration_covariance[1*3+1] = P_(dAY,dAY);
  bias.linear_acceleration_covariance[2*3+2] = P_(dAZ,dAZ);

  bias.angular_velocity.x = x_hat_(GX);
  bias.angular_velocity.y = x_hat_(GY);
  bias.angular_velocity.z = x_hat_(GZ);
  bias.angular_velocity_covariance[0*3+0] = P_(dGX,dGX);
  bias.angular_velocity_covariance[1*3+1] = P_(dGY,dGY);
  bias.angular_velocity_covariance[2*3+2] = P_(dGZ,dGZ);

  bias_pub_.publish(bias);
}


// low pass filter
double kalmanFilter::LPF(double yn, double un)
{
  return alpha_*yn + (1 - alpha_)*un;
}


// quaternion multiply, eq. 4
Eigen::Matrix<double, 4, 1> kalmanFilter::quatMul(const Eigen::Matrix<double, 4, 1> p, const Eigen::Matrix<double, 4, 1> q)
{
  // unpack elements of p
  double px = p(0);
  double py = p(1);
  double pz = p(2);
  double pw = p(3);

  // perform multiplication
  Eigen::Matrix<double, 4, 4> P;
  P <<  pw, -pz,  py,  px,
        pz,  pw, -px,  py,
       -py,  px,  pw,  pz,
       -px, -py, -pz,  pw;

  return P*q;
}


// update the state estimate, eq. 90 and 91
void kalmanFilter::stateUpdate(const Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x)
{
  // separate vector and quaternion parts of x_hat_ and associated parts of delta_x
  // described in paragraph below eq. 89
  Eigen::Matrix<double, 12, 1> x_hat_v;
  x_hat_v << x_hat_(PN), x_hat_(PE), x_hat_(PD), x_hat_(U), x_hat_(V), x_hat_(W), x_hat_(GX), x_hat_(GY), x_hat_(GZ), x_hat_(AX), x_hat_(AY), x_hat_(AZ);

  Eigen::Matrix<double, 4, 1> x_hat_q;
  x_hat_q << x_hat_(QX), x_hat_(QY), x_hat_(QZ), x_hat_(QW);

  Eigen::Matrix<double, 12, 1> delta_v;
  delta_v << delta_x(dPN), delta_x(dPE), delta_x(dPD), delta_x(dU), delta_x(dV), delta_x(dW), delta_x(dGX), delta_x(dGY), delta_x(dGZ), delta_x(dAX), delta_x(dAY), delta_x(dAZ);

  Eigen::Matrix<double, 4, 1> delta_theta_q;
  delta_theta_q << 0.5*delta_x(dPHI), 0.5*delta_x(dTHETA), 0.5*delta_x(dPSI), 1;

  // update state and covariance
  x_hat_v += delta_v;
  x_hat_q = quatMul(x_hat_q, delta_theta_q);

  // fill in new values of x_hat_
  x_hat_(PN) = x_hat_v(0);
  x_hat_(PE) = x_hat_v(1);
  x_hat_(PD) = x_hat_v(2);
  x_hat_(QX) = x_hat_q(0);
  x_hat_(QY) = x_hat_q(1);
  x_hat_(QZ) = x_hat_q(2);
  x_hat_(QW) = x_hat_q(3);
  x_hat_(U)  = x_hat_v(3);
  x_hat_(V)  = x_hat_v(4);
  x_hat_(W)  = x_hat_v(5);
  x_hat_(GX) = x_hat_v(6);
  x_hat_(GY) = x_hat_v(7);
  x_hat_(GZ) = x_hat_v(8);
  x_hat_(AX) = x_hat_v(9);
  x_hat_(AY) = x_hat_v(10);
  x_hat_(AZ) = x_hat_v(11);
}


} // namespace mekf


