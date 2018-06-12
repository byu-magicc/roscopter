// This is an implementation of a multiplicative extended kalman filter (MEKF) based on
// the "Derivation of the Relative Multiplicative Kalman Filter" by David Wheeler and 
// Daniel Koch

#include "mekf/mekf.h"

namespace mekf
{

kalmanFilter::kalmanFilter() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("mekf"))
{
  // retrieve params
  roscopter_common::rosImportScalar<int>(nh_private_, "declination", delta_d_, 0);
  roscopter_common::rosImportScalar<double>(nh_private_, "mu0", mu_, 0.05);
  roscopter_common::rosImportMatrix<double>(nh_private_, "p0", p_);
  roscopter_common::rosImportMatrix<double>(nh_private_, "v0", v_);
  roscopter_common::rosImportMatrix<double>(nh_private_, "bg0", bg_);
  roscopter_common::rosImportMatrix<double>(nh_private_, "ba0", ba_);

  Eigen::Vector4d q_eigen;
  roscopter_common::rosImportMatrix<double>(nh_private_, "q0", q_eigen);
  q_.convertFromEigen(q_eigen);

  roscopter_common::rosImportMatrix<double>(nh_private_, "P0", P_);
  roscopter_common::rosImportMatrix<double>(nh_private_, "Qu", Qu_);
  roscopter_common::rosImportMatrix<double>(nh_private_, "Qx", Qx_);
  roscopter_common::rosImportMatrix<double>(nh_private_, "Rgps", R_gps_);
  roscopter_common::rosImportMatrix<double>(nh_private_, "Ratt", R_att_);
  roscopter_common::rosImportScalar<double>(nh_private_, "Rsonar", R_sonar_, 0.05);
  roscopter_common::rosImportScalar<double>(nh_private_, "Rbaro", R_baro_, 0.05);
  roscopter_common::rosImportScalar<double>(nh_private_, "Rmag", R_mag_, 0.05);

  // setup publishers and subscribers
  imu_sub_    = nh_.subscribe("imu/data", 1, &kalmanFilter::imuCallback, this);
  baro_sub_   = nh_.subscribe("baro", 1, &kalmanFilter::baroCallback, this);
  sonar_sub_  = nh_.subscribe("sonar", 1, &kalmanFilter::sonarCallback, this);
  mag_sub_    = nh_.subscribe("magnetometer", 1, &kalmanFilter::magCallback, this);
  gps_sub_    = nh_.subscribe("gps/data", 1, &kalmanFilter::gpsCallback, this);
  status_sub_ = nh_.subscribe("status", 1, &kalmanFilter::statusCallback, this);
  // att_sub_    = nh_.subscribe("attitude", 1, &kalmanFilter::attitudeCallback, this);

  estimate_pub_  = nh_.advertise<nav_msgs::Odometry>("estimate", 1);
  bias_pub_      = nh_.advertise<sensor_msgs::Imu>("estimate/bias", 1);
  drag_pub_      = nh_.advertise<std_msgs::Float64>("estimate/drag", 1);
  accel_pub_     = nh_.advertise<sensor_msgs::Imu>("estimate/accel", 1);
  is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1);

  // initialize variables
  flying_ = false;
  first_gps_msg_ = true;
  gps_lat0_ = 0;
  gps_lon0_ = 0;
  gps_alt0_ = 0;
  k_ << 0, 0, 1;
  g_ << 0, 0, GRAVITY;
  I3_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  I23_ << 1, 0, 0, 0, 1, 0;
  M_ << 1, 0, 0, 0, 1, 0, 0, 0, 0;

  armed_ = false;
}


void kalmanFilter::statusCallback(const rosflight_msgs::Status msg)
{
  armed_ = msg.armed;
}



// the IMU serves as the heartbeat of the filter
void kalmanFilter::imuCallback(const sensor_msgs::Imu msg)
{
  // collect IMU measurements
  gyro_x_ = msg.angular_velocity.x;
  gyro_y_ = msg.angular_velocity.y;
  gyro_z_ = msg.angular_velocity.z;
  acc_x_ = msg.linear_acceleration.x;
  acc_y_ = msg.linear_acceleration.y;
  acc_z_ = msg.linear_acceleration.z;

  // wait for sufficient acceleration to start the filter
  if(!flying_)
  {
    if(fabs(acc_z_) > 9.9 && armed_)
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
    updateStep();
    publishEstimate();
  }
  return;
}


// this function propagates filter dynamics foward in time
void kalmanFilter::predictStep()
{
  // get the current time step and build error state propagation Jacobians
  double dt = (current_time_-previous_time_).toSec();

  // propagate the state estimate and error state covariance via Euler integration
  int N = 10;
  for (unsigned i = 0; i < N; i++)
  {
    // propagate states
    Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = f() * (dt / N);
    p_ += delta_x.segment(dPN,3);
    q_ = q_ * roscopter_common::exp_q(delta_x.segment(dPHI,3));
    v_ += delta_x.segment(dU,3);

    // propagate covariance
    Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> F = dfdx();
    Eigen::Matrix<double, NUM_ERROR_STATES, 6> G = dfdu();
    P_ += (dt / N) * (F * P_ + P_ * F.transpose() + G * Qu_ * G.transpose() + Qx_);
  }

  // store time for time step computation on the next iteration
  previous_time_ = current_time_;

  return;
}


// this function updates the drag coefficient
void kalmanFilter::updateStep()
{
  // measurement model
  Eigen::Vector3d h = -mu_ * M_ * v_ + ba_;

  // measurement Jacobian
  Eigen::Matrix<double, 3, NUM_ERROR_STATES> H3;
  H3.setZero();
  H3.block(0,6,3,3) = -mu_ * M_;
  H3.block(0,12,3,3) = I3_;
  H3.block(0,15,3,1) = -M_ * v_;

  Eigen::Matrix<double, 2, NUM_ERROR_STATES> H = I23_ * H3;

  // compute the residual covariance
  Eigen::Matrix<double, 2, 2> S = H * P_ * H.transpose() + Qu_.block(3,3,2,2);

  // compute Kalman gain
  Eigen::Matrix<double, NUM_ERROR_STATES, 2> K = P_ * H.transpose() * S.inverse();

  // compute measurement error
  Eigen::Vector3d y(acc_x_, acc_y_, acc_z_);
  Eigen::Matrix<double, 3, 1> residual = y - h;

  // compute delta_x
  Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K * (I23_ * residual);

  // update state and covariance
  stateUpdate(delta_x);
  P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K * H) * P_;

  return;
}


// update the state estimate, eq. 90 and 91
void kalmanFilter::stateUpdate(const Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x)
{
  p_ += delta_x.segment(dPN,3);
  q_ = q_ * roscopter_common::exp_q(delta_x.segment(dPHI,3));
  v_ += delta_x.segment(dU,3);
  bg_ += delta_x.segment(dGX,3);
  ba_ += delta_x.segment(dAX,3);
  mu_ += delta_x(dMU);
  // mu_ = roscopter_common::saturate(mu_, 0.999, 0.001);
}


// state estimate dynamics
Eigen::Matrix<double, NUM_ERROR_STATES, 1> kalmanFilter::f()
{
  // unpack relevant data
  Eigen::Vector3d omega_hat(gyro_x_ - bg_(0), gyro_y_ - bg_(1), gyro_z_ - bg_(2));
  Eigen::Vector3d a_hat(acc_x_ - ba_(0), acc_y_ - ba_(1), acc_z_ - ba_(2));
  Eigen::Matrix3d R = q_.rot();

  // eq. 105
  Eigen::Matrix<double, NUM_ERROR_STATES, 1> xdot;
  xdot.setZero();
  xdot.segment(dPN,3) = R.transpose() * v_;
  xdot.segment(dPHI,3) = omega_hat;
  xdot.segment(dU,3) = roscopter_common::skew(v_) * omega_hat + R * g_ + k_ * k_.transpose() * a_hat - mu_ * M_ * v_;

  // all other state derivatives are zero
  return xdot;
}


// error state propagation Jacobian
Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> kalmanFilter::dfdx()
{
  // unpack relevant data
  Eigen::Vector3d omega_hat(gyro_x_ - bg_(0), gyro_y_ - bg_(1), gyro_z_ - bg_(2));
  Eigen::Vector3d a_hat(acc_x_ - ba_(0), acc_y_ - ba_(1), acc_z_ - ba_(2));
  Eigen::Matrix3d R = q_.rot();

  // eq. 107
  Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> F;
  F.setZero();
  F.block(0,3,3,3)  = -R.transpose() * roscopter_common::skew(v_);
  F.block(0,6,3,3)  = R.transpose();
  F.block(3,3,3,3)  = -roscopter_common::skew(omega_hat);
  F.block(3,9,3,3)  = -I3_;
  F.block(6,3,3,3)  = roscopter_common::skew(R * g_);
  F.block(6,6,3,3)  = -roscopter_common::skew(omega_hat);
  F.block(6,9,3,3)  = -roscopter_common::skew(v_);
  F.block(6,12,3,3) = -k_ * k_.transpose();
  F.block(6,15,3,1) = -M_ * v_;

  // all other states are zero
  return F;
}


// error state propagation Jacobian
Eigen::Matrix<double, NUM_ERROR_STATES, 6> kalmanFilter::dfdu()
{
  // eq. 108
  Eigen::Matrix<double, NUM_ERROR_STATES, 6> G;
  G.setZero();
  G.block(3,0,3,3) = -I3_;
  G.block(6,0,3,3) = -roscopter_common::skew(v_);
  G.block(6,3,3,3) = -k_ * k_.transpose();

  // all other states are zero
  return G;
}


// update altitude with barometer measurements
void kalmanFilter::baroCallback(const rosflight_msgs::Barometer msg)
{
  // unpack message
  double y_alt = msg.altitude;

  // only update if valid sensor measurements
  if (fabs(p_(2)) > 5)
  {
    // measurement model
    double h_alt = -p_(2);

    // measurement Jacobian
    Eigen::Matrix<double, 1, NUM_ERROR_STATES> H;
    H.setZero();
    H(dPD) = -1;

    // compute the residual covariance
    double S = H * P_ * H.transpose() + R_baro_;

    // compute Kalman gain
    Eigen::Matrix<double, NUM_ERROR_STATES, 1> K = P_ * H.transpose() / S;

    // compute measurement error
    double r = y_alt - h_alt;

    // compute delta_x
    Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K * r;

    // update state and covariance
    stateUpdate(delta_x);
    P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K * H) * P_;
  }

  return;
}


// update altitude with sonar measurements
void kalmanFilter::sonarCallback(const sensor_msgs::Range msg)
{
  // unpack measurement
  double y_alt = msg.range;

  // only update if valid sensor measurements
  if (y_alt > msg.min_range && y_alt < msg.max_range)
  {
    // measurement model, eq. 120
    double h_alt = -p_(2);

    // measurement Jacobian, eq. 121
    Eigen::Matrix<double, 1, NUM_ERROR_STATES> H;
    H.setZero();
    H(dPD) = -1;

    // compute the residual covariance
    double S = H * P_ * H.transpose() + R_sonar_;

    // compute Kalman gain
    Eigen::Matrix<double, NUM_ERROR_STATES, 1> K = P_ * H.transpose() / S;

    // compute measurement error
    double r = y_alt - h_alt;

    // compute delta_x, eq. 88
    Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K * r;

    // update state and covariance
    stateUpdate(delta_x);
    P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K * H) * P_;
  }

  return;
}


// update heading with magnetometer measurements
void kalmanFilter::magCallback(const sensor_msgs::MagneticField msg)
{
  // compute roll and pitch
  double phi_hat = q_.roll();
  double theta_hat = q_.pitch();

  // the earth's magnetic field inclination is about 65 degrees here in Utah, so
  // if the aircraft is rolled or pitched over around 25 degrees, we cannot observe
  // the heading, check for this condition and skip update if condition is met
  if (sqrt(phi_hat*phi_hat + theta_hat*theta_hat) <= 0.3)
  {
    // unpack measurement
    Eigen::Vector3d m0(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);

    // remove aircraft roll and pitch
    Eigen::Matrix3d R_v22b = roscopter_common::R_v2_to_b(phi_hat);
    Eigen::Matrix3d R_v12v2 = roscopter_common::R_v1_to_v2(theta_hat);
    Eigen::Vector3d m = R_v12v2.transpose() * R_v22b.transpose() * m0;

    // compute heading measurement
    double psi_m = -atan2(m(1), m(0));
    double y_mag = delta_d_ + psi_m;

    // measurement model
    double h_mag = q_.yaw();

    // measurement Jacobian
    Eigen::Matrix<double, 1, NUM_ERROR_STATES> H;
    H.setZero();
    H(dPSI) = 1;

    // compute the residual covariance
    double S = H * P_ * H.transpose() + R_mag_;

    // compute Kalman gain
    Eigen::Matrix<double, NUM_ERROR_STATES, 1> K = P_ * H.transpose() / S;

    // compute measurement error
    double r = y_mag - h_mag;

    // wrap heading measurement
    while (r > PI)
      r = r - 2 * PI;
    while (r < -PI)
      r = r + 2 * PI;

    // compute delta_x
    Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K * r;

    // update state and covariance
    stateUpdate(delta_x);
    P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K * H) * P_;
  }

  return;
}


// update with GPS measurements
void kalmanFilter::gpsCallback(const rosflight_msgs::GPS msg)
{
  // unpack measurement and convert to radians
  double y_lat = msg.latitude * PI / 180;
  double y_lon = msg.longitude * PI / 180;
  double y_alt = msg.altitude;

  // set origin with first message
  if (first_gps_msg_ == true)
  {
    gps_lat0_ = y_lat;
    gps_lon0_ = y_lon;
    gps_alt0_ = y_alt;

    first_gps_msg_ = false;

    return;
  }

  // convert to cartesian coordinates
  double r = EARTH_RADIUS + y_alt;
  double y_pn = r * sin(y_lat - gps_lat0_); // north from origin
  double y_pe = r * cos(y_lat - gps_lat0_) * sin(y_lon - gps_lon0_); // east from origin
  double y_pd = -(y_alt - gps_alt0_); // altitude relative to origin
  Eigen::Vector2d y_gps(y_pn, y_pe);

  // measurement model
  Eigen::Vector2d h_gps(p_(0), p_(1));

  // measurement Jacobian
  Eigen::Matrix<double, 2, NUM_ERROR_STATES> H;
  H.setZero();
  H(0,dPN) = 1;
  H(1,dPE) = 1;

  // compute the residual covariance
  Eigen::Matrix2d S = H * P_ * H.transpose() + R_gps_;

  // compute Kalman gain
  Eigen::Matrix<double, NUM_ERROR_STATES, 2> K = P_ * H.transpose() * S.inverse();

  // compute measurement error
  Eigen::Vector2d residual = y_gps - h_gps;

  // compute delta_x
  Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K * residual;

  // update state and covariance
  stateUpdate(delta_x);
  P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K * H) * P_;

  return;
}


// build and publish estimate messages
void kalmanFilter::publishEstimate()
{
  // publish state estimates
  nav_msgs::Odometry estimate;

  estimate.pose.pose.position.x = p_(0);
  estimate.pose.pose.position.y = p_(1);
  estimate.pose.pose.position.z = p_(2);

  estimate.pose.pose.orientation.w = q_.w;
  estimate.pose.pose.orientation.x = q_.x;
  estimate.pose.pose.orientation.y = q_.y;
  estimate.pose.pose.orientation.z = q_.z;

  estimate.pose.covariance[0*6+0] = P_(dPN, dPN);
  estimate.pose.covariance[1*6+1] = P_(dPE, dPE);
  estimate.pose.covariance[2*6+2] = P_(dPD, dPD);
  estimate.pose.covariance[3*6+3] = P_(dPHI, dPHI);
  estimate.pose.covariance[4*6+4] = P_(dTHETA, dTHETA);
  estimate.pose.covariance[5*6+5] = P_(dPSI, dPSI);

  estimate.twist.twist.linear.x = v_(0);
  estimate.twist.twist.linear.y = v_(1);
  estimate.twist.twist.linear.z = v_(2);
  estimate.twist.twist.angular.x = gyro_x_ - bg_(0);
  estimate.twist.twist.angular.y = gyro_y_ - bg_(1);
  estimate.twist.twist.angular.z = gyro_z_ - bg_(2);

  estimate.twist.covariance[0*6+0] = P_(dU, dU);
  estimate.twist.covariance[1*6+1] = P_(dV, dV);
  estimate.twist.covariance[2*6+2] = P_(dW, dW);
  estimate.twist.covariance[3*6+3] = 0.05;  // not being estimated
  estimate.twist.covariance[4*6+4] = 0.05;
  estimate.twist.covariance[5*6+5] = 0.05;

  estimate.header.frame_id = "body_link";
  estimate.header.stamp = current_time_;
  estimate_pub_.publish(estimate);

  // publish bias estimates
  sensor_msgs::Imu bias;

  bias.linear_acceleration.x = ba_(0);
  bias.linear_acceleration.y = ba_(1);
  bias.linear_acceleration.z = ba_(2);
  bias.linear_acceleration_covariance[0*3+0] = P_(dAX,dAX);
  bias.linear_acceleration_covariance[1*3+1] = P_(dAY,dAY);
  bias.linear_acceleration_covariance[2*3+2] = P_(dAZ,dAZ);

  bias.angular_velocity.x = bg_(0);
  bias.angular_velocity.y = bg_(1);
  bias.angular_velocity.z = bg_(2);
  bias.angular_velocity_covariance[0*3+0] = P_(dGX,dGX);
  bias.angular_velocity_covariance[1*3+1] = P_(dGY,dGY);
  bias.angular_velocity_covariance[2*3+2] = P_(dGZ,dGZ);

  bias_pub_.publish(bias);

  // publish drag estimate
  std_msgs::Float64 drag;

  drag.data = mu_;

  drag_pub_.publish(drag);

  // publish body acceleration estimate
  Eigen::Vector3d ahat(acc_x_, acc_y_, acc_z_);
  Eigen::Vector3d omega(gyro_x_, gyro_y_, gyro_z_);
  ahat = ahat - ba_ + q_.rot() * g_ - omega.cross(v_);
  sensor_msgs::Imu accel;

  accel.linear_acceleration.x = ahat(0);
  accel.linear_acceleration.y = ahat(1);
  accel.linear_acceleration.z = ahat(2);

  accel_pub_.publish(accel);
}


} // namespace mekf



