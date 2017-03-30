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
	nh_private_.param<double>("mass", mass_, 1);
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
	k_ << 0, 0, 1;
	g_ << 0, 0, G;
	M_ << 1/mass_, 0, 0, 0, 1/mass_, 0, 0, 0, 0;
	I3_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
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
	// unpack the input and create relevant matrices
  Eigen::Matrix<double, 4, 1> q_hat;
  Eigen::Matrix<double, 3, 1> v_hat;
  Eigen::Matrix<double, 3, 1> omega_hat;
  Eigen::Matrix<double, 4, 1> omega_hat_4x1;
  Eigen::Matrix<double, 3, 1> a_hat;

  q_hat         <<        x(QX),        x(QY),        x(QZ), x(QW);
  v_hat         <<         x(U),         x(V),         x(W);
  omega_hat     <<   ygx_-x(GX),   ygy_-x(GY),   ygz_-x(GZ);
  a_hat         <<   yax_-x(AX),   yay_-x(AY),   yaz_-x(AZ);
  omega_hat_4x1 << omega_hat(0), omega_hat(1), omega_hat(2), 0;

	double mu_hat(x(MU));

	// eq. 105
	Eigen::Matrix<double, NUM_STATES, 1> xdot;
	xdot.setZero();
	xdot.block(0,0,3,1) = Rq(q_hat).transpose()*v_hat;
	xdot.block(3,0,4,1) = 0.5*quatMul(q_hat,omega_hat_4x1);
	xdot.block(7,0,3,1) = skew(v_hat)*omega_hat+Rq(q_hat)*g_+k_*k_.transpose()*a_hat-mu_hat*M_*v_hat;

	// all other state derivatives are zero
	return xdot;
}


// error state propagation Jacobian
Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> kalmanFilter::dfdx(const Eigen::Matrix<double, NUM_STATES, 1> x)
{
	// unpack the input and create relevant matrices
  Eigen::Matrix<double, 4, 1> q_hat;
  Eigen::Matrix<double, 3, 1> v_hat;
  Eigen::Matrix<double, 3, 1> omega_hat;

  q_hat    <<       x(QX),      x(QY),      x(QZ), x(QW);
  v_hat    <<        x(U),       x(V),       x(W);
  omega_hat << ygx_-x(GX), ygy_-x(GY), ygz_-x(GZ);

  double mu_hat(x(MU));

	// eq. 107
	Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> A;
	A.setZero();
	A.block(0,3,3,3)  = -Rq(q_hat).transpose()*skew(v_hat);
	A.block(0,6,3,3)  = Rq(q_hat).transpose();
	A.block(3,3,3,3)  = -skew(omega_hat);
	A.block(3,9,3,3)  = -I3_;
	A.block(6,3,3,3)  = skew(Rq(q_hat)*g_);
	A.block(6,6,3,3)  = -skew(omega_hat)-mu_hat*M_;
	A.block(6,9,3,3)  = -skew(v_hat);
	A.block(6,12,3,3) = -k_*k_.transpose();
	A.block(6,21,3,1) = -M_*v_hat;

	// all other states are zero
	return A;
}


// error state propagation Jacobian
Eigen::Matrix<double, NUM_ERROR_STATES, 6> kalmanFilter::dfdu(const Eigen::Matrix<double, NUM_STATES, 1> x)
{
	// unpack the input and create relevant matrices
  Eigen::Matrix<double, 3, 1> v_hat;

  v_hat << x(U), x(V), x(W);

	// eq. 108
	Eigen::Matrix<double, NUM_ERROR_STATES, 6> A;
	A.setZero();
	A.block(3,0,3,3) = -I3_;
	A.block(6,0,3,3) = -skew(v_hat);
	A.block(6,3,3,3) = -k_*k_.transpose();

	// all other states are zero
	return A;
}


// update state using accelerometer x and y measurements, Section 4.3.1
void kalmanFilter::updateIMU(const sensor_msgs::Imu msg)
{
	// collect IMU measurements
	ygx_ = msg.angular_velocity.x;
	ygy_ = msg.angular_velocity.y;
	ygz_ = msg.angular_velocity.z;
	yax_ = msg.linear_acceleration.x;
	yay_ = msg.linear_acceleration.y;
	yaz_ = msg.linear_acceleration.z;

	// // only apply update when accelerometer is approximately measuring gravity
	// double tol = 0.1;
	// if (fabs(yax_) < tol && fabs(yay_) < tol && fabs(yaz_) > G*(1-tol) && fabs(yaz_) < G*(1+tol))
	// {
	// 	// build measurement vector (only use x and y)
	// 	Eigen::Matrix<double, 2, 1> y;
	// 	y << yax_, yay_;

	// 	// unpack the input
	// 	double u(x_hat_(U)), v(x_hat_(V)), w(x_hat_(W));
	// 	double p_hat(ygx_-x_hat_(GX)), q_hat(ygy_-x_hat_(GY)), r_hat(ygz_-x_hat_(GZ));
	// 	double bax(x_hat_(AX)), bay(x_hat_(AY));

	// 	// measurement Jacobian, eq. 115
	// 	Eigen::Matrix<double, 2, NUM_ERROR_STATES> H;
	// 	H << 0, 0, 0, 0, 0, 0,      0,  r_hat, -q_hat,  0,  w, -v, 1, 0, 0,
	// 	     0, 0, 0, 0, 0, 0, -r_hat,      0,  p_hat, -w,  0,  u, 0, 1, 0;

	// 	// input noise selector, eq. 118
	// 	Eigen::Matrix<double, 2, 6> J;
	// 	J <<  0,  w, -v, 1, 0, 0,
	// 	     -w,  0,  u, 0, 1, 0;

	// 	// accelerometer noise matrix for x and y
	// 	// input noise selector, eq. 118
	// 	Eigen::Matrix<double, 2, 2> R;
	// 	R << Qu_(3,3),        0, 
	// 	            0, Qu_(4,4);

	// 	// compute Kalman gain, eq. 117
	// 	Eigen::Matrix<double, NUM_ERROR_STATES, 2> K;
	// 	K = P_*H.transpose()*(H*P_*H.transpose() + J*Qu_*J.transpose() + R).inverse();

	// 	// compute measurement error below, eq. 114
	// 	Eigen::Matrix<double, 2, 1> r;
	// 	r << yax_ - (bax - q_hat*w + r_hat*v),
	// 	     yay_ - (bay + p_hat*w - r_hat*u);

	// 	// compute delta_x, eq. 88
	// 	Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x;
	// 	delta_x = K*r;

	// 	// update state and covariance
	// 	stateUpdate(delta_x);
	// 	P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K*H)*P_;
	// }
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
	// create needed matrices/vectors
  Eigen::Matrix<double, 3, 1> p_bar;
  p_bar << p(0), p(1), p(2);

  double pw(p(3));

	// perform multiplication
	Eigen::Matrix<double, 4, 4> P;
	P.block(0,0,3,3) = pw*I3_+skew(p_bar);
	P.block(0,3,3,1) = p_bar;
	P.block(3,0,1,3) = -p_bar.transpose();
	P(3,3) = pw;

	return P*q;
}


// 3x3 rotation matrix from quaternion, eq. 15
Eigen::Matrix<double, 3, 3> kalmanFilter::Rq(const Eigen::Matrix<double, 4, 1> q)
{
  // create needed matrices/vectors
  Eigen::Matrix<double, 3, 1> q_bar;
  q_bar << q(0), q(1), q(2);

  double qw(q(3));

  // compute rotation matrix
  return (2*qw*qw-1)*I3_-2*qw*skew(q_bar)+2*q_bar*q_bar.transpose();
}


// skew symmetric matrix from vector, eq. 5
Eigen::Matrix<double, 3, 3> kalmanFilter::skew(const Eigen::Matrix<double, 3, 1> vec)
{
  Eigen::Matrix<double, 3, 3> A;
  A <<       0, -vec(2),  vec(1),
        vec(2),       0, -vec(0),
       -vec(1),  vec(0),       0;
  return A;
}


// update the state estimate, eq. 90 and 91
void kalmanFilter::stateUpdate(const Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x)
{
	// separate vector and quaternion parts of x_hat_ and associated parts of delta_x
	// described in paragraph below eq. 89
	Eigen::Matrix<double, 16, 1> x_hat_v;
	Eigen::Matrix<double,  4, 1> x_hat_q;
	Eigen::Matrix<double,  4, 1> x_hat_q_k;
	Eigen::Matrix<double, 16, 1> delta_v;
	Eigen::Matrix<double,  3, 1> delta_theta;
	Eigen::Matrix<double,  3, 1> delta_theta_k;
	Eigen::Matrix<double,  4, 1> delta_theta_q;
	Eigen::Matrix<double,  4, 1> delta_theta_q_k;

	x_hat_v.block(0,0,3,1) = x_hat_.block(PN,0,3,1);
	x_hat_v.block(3,0,12,1) = x_hat_.block(U,0,12,1);
	x_hat_v(15) = x_hat_(MU);

	x_hat_q = x_hat_.block(QX,0,4,1);
	x_hat_q_k = x_hat_.block(QXK,0,4,1);

	delta_v.block(0,0,3,1) = delta_x.block(dPN,0,3,1);
	delta_v.block(3,0,12,1) = delta_x.block(dU,0,12,1);
	delta_v(15) = delta_x(dMU);

	delta_theta = delta_x.block(dPHI,0,3,1);
	delta_theta_k = delta_x.block(dPHIK,0,3,1);

	delta_theta_q.block(0,0,3,1) = 0.5*delta_theta;
	delta_theta_q(3) = 1;
	delta_theta_q /= sqrt(1+0.25*delta_theta.transpose()*delta_theta);

	delta_theta_q_k.block(0,0,3,1) = 0.5*delta_theta_k;
	delta_theta_q_k(3) = 1;
	delta_theta_q_k /= sqrt(1+0.25*delta_theta_k.transpose()*delta_theta_k);

	// update body and keyframe states
	x_hat_v += delta_v;
	x_hat_q = quatMul(x_hat_q, delta_theta_q);
	x_hat_q_k = quatMul(x_hat_q_k, delta_theta_q_k);

	// fill in new values of x_hat_
	x_hat_.block(PN,0,3,1) = x_hat_v.block(0,0,3,1);
	x_hat_.block(QX,0,4,1) = x_hat_q;
	x_hat_.block(U,0,12,1) = x_hat_v.block(3,0,12,1);
	x_hat_.block(QXK,0,4,1) = x_hat_q_k;
	x_hat_(MU) = x_hat_v(15);
}


} // namespace mekf


