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
	nh_private_.param<double>("Rsonar", R_sonar_, 0.05);
	nh_private_.param<double>("Rbaro", R_baro_, 0.05);
	nh_private_.param<double>("Rmag", R_mag_, 0.05);
	nh_private_.param<double>("declination", delta_d_, 0);
	nh_private_.param<int>("euler_integration_steps", N_, 20);
	ros_copter::importMatrixFromParamServer(nh_private_, x_hat_, "x0");
	ros_copter::importMatrixFromParamServer(nh_private_, P_, "P0");
	ros_copter::importMatrixFromParamServer(nh_private_, Qu_, "Qu");
	ros_copter::importMatrixFromParamServer(nh_private_, Qx_, "Qx");
  	ros_copter::importMatrixFromParamServer(nh_private_, R_gps_, "Rgps");
  	ros_copter::importMatrixFromParamServer(nh_private_, R_att_, "Ratt");

	// setup publishers and subscribers
	imu_sub_ = nh_.subscribe("imu/data", 1, &kalmanFilter::imuCallback, this);
	baro_sub_ = nh_.subscribe("baro", 1, &kalmanFilter::baroCallback, this);
	sonar_sub_ = nh_.subscribe("sonar/data", 1, &kalmanFilter::sonarCallback, this);
	mag_sub_ = nh_.subscribe("magnetometer", 1, &kalmanFilter::magCallback, this);
	gps_sub_ = nh_.subscribe("gps/data", 1, &kalmanFilter::gpsCallback, this);
	att_sub_ = nh_.subscribe("attitude", 1, &kalmanFilter::attitudeCallback, this);

	estimate_pub_  = nh_.advertise<nav_msgs::Odometry>("estimate", 1);
	bias_pub_      = nh_.advertise<sensor_msgs::Imu>("estimate/bias", 1);
	is_flying_pub_ = nh_.advertise<std_msgs::Bool>("is_flying", 1);

	// initialize variables
	flying_ = false;
	first_gps_msg_ = true;
	gps_lat0_ = 0;
	gps_lon0_ = 0;
	gps_alt0_ = 0;
	k_ << 0, 0, 1;
	g_ << 0, 0, G;
	I3_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
}


// the IMU serves as the heartbeat of the filter
void kalmanFilter::imuCallback(const sensor_msgs::Imu msg)
{
	// collect IMU measurements
	ygx_ = msg.angular_velocity.x;
	ygy_ = msg.angular_velocity.y;
	ygz_ = msg.angular_velocity.z;
	yax_ = msg.linear_acceleration.x;
	yay_ = msg.linear_acceleration.y;
	yaz_ = msg.linear_acceleration.z;

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
		statePropagate((dt/N_)*f(x_hat_));
		P_ += (dt/N_)*(F*P_ + P_*F.transpose() + Gu*Qu_*Gu.transpose() + Qx_);
	}

	// store time for time step computation on the next iteration
	previous_time_ = current_time_;

	return;
}


// update the state estimate, eq. 90 and 91
void kalmanFilter::statePropagate(const Eigen::Matrix<double, NUM_STATES, 1> delta_x)
{
	// separate vector and quaternion parts of x_hat_ and associated parts of delta_x
	Eigen::Matrix<double, NUM_STATES-4, 1> x_hat_v;
	Eigen::Matrix<double,            4, 1> x_hat_q;
	Eigen::Matrix<double, NUM_STATES-4, 1> delta_v;

	x_hat_v.block(0,0,3,1) = x_hat_.block(PN,0,3,1);
	x_hat_v.block(3,0,9,1) = x_hat_.block(U,0,9,1);

	x_hat_q = x_hat_.block(QX,0,4,1);

	delta_v.block(0,0,3,1) = delta_x.block(PN,0,3,1);
	delta_v.block(3,0,9,1) = delta_x.block(U,0,9,1);

	// update body and keyframe states
	x_hat_v += delta_v;
	x_hat_q = quatMul(x_hat_q, qexp(delta_x.block(QX,0,3,1)));

	// fill in new values of x_hat_
	x_hat_.block(PN,0,3,1) = x_hat_v.block(0,0,3,1);
	x_hat_.block(QX,0,4,1) = x_hat_q;
	x_hat_.block(U,0,9,1)  = x_hat_v.block(3,0,9,1);
}


// update the state estimate, eq. 90 and 91
void kalmanFilter::stateUpdate(const Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x)
{
	// separate vector and quaternion parts of x_hat_ and associated parts of delta_x
	// described in paragraph below eq. 89
	Eigen::Matrix<double, NUM_STATES-4, 1> x_hat_v;
	Eigen::Matrix<double,            4, 1> x_hat_q;
	Eigen::Matrix<double, NUM_STATES-4, 1> delta_v;
	Eigen::Matrix<double,            3, 1> delta_theta;

	x_hat_v.block(0,0,3,1) = x_hat_.block(PN,0,3,1);
	x_hat_v.block(3,0,9,1) = x_hat_.block(U,0,9,1);

	x_hat_q = x_hat_.block(QX,0,4,1);

	delta_v.block(0,0,3,1) = delta_x.block(dPN,0,3,1);
	delta_v.block(3,0,9,1) = delta_x.block(dU,0,9,1);

	delta_theta = delta_x.block(dPHI,0,3,1);

	// update body and keyframe states
	x_hat_v += delta_v;
	x_hat_q = quatMul(x_hat_q, qexp(delta_theta/2));

	// fill in new values of x_hat_
	x_hat_.block(PN,0,3,1) = x_hat_v.block(0,0,3,1);
	x_hat_.block(QX,0,4,1) = x_hat_q;
	x_hat_.block(U,0,9,1)  = x_hat_v.block(3,0,9,1);
}


// update altitude with barometer measurements
void kalmanFilter::baroCallback(const fcu_common::Barometer msg)
{
	// unpack message
	double y_alt = msg.altitude;

	// only update if valid sensor measurements
	if (fabs(x_hat_(PD)) > 5)
	{
		// measurement model
		double h_alt = -x_hat_(PD);

		// measurement Jacobian
		Eigen::Matrix<double, 1, NUM_ERROR_STATES> H;
		H.setZero();
		H(dPD) = -1;

		// compute the residual covariance
		double S = H*P_*H.transpose() + R_baro_;

		// compute Kalman gain
		Eigen::Matrix<double, NUM_ERROR_STATES, 1> K = P_*H.transpose()/S;

		// compute measurement error
		double r = y_alt - h_alt;

		// compute delta_x
		Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K*r;

		// update state and covariance
		stateUpdate(delta_x);
		P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K*H)*P_;
	}

	return;
}


// update altitude with sonar measurements
void kalmanFilter::sonarCallback(const sensor_msgs::Range msg)
{
	// unpack measurement
	double y_alt = msg.range;

	// only update if valid sensor measurements
	if (fabs(x_hat_(PD)) <= 5)
	{
		// measurement model, eq. 120
		double h_alt = -x_hat_(PD);

		// measurement Jacobian, eq. 121
		Eigen::Matrix<double, 1, NUM_ERROR_STATES> H;
		H.setZero();
		H(dPD) = -1;

		// compute the residual covariance
		double S = H*P_*H.transpose() + R_sonar_;

		// compute Kalman gain
		Eigen::Matrix<double, NUM_ERROR_STATES, 1> K = P_*H.transpose()/S;

		// compute measurement error
		double r = y_alt - h_alt;

		// compute delta_x, eq. 88
		Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K*r;

		// update state and covariance
		stateUpdate(delta_x);
		P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K*H)*P_;
	}

	return;
}


// update heading with magnetometer measurements
void kalmanFilter::magCallback(const sensor_msgs::MagneticField msg)
{
	// compute roll and pitch
	Eigen::Matrix<double, 4, 1> q_hat = x_hat_.block(QX,0,4,1);
	double phi_hat = phi(q_hat);
	double theta_hat = theta(q_hat);

	// the earth's magnetic field inclination is about 65 degrees here in Utah, so
	// if the aircraft is rolled or pitched over around 25 degrees, we cannot observe
	// the heading, check for this condition and skip update if condition is met
	if (sqrt(phi_hat*phi_hat + theta_hat*theta_hat) <= 0.3)
	{
		// unpack measurement
		Eigen::Matrix<double, 3, 1> m0;
		m0 << msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z;

		// remove aircraft roll and pitch
		Eigen::Matrix<double, 3, 3> R_v22b = R_v2_to_b(phi_hat);
		Eigen::Matrix<double, 3, 3> R_v12v2 = R_v1_to_v2(theta_hat);
		Eigen::Matrix<double, 3, 1> m = R_v12v2.transpose()*R_v22b.transpose()*m0;

		// compute heading measurement
		double psi_m = -atan2(m(1), m(0));
		double y_mag = delta_d_ + psi_m;

		// measurement model
		double h_mag = psi(q_hat);

		// measurement Jacobian
		Eigen::Matrix<double, 1, NUM_ERROR_STATES> H;
		H.setZero();
		H(dPSI) = 1;

		// compute the residual covariance
		double S = H*P_*H.transpose() + R_mag_;

		// compute Kalman gain
		Eigen::Matrix<double, NUM_ERROR_STATES, 1> K = P_*H.transpose()/S;

		// compute measurement error
		double r = y_mag - h_mag;

		// compute delta_x
		Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K*r;

		// update state and covariance
		stateUpdate(delta_x);
		P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K*H)*P_;
	}

	return;
}


// update with GPS measurements
void kalmanFilter::gpsCallback(const fcu_common::GPS msg)
{
	// unpack measurement and convert to radians
	double y_lat = msg.latitude*PI/180;
	double y_lon = msg.longitude*PI/180;
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
  	Eigen::Matrix<double, 2, 1> y_gps; 
	double r = EARTH_RADIUS+y_alt;
	double y_pn = r*sin(y_lat-gps_lat0_); // north from origin
	double y_pe = r*cos(y_lat-gps_lat0_)*sin(y_lon-gps_lon0_); // east from origin
	double y_pd = -(y_alt - gps_alt0_); // altitude relative to origin
	y_gps << y_pn, y_pe;

	// measurement model
	Eigen::Matrix<double, 2, 1> h_gps; 
	h_gps << x_hat_(PN), x_hat_(PE);

	// measurement Jacobian
	Eigen::Matrix<double, 2, NUM_ERROR_STATES> H;
	H.setZero();
	H(0,dPN) = 1;
	H(1,dPE) = 1;

	// compute the residual covariance
	Eigen::Matrix<double, 2, 2> S = H*P_*H.transpose() + R_gps_;

	// compute Kalman gain
	Eigen::Matrix<double, NUM_ERROR_STATES, 2> K = P_*H.transpose()*S.inverse();

	// compute measurement error
	Eigen::Matrix<double, 2, 1> residual = y_gps - h_gps;

	// compute delta_x
	Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K*residual;

	// update state and covariance
	stateUpdate(delta_x);
	P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K*H)*P_;

	return;
}


// update with ROSflight attitude estimate
void kalmanFilter::attitudeCallback(const fcu_common::Attitude msg)
{
	// unpack measurement
	Eigen::Matrix<double, 4, 1> q_meas;
	q_meas << msg.attitude.x, msg.attitude.y, msg.attitude.z, msg.attitude.w;
	Eigen::Matrix<double, 2, 1> y_att;
	y_att << phi(q_meas), theta(q_meas);

	// measurement model
	Eigen::Matrix<double, 4, 1> q_hat = x_hat_.block(QX,0,4,1);
	Eigen::Matrix<double, 2, 1> h_att;
	h_att << phi(q_hat), theta(q_hat);

	// measurement Jacobian
	Eigen::Matrix<double, 2, NUM_ERROR_STATES> H;
	H.setZero();
	H(0,dPHI) = 1;
	H(1,dTHETA) = 1;

	// compute the residual covariance
	Eigen::Matrix<double, 2, 2> S = H*P_*H.transpose() + R_att_;

	// compute Kalman gain
	Eigen::Matrix<double, NUM_ERROR_STATES, 2> K = P_*H.transpose()*S.inverse();

	// compute measurement error
	Eigen::Matrix<double, 2, 1> r = y_att - h_att;

	// compute delta_x
	Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x = K*r;

	// update state and covariance
	stateUpdate(delta_x);
	P_ = (Eigen::MatrixXd::Identity(NUM_ERROR_STATES,NUM_ERROR_STATES) - K*H)*P_;

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

	// eq. 105
	Eigen::Matrix<double, NUM_STATES, 1> xdot;
	xdot.setZero();
	xdot.block(0,0,3,1) = Rq(q_hat).transpose()*v_hat;
	xdot.block(3,0,4,1) = 0.5*quatMul(q_hat,omega_hat_4x1);
	xdot.block(7,0,3,1) = skew(v_hat)*omega_hat + Rq(q_hat)*g_ + k_*k_.transpose()*a_hat;

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

	q_hat     <<      x(QX),      x(QY),      x(QZ), x(QW);
	v_hat     <<       x(U),       x(V),       x(W);
	omega_hat << ygx_-x(GX), ygy_-x(GY), ygz_-x(GZ);

	// eq. 107
	Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> A;
	A.setZero();
	A.block(0,3,3,3)  = -Rq(q_hat).transpose()*skew(v_hat);
	A.block(0,6,3,3)  = Rq(q_hat).transpose();
	A.block(3,3,3,3)  = -skew(omega_hat);
	A.block(3,9,3,3)  = -I3_;
	A.block(6,3,3,3)  = skew(Rq(q_hat)*g_);
	A.block(6,6,3,3)  = -skew(omega_hat);
	A.block(6,9,3,3)  = -skew(v_hat);
	A.block(6,12,3,3) = -k_*k_.transpose();

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


// 3x3 rotation matrix from quaternion that rotates vehicle to body, eq. 15
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


Eigen::Matrix<double, 4, 1> kalmanFilter::qexp(const Eigen::Matrix<double, 3, 1> delta)
{
	// compute norm of delta
	double norm = sqrt(delta(0)*delta(0) + delta(1)*delta(1) + delta(2)*delta(2));

	// compute the exponential
	Eigen::Matrix<double, 4, 1> result;
	result.block(0,0,3,1) = norm == 0 ? 0*delta : (sin(norm)/norm)*delta;
	result(3) = cos(norm);

	return result;
}


// rotation from vehicle-2 to body frame
Eigen::Matrix<double, 3, 3> kalmanFilter::R_v2_to_b(double phi)
{
	Eigen::Matrix<double, 3, 3> R_v22b;
	R_v22b << 1, 0, 0, 0, cos(phi), sin(phi), 0, -sin(phi), cos(phi);
	return R_v22b;
}


// rotation from vehicle-1 to vehicle-2 frame
Eigen::Matrix<double, 3, 3> kalmanFilter::R_v1_to_v2(double theta)
{
	Eigen::Matrix<double, 3, 3> R_v12v2;
	R_v12v2 << cos(theta), 0, -sin(theta), 0, 1, 0, sin(theta), 0, cos(theta);
	return R_v12v2;
}


// rotation from vehicle to vehicle-1 frame
Eigen::Matrix<double, 3, 3> kalmanFilter::R_v_to_v1(double psi)
{
	Eigen::Matrix<double, 3, 3> R_v2v1;
	R_v2v1 << cos(psi), sin(psi), 0, -sin(psi), cos(psi), 0, 0, 0, 1;
	return R_v2v1;
}


// conversion from quaternion to roll angle
double kalmanFilter::phi(const Eigen::Matrix<double, 4, 1> q)
{
	// unpack quaternion
	double qx = q(0);
	double qy = q(1);
	double qz = q(2);
	double qw = q(3);

	// computer angle
	return atan2(2*qw*qx + 2*qy*qz, qw*qw + qz*qz - qx*qx - qy*qy);
}


// conversion from quaternion to pitch angle
double kalmanFilter::theta(const Eigen::Matrix<double, 4, 1> q)
{
	// unpack quaternion
	double qx = q(0);
	double qy = q(1);
	double qz = q(2);
	double qw = q(3);

	// computer angle
	return asin(2*qw*qy - 2*qx*qz);
}


// conversion from quaternion to yaw angle
double kalmanFilter::psi(const Eigen::Matrix<double, 4, 1> q)
{
	// unpack quaternion
	double qx = q(0);
	double qy = q(1);
	double qz = q(2);
	double qw = q(3);

	// computer angle
	return atan2(2*qw*qz + 2*qx*qy, qw*qw + qx*qx - qy*qy - qz*qz);
}


} // namespace mekf


