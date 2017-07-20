#ifndef MEKF_H
#define MEKF_H

#include <stdio.h>
#include <deque>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <lib/eigen.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosflight_msgs/GPS.h>
#include <rosflight_msgs/Barometer.h>
#include <rosflight_msgs/Attitude.h>

#include <mekf/mekf_math.h>



// state numbers
enum StateIndices
{
	PN,	PE,	PD,
	QX,	QY,	QZ,	Q0,
	U, V, W,
	GX,	GY,	GZ,
	AX,	AY,	AZ,
	MU,
	NUM_STATES
};

// error state numbers
enum ErrorStateIndices
{
	dPN, dPE, dPD,
	dPHI, dTHETA, dPSI,
	dU, dV, dW,
	dGX, dGY, dGZ,
	dAX, dAY, dAZ,
	dMU,
	NUM_ERROR_STATES
};


#define GRAVITY 9.80665
#define EARTH_RADIUS 6371000
#define PI 3.14159265359

namespace mekf
{

class kalmanFilter
{

public:

	kalmanFilter();

private:

	// node handles, publishers, subscribers
	ros::NodeHandle nh_, nh_private_;

	// publishers and subscribers
	ros::Subscriber imu_sub_;
	ros::Subscriber gps_sub_;
	ros::Subscriber mag_sub_;
	ros::Subscriber att_sub_;
	ros::Subscriber baro_sub_;
	ros::Subscriber sonar_sub_;

	ros::Publisher estimate_pub_;
	ros::Publisher bias_pub_;
	ros::Publisher drag_pub_;
	ros::Publisher accel_pub_;
	ros::Publisher is_flying_pub_;

	// covariance and noise matrices
	Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> P_, Qx_;
	Eigen::Matrix<double, 6, 6> Qu_;
	Eigen::Matrix2d R_gps_;
	Eigen::Matrix2d R_att_;
	double R_sonar_, R_baro_, R_mag_;

	// states
	Eigen::Vector3d p_;
	mekf_math::Quaternion q_;
	Eigen::Vector3d v_;
	Eigen::Vector3d bg_;
	Eigen::Vector3d ba_;
	double mu_;

	// other parameters/variables
	Eigen::Vector3d k_, g_;
	Eigen::Matrix3d I3_, M_;
	Eigen::Matrix<double, 2, 3> I23_;

	bool flying_, first_gps_msg_;
	double delta_d_;
	double gyro_x_, gyro_y_, gyro_z_, acc_x_, acc_y_, acc_z_;
	double gps_lat0_, gps_lon0_, gps_alt0_;

	ros::Time current_time_, previous_time_;


	// functions
	void predictStep();
	void updateStep();
	void publishEstimate();
	void stateUpdate(const Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x);

	void imuCallback(const sensor_msgs::Imu msg);
	void baroCallback(const rosflight_msgs::Barometer msg);
	void sonarCallback(const sensor_msgs::Range msg);
	void magCallback(const sensor_msgs::MagneticField msg);
	void gpsCallback(const rosflight_msgs::GPS msg);
	void attitudeCallback(const rosflight_msgs::Attitude msg);

	Eigen::Matrix<double, NUM_ERROR_STATES, 1> f();
	Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> dfdx();
	Eigen::Matrix<double, NUM_ERROR_STATES, 6> dfdu();

};

} // namespace mekf

#endif // MEKF_H
