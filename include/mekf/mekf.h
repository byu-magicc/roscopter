#ifndef MEKF_H
#define MEKF_H

#include <stdio.h>
#include <deque>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <lib/eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <fcu_common/GPS.h>



// state numbers
#define PN 0
#define PE 1
#define PD 2
#define QX 3
#define QY 4
#define QZ 5
#define QW 6
#define U 7
#define V 8
#define W 9
#define GX 10
#define GY 11
#define GZ 12
#define AX 13
#define AY 14
#define AZ 15
#define PNK 16
#define PEK 17
#define PDK 18
#define QXK 19
#define QYK 20
#define QZK 21
#define QWK 22
#define MU 23

#define NUM_STATES 24

// error state numbers
#define dPN 0
#define dPE 1
#define dPD 2
#define dPHI 3
#define dTHETA 4
#define dPSI 5
#define dU 6
#define dV 7
#define dW 8
#define dGX 9
#define dGY 10
#define dGZ 11
#define dAX 12
#define dAY 13
#define dAZ 14
#define dPNK 15
#define dPEK 16
#define dPDK 17
#define dPHIK 18
#define dTHETAK 19
#define dPSIK 20
#define dMU 21

#define NUM_ERROR_STATES 22

#define G 9.80
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
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	// publishers and subscribers
	ros::Subscriber imu_sub_;
	ros::Subscriber alt_sub_;
	ros::Subscriber gps_sub_;

	ros::Publisher estimate_pub_;
	ros::Publisher bias_pub_;
	ros::Publisher is_flying_pub_;
	ros::Publisher drag_pub_;

	// parameters
	double mass_;
	Eigen::Matrix<double, 3, 1> k_, g_;
	Eigen::Matrix<double, 3, 3> M_, I3_;

	struct inertia
	{
		double x;
		double y;
		double z;
	} J_;


	// local variables
	Eigen::Matrix<double, 6, 6> Qu_;
	Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> Qx_;
	Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> P_;
	Eigen::Matrix<double, NUM_STATES, 1> x_hat_;
	Eigen::Matrix<double, 3, 3> R_gps_;

	ros::Time current_time_;
	ros::Time previous_time_;

	double p_prev_, q_prev_, r_prev_;
	double ygx_, ygy_, ygz_, yaz_, yax_, yay_;
	double R_alt_;
	double gps_lat0_, gps_lon0_, gps_alt0_;
	int N_;
	bool flying_, first_gps_msg_;

	// functions
	void imuCallback(const sensor_msgs::Imu msg);
	void altCallback(const sensor_msgs::Range msg);
	void gpsCallback(const fcu_common::GPS msg);
	void predictStep();
	void updateStep();
	void updateIMU(const sensor_msgs::Imu msg);
	void publishEstimate();
	void stateUpdate(const Eigen::Matrix<double, NUM_ERROR_STATES, 1> delta_x);

	double LPF(double yn, double un, double alpha);

	Eigen::Matrix<double, NUM_STATES, 1> f(const Eigen::Matrix<double, NUM_STATES, 1> x);
	Eigen::Matrix<double, NUM_ERROR_STATES, NUM_ERROR_STATES> dfdx(const Eigen::Matrix<double, NUM_STATES, 1> x);
	Eigen::Matrix<double, NUM_ERROR_STATES, 6> dfdu(const Eigen::Matrix<double, NUM_STATES, 1> x);
	Eigen::Matrix<double, 4, 1> quatMul(const Eigen::Matrix<double, 4, 1> p, const Eigen::Matrix<double, 4, 1> q);
	Eigen::Matrix<double, 3, 3> Rq(const Eigen::Matrix<double, 4, 1> q);
	Eigen::Matrix<double, 3, 3> skew(const Eigen::Matrix<double, 3, 1> vec);

};

} // namespace mekf

#endif // MEKF_H
