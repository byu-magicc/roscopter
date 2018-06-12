#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <boost/array.hpp>
#include <ros/ros.h>

namespace roscopter_common
{

class Quaternion
{

public:

  Quaternion();
  ~Quaternion();
  Quaternion(double _w, double _x, double _y, double _z);
  Quaternion(double roll, double pitch, double yaw);
  Quaternion(Eigen::Vector4d v);
  Quaternion(Eigen::Vector3d fz);

  double x;
  double y;
  double z;
  double w;

  Quaternion operator*(const Quaternion &q2);
  friend std::ostream& operator<<(std::ostream &os, const Quaternion &q);
  Quaternion inv();
  double mag();
  void normalize();
  double roll();
  double pitch();
  double yaw();
  void convertFromEigen(Eigen::Vector4d q);
  Eigen::Vector4d convertToEigen();
  Eigen::Matrix3d rot();
  Eigen::Vector3d rotateVector(Eigen::Vector3d v);
  Eigen::Vector3d unitVector();
  Eigen::MatrixXd projection();


};

Eigen::VectorXd rk5(Eigen::VectorXd state, Eigen::VectorXd input, std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd)> ode, double h);
Quaternion exp_q(const Eigen::Vector3d delta);
Eigen::Vector3d log_q(const Quaternion q);
Eigen::Vector3d log_R(const Eigen::Matrix3d R);
Quaternion vec2quat(const Eigen::Vector3d v);
Eigen::Vector3d vex(const Eigen::Matrix3d mat);
Eigen::Matrix3d skew(const Eigen::Vector3d vec);
Eigen::Matrix3d R_v2_to_b(double phi);
Eigen::Matrix3d R_v1_to_v2(double theta);
Eigen::Matrix3d R_v_to_v1(double psi);
Eigen::Matrix3d R_v_to_b(double phi, double theta, double psi);
Eigen::Matrix3d R_cb2c();

double saturate(double value, double max, double min);

/*---------------------------------------------
Templated functions implemented in header to avoid 
explicity instantiating all needed template instances.
----------------------------------------------*/

template <typename T1, typename T2>
void rosImportScalar(ros::NodeHandle nh, std::string param, T2& value, T1 default_value)
{
  // get scalar from ROS parameter server
  if (!nh.getParam(param, value))
  {
    ROS_WARN("Could not find %s/%s on the server.", nh.getNamespace().c_str(), param.c_str());
    value = default_value;
  }
}

template <typename T1, typename T2>
void rosImportMatrix(ros::NodeHandle nh, std::string param, Eigen::MatrixBase<T2>& mat)
{
  // get array from ROS parameter server
  std::vector<T1> vec;
  if (!nh.getParam(param, vec))
  {
    ROS_WARN("Could not find %s/%s on the server. Set to zeros.", nh.getNamespace().c_str(), param.c_str());
    mat.setZero();
    return;
  }
 
  // ensure imported array has correct number of values then populate the matrix
  ROS_ASSERT_MSG(vec.size() == mat.rows()*mat.cols(), "Param %s/%s is the wrong size", nh.getNamespace().c_str(),param.c_str());
  for (unsigned i = 0; i < mat.rows(); i++)
    for (unsigned j = 0; j < mat.cols(); j++)
      mat(i,j) = vec[mat.cols()*i+j];
}


} //end namespace roscopter_common

#endif // COMMON_H