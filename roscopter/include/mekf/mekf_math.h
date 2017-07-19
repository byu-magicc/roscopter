// Name: Jerel Nielsen
// Date: 18 July 2017
// Desc: Container for common functions.

#ifndef MEKF_MATH_H
#define MEKF_MATH_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace mekf_math
{


class Quaternion
{

public:

  Quaternion();
  ~Quaternion();
  Quaternion(double w, double x, double y, double z);
  Quaternion(double roll, double pitch, double yaw);

  double x_;
  double y_;
  double z_;
  double w_;

  Quaternion operator*(const Quaternion &q2);
  friend std::ostream& operator<<(std::ostream &os, const Quaternion &q);
  Quaternion inv();
  double mag();
  void normalize();
  double phi();
  double theta();
  double psi();
  void convertFromEigen(Eigen::Vector4d q);
  Eigen::Vector4d convertToEigen();
  Eigen::Matrix3d rot();
  Eigen::Vector3d rotateVector(Eigen::Vector3d v);

};

Quaternion exp_q(const Eigen::Vector3d delta);
Eigen::Vector3d log_q(const Quaternion q);
Eigen::Matrix3d skew(const Eigen::Vector3d vec);
Eigen::Matrix3d R_v2_to_b(double phi);
Eigen::Matrix3d R_v1_to_v2(double theta);
Eigen::Matrix3d R_v_to_v1(double psi);
Eigen::Matrix3d R_v_to_b(double phi, double theta, double psi);
Eigen::Matrix3d R_cb2c();
double saturate(double value, double max, double min);

} // namespace mekf_math

#endif // MEKF_MATH_H
