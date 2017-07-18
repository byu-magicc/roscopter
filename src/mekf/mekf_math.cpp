// Name: Jerel Nielsen
// Date: 15 June 2017
// Desc: Container for common functions.

#include "mekf/mekf_math.h"

namespace mekf_math
{

/*=============================================================*/
// begin Quaternion class
/*=============================================================*/

Quaternion::Quaternion() 
{
  w_ = 1;
  x_ = 0;
  y_ = 0;
  z_ = 0;
}

Quaternion::~Quaternion() {}

Quaternion::Quaternion(double w, double x, double y, double z)
{
  w_ = w;
  x_ = x;
  y_ = y;
  z_ = z;
}

Quaternion::Quaternion(double roll, double pitch, double yaw)
{
  w_ = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
  x_ = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
  y_ = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
  z_ = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
}

// overload multiply operator for simple quaternion multiplication
Quaternion Quaternion::operator*(const Quaternion &q2)
{
  double qw = w_*q2.w_ - x_*q2.x_ - y_*q2.y_ - z_*q2.z_;
  double qx = w_*q2.x_ + x_*q2.w_ + y_*q2.z_ - z_*q2.y_;
  double qy = w_*q2.y_ - x_*q2.z_ + y_*q2.w_ + z_*q2.x_;
  double qz = w_*q2.z_ + x_*q2.y_ - y_*q2.x_ + z_*q2.w_;
  
  return Quaternion(qw, qx, qy, qz);
}

// overload stream operator for simple quaternion displaying
std::ostream& operator<<(std::ostream &os, const Quaternion &q)
{
  os << q.w_ << ", " << q.x_ << ", " << q.y_ << ", " << q.z_;
  return os;
}

// quaternion inverse
Quaternion Quaternion::inv()
{
  Quaternion q;
  q.w_ = w_;
  q.x_ = -x_;
  q.y_ = -y_;
  q.z_ = -z_;

  return q;
}

// quaternion norm
double Quaternion::mag()
{
  return sqrt(w_*w_ + x_*x_ + y_*y_ + z_*z_);
}

// quaternion normalization
void Quaternion::normalize()
{
  double mag = this->mag();
  w_ /= mag;
  x_ /= mag;
  y_ /= mag;
  z_ /= mag;
}

// conversion from quaternion to roll angle
double Quaternion::phi()
{
  return atan2(2*w_*x_ + 2*y_*z_, w_*w_ + z_*z_ - x_*x_ - y_*y_);
}

// conversion from quaternion to pitch angle
double Quaternion::theta()
{
  return asin(2*w_*y_ - 2*x_*z_);
}

// conversion from quaternion to yaw angle
double Quaternion::psi()
{
  return atan2(2*w_*z_ + 2*x_*y_, w_*w_ + x_*x_ - y_*y_ - z_*z_);
}

// convert Quaternion to Eigen vector
Eigen::Vector4d Quaternion::convertToEigen()
{
  Eigen::Vector4d v(w_, x_, y_, z_);
  return v;
}

// convert Eigen Vector to Quaternion
void Quaternion::convertFromEigen(Eigen::Vector4d q)
{
  w_ = q(0);
  x_ = q(1);
  y_ = q(2);
  z_ = q(3);
}

// rotation matrix
// e.g. if the quaternion is defined as the rotation to the body w.r.t. the
// vehicle frame, this is the 3x3 rotation from the vehicle to the body
Eigen::Matrix3d Quaternion::rot()
{
  Eigen::Matrix3d R;
  R <<  2*w_*w_ + 2*x_*x_ - 1,      2*w_*z_ + 2*x_*y_,     -2*w_*y_ + 2*x_*z_,
           -2*w_*z_ + 2*x_*y_,  2*w_*w_ + 2*y_*y_ - 1,      2*w_*x_ + 2*y_*z_,
            2*w_*y_ + 2*x_*z_,     -2*w_*x_ + 2*y_*z_,  2*w_*w_ + 2*z_*z_ - 1;
  return R;
}

// rotate a vector directly
Eigen::Vector3d Quaternion::rotateVector(Eigen::Vector3d v)
{
  Quaternion qv = Quaternion(0, v(0), v(1), v(2));
  Quaternion qv_new = this->inv() * qv * *this;
  return Eigen::Vector3d(qv_new.x_, qv_new.y_, qv_new.z_);
}

/*=============================================================*/
// end Quaternion class
/*=============================================================*/


// exponential map to unit quaternion
Quaternion exp_q(const Eigen::Vector3d delta)
{
  double delta_norm = delta.norm();

  Quaternion q;
  if (delta_norm < 1e-8) // avoid numerical error with approximation
  {
    q.w_ = 1;
    q.x_ = delta(0) / 2;
    q.y_ = delta(1) / 2;
    q.z_ = delta(2) / 2;
  }
  else
  {
    double sn = sin(delta_norm / 2)  /  delta_norm;
    q.w_ = cos(delta_norm / 2);
    q.x_ = sn * delta(0);
    q.y_ = sn * delta(1);
    q.z_ = sn * delta(2);
  }

  return q;
}


// unit quaternion logarithmic map to vector
Eigen::Vector3d log_q(const Quaternion q)
{
  // get magnitude of complex portion
  Eigen::Vector3d qbar(q.x_, q.y_, q.z_);
  double qbar_mag = qbar.norm();

  // avoid numerical error with approximation
  Eigen::Vector3d delta;
  if (qbar_mag < 1e-8)
  {
    q.w_ >= 0 ? delta = qbar : delta = -qbar;
  }
  else
  {
    delta = 2. * atan2(qbar_mag,q.w_) * qbar / qbar_mag;
  }

  return delta;
}


// skew symmetric matrix from vector
Eigen::Matrix3d skew(const Eigen::Vector3d vec)
{
  Eigen::Matrix3d A;
  A <<     0, -vec(2),  vec(1),
      vec(2),       0, -vec(0),
     -vec(1),  vec(0),       0;
  return A;
}


// rotation from vehicle-2 to body frame
Eigen::Matrix3d R_v2_to_b(double phi)
{
  Eigen::Matrix3d R_v22b;
  R_v22b << 1,         0,        0,
            0,  cos(phi), sin(phi),
            0, -sin(phi), cos(phi);
  return R_v22b;
}


// rotation from vehicle-1 to vehicle-2 frame
Eigen::Matrix3d R_v1_to_v2(double theta)
{
  Eigen::Matrix3d R_v12v2;
  R_v12v2 << cos(theta), 0, -sin(theta),
                      0, 1,           0,
             sin(theta), 0,  cos(theta);
  return R_v12v2;
}


// rotation from vehicle to vehicle-1 frame
Eigen::Matrix3d R_v_to_v1(double psi)
{
  Eigen::Matrix3d R_v2v1;
  R_v2v1 <<  cos(psi), sin(psi), 0,
            -sin(psi), cos(psi), 0,
                    0,        0, 1;
  return R_v2v1;
}


// rotation from vehicle to body frame
Eigen::Matrix3d R_v_to_b(double phi, double theta, double psi)
{
  return R_v2_to_b(phi) * R_v1_to_v2(theta) * R_v_to_v1(psi);
}


// rotation from NED style camera body coordinates to camera coordinates
Eigen::Matrix3d R_cb2c()
{
  Eigen::Matrix3d R;
  R << 0, 1, 0,
       0, 0, 1,
       1, 0, 0;

  return R;
}


// saturation function
double saturate(double value, double max, double min)
{
  if (value > max)
  {
    return max;
  }
  else if (value < min)
  {
    return min;
  }
  else
  {
    return value;
  }
}


} // namespace mekf_math


