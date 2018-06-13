#include "roscopter_common/common.h"

namespace roscopter_common
{

/*=============================================================*/
// begin Quaternion class
/*=============================================================*/

Quaternion::Quaternion() 
{
  w = 1;
  x = 0;
  y = 0;
  z = 0;
}

Quaternion::~Quaternion() {}

Quaternion::Quaternion(double _w, double _x, double _y, double _z)
{
  w = _w;
  x = _x;
  y = _y;
  z = _z;
}

Quaternion::Quaternion(double roll, double pitch, double yaw)
{
  w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
  x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
  y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
  z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
}

Quaternion::Quaternion(Eigen::Vector4d v)
{
  w = v(0);
  x = v(1);
  y = v(2);
  z = v(3);
}

Quaternion::Quaternion(Eigen::Vector3d fz)
{
  // convert to axis-angle representation
  Eigen::Vector3d ez(0, 0, 1);
  double theta = acos(fz.dot(ez));

  if (theta < 1e-8)
  {
    w = 1;
    x = 0;
    y = 0;
    z = 0;
  }
  else
  {
    Eigen::Vector3d fzcross_ez = fz.cross(ez);
    Eigen::Vector3d iaa = fzcross_ez/(fzcross_ez.norm());

    // get complex portion of quaternion
    Eigen::Vector3d qv = iaa * sin(theta/2);

    w = cos(theta/2);
    x = qv(0);
    y = qv(1);
    z = qv(2);
  }
}

// overload multiply operator for simple quaternion multiplication
Quaternion Quaternion::operator*(const Quaternion &q2)
{
  double qw = w*q2.w - x*q2.x - y*q2.y - z*q2.z;
  double qx = w*q2.x + x*q2.w + y*q2.z - z*q2.y;
  double qy = w*q2.y - x*q2.z + y*q2.w + z*q2.x;
  double qz = w*q2.z + x*q2.y - y*q2.x + z*q2.w;
  
  return Quaternion(qw, qx, qy, qz);
}

// overload stream operator for simple quaternion displaying
std::ostream& operator<<(std::ostream &os, const Quaternion &q)
{
  os << q.w << ", " << q.x << ", " << q.y << ", " << q.z;
  return os;
}

// quaternion inverse
Quaternion Quaternion::inv()
{
  Quaternion q;
  q.w = w;
  q.x = -x;
  q.y = -y;
  q.z = -z;

  return q;
}

// quaternion norm
double Quaternion::mag()
{
  return sqrt(w*w + x*x + y*y + z*z);
}

// quaternion normalization
void Quaternion::normalize()
{
  double mag = this->mag();
  w /= mag;
  x /= mag;
  y /= mag;
  z /= mag;
}

// conversion from quaternion to roll angle
double Quaternion::roll()
{
  return atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
}

// conversion from quaternion to pitch angle
double Quaternion::pitch()
{
  double val = 2*(w*y - x*z);

  // hold at 90 degrees if invalid
  if (fabs(val) > 1)
    return copysign(1, val)*M_PI/2;
  else
    return asin(val);
}

// conversion from quaternion to yaw angle
double Quaternion::yaw()
{
  return atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
}

// convert Quaternion to Eigen vector
Eigen::Vector4d Quaternion::convertToEigen()
{
  Eigen::Vector4d v;
  v << w, x, y, z;
  return v;
}

// convert Eigen Vector to Quaternion
void Quaternion::convertFromEigen(Eigen::Vector4d q)
{
  w = q(0);
  x = q(1);
  y = q(2);
  z = q(3);
}

// create rotation matrix from quaternion
Eigen::Matrix3d Quaternion::rot()
{
  Eigen::Matrix3d R;
  R <<  2*w*w + 2*x*x - 1,      2*w*z + 2*x*y,     -2*w*y + 2*x*z,
           -2*w*z + 2*x*y,  2*w*w + 2*y*y - 1,      2*w*x + 2*y*z,
            2*w*y + 2*x*z,     -2*w*x + 2*y*z,  2*w*w + 2*z*z - 1;
  return R;
}

// rotate a 3-vector directly
Eigen::Vector3d Quaternion::rotateVector(Eigen::Vector3d v)
{
  Quaternion qv = Quaternion(0, v(0), v(1), v(2));
  Quaternion qv_new = this->inv() * qv * *this;
  return Eigen::Vector3d(qv_new.x, qv_new.y, qv_new.z);
}

// compute the unit vector in the camera frame given its quaternion
Eigen::Vector3d Quaternion::unitVector()
{
  Eigen::Vector3d ez;
  ez << 0, 0, 1;
  return rot() * ez;
}

// projection matrix of unit vector onto its tangent space
Eigen::MatrixXd Quaternion::projection()
{
  Eigen::MatrixXd E(3,2);
  E << 1, 0,
       0, 1,
       0, 0;
  return rot() * E;
}


/*=============================================================*/
// end Quaternion class
/*=============================================================*/


// solve for future state vector given associated ordinary differential equations
Eigen::VectorXd rk5(Eigen::VectorXd state, Eigen::VectorXd input, std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd)> ode, double h)
{
  // 5th order Dormand-Prince integration
  Eigen::VectorXd k1 = ode(state                      , input);
  Eigen::VectorXd k2 = ode(state + k1 * (h / 5.)      , input);
  Eigen::VectorXd k3 = ode(state + k2 * (h * 3. / 10.), input);
  Eigen::VectorXd k4 = ode(state + k3 * (h * 4. / 5.) , input);
  Eigen::VectorXd k5 = ode(state + k4 * (h * 8. / 9.) , input);
  Eigen::VectorXd k6 = ode(state + k5 * h             , input);

  return state + (k1 * (35. / 384.) + k3 * (500. / 1113.) + k4 * (125. / 192.) + k5 * (-2187. / 6784.) + k6 * (11. /84.)) * h;
} 


// exponential map to unit quaternion
Quaternion exp_q(const Eigen::Vector3d delta)
{
  double delta_norm = delta.norm();

  Quaternion q;
  if (delta_norm < 1e-6) // avoid numerical error with approximation
  {
    q.w = 1;
    q.x = delta(0)/2;
    q.y = delta(1)/2;
    q.z = delta(2)/2;
  }
  else
  {
    double sn = sin(delta_norm/2)/delta_norm;
    q.w = cos(delta_norm/2);
    q.x = sn*delta(0);
    q.y = sn*delta(1);
    q.z = sn*delta(2);
  }

  return q;
}


// unit quaternion logarithmic map to vector
Eigen::Vector3d log_q(const Quaternion q)
{
  // get magnitude of complex portion
  Eigen::Vector3d qbar(q.x, q.y, q.z);
  double qbar_mag = qbar.norm();

  // avoid numerical error with approximation
  Eigen::Vector3d delta;
  if (qbar_mag < 1e-6)
    q.w >= 0 ? delta = qbar : delta = -qbar;
  else
    delta = 2. * atan2(qbar_mag,q.w) * qbar / qbar_mag;

  return delta;
}


// rotation matrix logarithmic map to vector
Eigen::Vector3d log_R(const Eigen::Matrix3d R)
{
  // rotation magnitude
  double theta = acos((R.trace()-1)/2.0);

  // avoid numerical error with approximation
  Eigen::Vector3d delta(0,0,0);
  if (theta > 1e-6)
  {
    Eigen::Vector3d axis(R(2,1) - R(1,2),
                         R(0,2) - R(2,0),
                         R(1,0) - R(0,1));
    delta = theta/(2*sin(theta))*axis;
  }

  return delta;
}


// convert unit vector to quaternion (relative to fixed frame)
Quaternion vec2quat(const Eigen::Vector3d v)
{
  // convert to axis-angle representation
  static Eigen::Vector3d ix(1, 0, 0);
  double theta = acos(ix.dot(v));

  // avoid numerical problems
  Quaternion q;
  if (theta < 1e-6)
  {
    q.w = 1;
    q.x = 0;
    q.y = 0;
    q.z = 0;
  }
  else
  {
    // compute axis of rotation
    Eigen::Vector3d axis = ix.cross(v);
    axis /= axis.norm();

    // complex portion of quaternion
    Eigen::Vector3d qbar = axis * sin(theta/2);

    // populate quaternion components
    q.w = cos(theta/2);
    q.x = qbar(0);
    q.y = qbar(1);
    q.z = qbar(2);
  }

  return q;
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


// vector from skew symmetric matrix
Eigen::Vector3d vex(const Eigen::Matrix3d mat)
{
  Eigen::Vector3d v(mat(2,1), mat(0,2), mat(1,0));
  return v;
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


// rotation from vehicle to body frame (3-2-1 Euler)
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
    return max;
  else if (value < min)
    return min;
  else
    return value;
}


} // namespace roscopter_common


