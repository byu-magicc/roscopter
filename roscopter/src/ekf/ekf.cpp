#include "ekf/ekf.h"

#define T transpose()

using namespace Eigen;

namespace roscopter
{
namespace ekf
{

const dxMat EKF::I_BIG = dxMat::Identity();

EKF::EKF() :
  xbuf_(100)
{

}

EKF::~EKF()
{
  for (int i = 0; i < NUM_LOGS; i++)
    delete logs_[i];
}

void EKF::load(const std::string &filename)
{
  // Constant Parameters
  get_yaml_eigen("p_b2g", filename, p_b2g_);
  get_yaml_diag("Qx", filename, Qx_);
  get_yaml_diag("P0", filename, P());
  P0_yaw_ = P()(ErrorState::DQ + 2, ErrorState::DQ + 2);
  get_yaml_diag("R_zero_vel", filename, R_zero_vel_);

  // Partial Update
  get_yaml_eigen("lambda", filename, lambda_vec_);
  const dxVec ones = dxVec::Constant(1.0);
  lambda_mat_ = ones * lambda_vec_.transpose() + lambda_vec_ * ones.transpose() -
                lambda_vec_ * lambda_vec_.transpose();

  // Measurement Flags
  get_yaml_node("enable_partial_update", filename, enable_partial_update_);
  get_yaml_node("enable_out_of_order", filename, enable_out_of_order_);
  get_yaml_node("use_mocap", filename, use_mocap_);
  get_yaml_node("use_gnss", filename, use_gnss_);
  get_yaml_node("use_baro", filename, use_baro_);
  get_yaml_node("use_range", filename, use_range_);
  get_yaml_node("use_zero_vel", filename, use_zero_vel_);

  // Armed Check
  get_yaml_node("enable_arm_check", filename, enable_arm_check_);
  get_yaml_node("is_flying_threshold", filename, is_flying_threshold_);

  // load initial state
  double ref_heading;
  get_yaml_node("ref_heading", filename, ref_heading);
  q_n2I_ = quat::Quatd::from_euler(0, 0, M_PI/180.0 * ref_heading);

  ref_lla_set_ = false;
  bool manual_ref_lla;
  get_yaml_node("manual_ref_lla", filename, manual_ref_lla);
  if (manual_ref_lla)
  {
    Vector3d ref_lla;
    get_yaml_eigen("ref_lla", filename, ref_lla);
    std::cout << "Set ref lla: " << ref_lla.transpose() << std::endl;
    ref_lla.head<2>() *= M_PI/180.0; // convert to rad
    xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
    x_e2I_.t() = x_e2n.t();
    x_e2I_.q() = x_e2n.q() * q_n2I_;

    // initialize the estimated ref altitude state
    x().ref = ref_lla(2);
    ref_lat_radians_ = ref_lla(0);
    ref_lon_radians_ = ref_lla(1);

    ref_lla_set_ = true;
  }

  ground_pressure_ = 0.;
  ground_temperature_ = 0.;
  update_baro_ = false;
  get_yaml_node("update_baro_velocity_threshold", filename, update_baro_vel_thresh_);

  get_yaml_eigen("x0", filename, x0_.arr());

  initLog(filename);
}

void EKF::initLog(const std::string &filename)
{
  get_yaml_node("enable_log", filename, enable_log_);
  get_yaml_node("log_prefix", filename, log_prefix_);

  std::experimental::filesystem::create_directories(log_prefix_);

  logs_.resize(NUM_LOGS);
  for (int i = 0; i < NUM_LOGS; i++)
    logs_[i] = new Logger(log_prefix_ + "/" + log_names_[i] + ".bin");
}

void EKF::initialize(double t)
{
  x().t = t;
  x().x = x0_;
  x().v.setZero();
  x().ba.setZero();
  x().bg.setZero();
  x().bb = 0.;
  if (ref_lla_set_)
    x().ref = x().ref;
  else
    x().ref = 0.;
  x().a = -gravity;
  x().w.setZero();
  is_flying_ = false;
  armed_ = false;
}

void EKF::propagate(const double &t, const Vector6d &imu, const Matrix6d &R)
{
  if (std::isnan(x().t))
  {
    initialize(t);
    return;
  }

  double dt = t - x().t;
  assert(dt >= 0);
  if (dt < 1e-6)
    return;

  dynamics(x(), imu, dx_, true);

  // do the state propagation
  xbuf_.next().x = x() + dx_ * dt;
  xbuf_.next().x.t = t;
  xbuf_.next().x.imu = imu;

  // discretize jacobians (first order)
  A_ = I_BIG + A_*dt;
  B_ = B_*dt;
  CHECK_NAN(P());
  CHECK_NAN(A_);
  CHECK_NAN(B_);
  CHECK_NAN(Qx_);
  xbuf_.next().P = A_*P()*A_.T + B_*R*B_.T + Qx_*dt*dt; // covariance propagation
  CHECK_NAN(xbuf_.next().P);
  xbuf_.advance();
  Qu_ = R; // copy because we might need it later.

  if (enable_log_)
  {
    logs_[LOG_STATE]->logVectors(x().arr, x().q.euler());
    logs_[LOG_COV]->log(x().t);
    logs_[LOG_COV]->logVectors(P());
    logs_[LOG_IMU]->log(t);
    logs_[LOG_IMU]->logVectors(imu);
  }
}

void EKF::run()
{
  meas::MeasSet::iterator nmit = getOldestNewMeas();
  if (nmit == meas_.end())
    return;

  // rewind state history
  if (!xbuf_.rewind((*nmit)->t))
    throw std::runtime_error("unable to rewind enough, expand STATE_BUF");

  // re-propagate forward, integrating measurements on the way
  while (nmit != meas_.end())
  {
    update(*nmit);
    nmit++;
  }

  // clear off measurements older than the state history
  while (xbuf_.begin().x.t > (*meas_.begin())->t)
    meas_.erase(meas_.begin());

  logState();
}

void EKF::update(const meas::Base* m)
{
  if (m->type == meas::Base::IMU)
  {
    const meas::Imu* z = dynamic_cast<const meas::Imu*>(m);
    propagate(z->t, z->z, z->R);

    if (!is_flying_)
      zeroVelUpdate(z->t);
  }
  else if (!std::isnan(x().t))
  {
    propagate(m->t, x().imu, Qu_);
    switch(m->type)
    {
    case meas::Base::GNSS:
      {
        const meas::Gnss* z = dynamic_cast<const meas::Gnss*>(m);
        gnssUpdate(*z);
        break;
      }
    case meas::Base::MOCAP:
      {
        const meas::Mocap* z = dynamic_cast<const meas::Mocap*>(m);
        mocapUpdate(*z);
        break;
      }
    default:
      break;
    }
  }
  cleanUpMeasurementBuffers();
}

meas::MeasSet::iterator EKF::getOldestNewMeas()
{
  meas::MeasSet::iterator it = meas_.begin();
  while (it != meas_.end() && (*it)->handled)
  {
    it++;
  }
  return it;
}

bool EKF::measUpdate(const VectorXd &res, const MatrixXd &R, const MatrixXd &H)
{
  int size = res.rows();
  auto K = K_.leftCols(size);

  ///TODO: perform covariance gating
  MatrixXd innov = (H*P()*H.T + R).inverse();

  CHECK_NAN(H); CHECK_NAN(R); CHECK_NAN(P());
  K = P() * H.T * innov;
  CHECK_NAN(K);

  if (enable_partial_update_)
  {
    // Apply Fixed Gain Partial update per
    // "Partial-Update Schmidt-Kalman Filter" by Brink
    // Modified to operate inline and on the manifold
    x() += lambda_vec_.asDiagonal() * K * res;
    dxMat ImKH = I_BIG - K*H;
    P() += lambda_mat_.cwiseProduct(ImKH*P()*ImKH.T + K*R*K.T - P());
  }
  else
  {
    x() += K * res;
    dxMat ImKH = I_BIG - K*H;
    P() = ImKH*P()*ImKH.T + K*R*K.T;
  }

  CHECK_NAN(P());
  return true;
}

void EKF::imuCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
  if (!is_flying_)
    checkIsFlying();

  ///TODO: make thread-safe (wrap in mutex)
  if (enable_out_of_order_)
  {
    imu_meas_buf_.push_back(meas::Imu(t, z, R));
    meas_.insert(meas_.end(), &imu_meas_buf_.back());
    run(); // For now, run on the IMU heartbeat (could be made multi-threaded)
  }
  else
  {
    propagate(t, z, R);
    if (!is_flying_)
      zeroVelUpdate(t);
  }

  if (enable_log_)
  {
    logs_[LOG_IMU]->log(t);
    logs_[LOG_IMU]->logVectors(z);
  }

}

void EKF::baroCallback(const double &t, const double &z, const double &R,
                       const double &temp)
{
  if (enable_out_of_order_)
  {
    std::cout << "ERROR OUT OF ORDER BARO NOT IMPLEMENTED" << std::endl;
  }
  else
    baroUpdate(meas::Baro(t, z, R, temp));
}

void EKF::rangeCallback(const double& t, const double& z, const double& R)
{
  if (enable_out_of_order_)
  {
    std::cout << "ERROR OUT OF ORDER RANGE NOT IMPLEMENTED" << std::endl;
  }
  else
    rangeUpdate(meas::Range(t, z, R));
}

void EKF::gnssCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
  if (!ref_lla_set_)
    return;

  if (enable_out_of_order_)
  {
    gnss_meas_buf_.push_back(meas::Gnss(t, z, R));
    meas_.insert(meas_.end(), &gnss_meas_buf_.back());
  }
  else
    gnssUpdate(meas::Gnss(t, z, R));

  if (enable_log_)
  {
    logs_[LOG_LLA]->log(t);
    logs_[LOG_LLA]->logVectors(ecef2lla((x_e2I_ * x().x).t()));
    logs_[LOG_LLA]->logVectors(ecef2lla(z.head<3>()));
  }
}

void EKF::mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R)
{
  if (enable_out_of_order_)
  {
    mocap_meas_buf_.push_back(meas::Mocap(t, z, R));
    meas_.insert(meas_.end(), &mocap_meas_buf_.back());
  }
  else
    mocapUpdate(meas::Mocap(t, z, R));


  if (enable_log_)
  {
    logs_[LOG_REF]->log(t);
    logs_[LOG_REF]->logVectors(z.arr(), z.q().euler());
  }
}

void EKF::baroUpdate(const meas::Baro &z)
{
  if (!this->groundTempPressSet())
  {
    return;
  }
  else if (!update_baro_ || !is_flying_)
  {
    // Take the lowest pressure while I'm not flying as ground pressure
    // This has the effect of hopefully underestimating my altitude instead of
    // over estimating.
    if (z.z(0) < ground_pressure_)
    {
      ground_pressure_ = z.z(0);
      std::cout << "New ground pressure: " << ground_pressure_ << std::endl;
    }

    // check if we should start updating with the baro yet based on
    // velocity estimate
    if (x().v.norm() > update_baro_vel_thresh_)
      update_baro_ = true;

    return;
  }

  using Vector1d = Eigen::Matrix<double, 1, 1>;

  // // From "Small Unmanned Aircraft: Theory and Practice" eq 7.8
  const double g = 9.80665; // m/(s^2) gravity 
  const double R = 8.31432; // universal gas constant
  const double M = 0.0289644; // kg / mol. molar mass of Earth's air

  const double altitude = -x().p(2);
  const double baro_bias = x().bb;

  // From "Small Unmanned Aircraft: Theory and Practice" eq 7.9
  // const double rho = M * ground_pressure_ / R / ground_temperature_;
  const double rho = M * ground_pressure_ / R / z.temp;

  const double press_hat = ground_pressure_ - rho * g * altitude + baro_bias;

  const Vector1d zhat(press_hat);
  Vector1d r = z.z - zhat;

  typedef ErrorState E;

  Matrix<double, 1, E::NDX> H;
  H.setZero();
  H(0, E::DP + 2) = rho * g;
  H(0, E::DBB) = 1.;

  /// TODO: Saturate r
  if (use_baro_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_BARO_RES]->log(z.t);
    logs_[LOG_BARO_RES]->logVectors(r, z.z, zhat);
    logs_[LOG_BARO_RES]->log(z.temp);
  }

}

void EKF::rangeUpdate(const meas::Range &z)
{
  // Assume that the earth is flat and that the range sensor is rigidly attached
  // to the UAV, so distance is dependent on the attitude of the UAV.
  // TODO this assumes that the laser is positioned at 0,0,0 in the body frame
  // of the UAV
  // TODO this also only updates if the UAV is pretty close to level and the
  // measurement model jacobian assumes that the measurement is not dependent
  // on roll or pitch at all
  using Vector1d = Eigen::Matrix<double, 1, 1>;

  const double altitude = -x().p(2);
  const double roll = x().q.roll();
  const double pitch = x().q.pitch();

  // const double level_threshold = 2. * M_PI / 180.; // 1 degree

  // Only update if UAV is close to level
  // if ((abs(roll) > level_threshold) || (abs(pitch) > level_threshold))
    // return;

  const Vector1d zhat(altitude / cos(roll) / cos(pitch)); // TODO roll/ pitch of drone
  Vector1d r = z.z - zhat; // residual

  // std::cout << "Laser Update: " << std::endl;
  // std::cout << "Altitude meas: " << z.z(0) << std::endl;
  // std::cout << "Altitude est: " << altitude << std::endl;

  typedef ErrorState E;

  Matrix<double, 1, E::NDX> H;
  H.setZero();
  H(0, E::DP + 2) = -1.;

  // Vector1d r_saturated
  double r_sat = 0.1;
  if (abs(r(0)) > r_sat)
  {
    double r_sign = (r(0) > 0) - (r(0) < 0);
    r(0) = r_sat * r_sign;
  }

  // TODO: Saturate r
  if (use_range_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_RANGE_RES]->log(z.t);
    logs_[LOG_RANGE_RES]->logVectors(r, z.z, zhat);
  }

}

void EKF::gnssUpdate(const meas::Gnss &z)
{
  const Vector3d w = x().w - x().bg;
  const Vector3d gps_pos_I = x().p + x().q.rota(p_b2g_);
  const Vector3d gps_vel_b = x().v + w.cross(p_b2g_);
  const Vector3d gps_vel_I = x().q.rota(gps_vel_b);

  // Update ref_lla based on current estimate
  Vector3d ref_lla(ref_lat_radians_, ref_lon_radians_, x().ref);
  xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
  x_e2I_.t() = x_e2n.t();
  x_e2I_.q() = x_e2n.q() * q_n2I_;

  Vector6d zhat;
  zhat << x_e2I_.transforma(gps_pos_I),
          x_e2I_.rota(gps_vel_I);
  const Vector6d r = z.z - zhat; // residual

  const Matrix3d R_I2e = x_e2I_.q().R().T;
  const Matrix3d R_b2I = x().q.R().T;
  const Matrix3d R_e2b = R_I2e * R_b2I;

  const double sin_lat = sin(ref_lat_radians_);
  const double cos_lat = cos(ref_lat_radians_);
  const double sin_lon = sin(ref_lon_radians_);
  const double cos_lon = cos(ref_lon_radians_);
  const Vector3d dpEdRefAlt(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);

  typedef ErrorState E;

  Matrix<double, 6, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = R_I2e; // dpE/dpI
  H.block<3,3>(0, E::DQ) = -R_e2b * skew(p_b2g_);
  H.block<3, 1>(0, E::DREF) = dpEdRefAlt;
  H.block<3,3>(3, E::DQ) = -R_e2b * skew(gps_vel_b); // dvE/dQI
  H.block<3,3>(3, E::DV) = R_e2b;
  H.block<3,3>(3, E::DBG) = R_e2b * skew(p_b2g_);

  /// TODO: Saturate r
  if (use_gnss_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_GNSS_RES]->log(z.t);
    logs_[LOG_GNSS_RES]->logVectors(r, z.z, zhat);
  }
}

void EKF::mocapUpdate(const meas::Mocap &z)
{
  xform::Xformd zhat = x().x;

  // TODO Do we need to fix "-" operator for Xformd?
  // Right now using piecewise subtraction
  // on position and attitude separately. This may be correct though because
  // our state is represented as R^3 x S^3 (position, quaterion) not SE3
  Vector6d r;
  r.segment<3>(0) = z.z.t_ - zhat.t_;
  r.segment<3>(3) = z.z.q_ - zhat.q_;

  typedef ErrorState E;
  Matrix<double, 6, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = I_3x3;
  H.block<3,3>(3, E::DQ) = I_3x3;

  /// TODO: Saturate r
  if (use_mocap_)
  {
    measUpdate(r, z.R, H);
  }

  if (enable_log_)
  {
    logs_[LOG_MOCAP_RES]->log(z.t);
    logs_[LOG_MOCAP_RES]->logVectors(r, z.z.arr(), zhat.arr());
  }
}

void EKF::zeroVelUpdate(double t)
{
  // Update Zero velocity and zero altitude
  typedef ErrorState E;
  Matrix<double, 4, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DV) = I_3x3;
  H(3, E::DP + 2) = 1.;

  Vector4d r;
  r.head<3>() = -x().v;
  r(3) = -x().p(2);

  if (use_zero_vel_)
    measUpdate(r, R_zero_vel_, H);

  // Reset the uncertainty in yaw
  P().block<ErrorState::SIZE, 1>(0, ErrorState::DQ + 2).setZero();
  P().block<1, ErrorState::SIZE>(ErrorState::DQ + 2, 0).setZero();
  P()(ErrorState::DQ + 2, ErrorState::DQ + 2) = P0_yaw_;

  if (enable_log_)
  {
    logs_[LOG_ZERO_VEL_RES]->log(t);
    logs_[LOG_ZERO_VEL_RES]->logVectors(r);
  }
}

void EKF::setRefLla(Vector3d ref_lla)
{
  if (ref_lla_set_)
    return;

  std::cout << "Set ref lla: " << ref_lla.transpose() << std::endl;
  ref_lla.head<2>() *= M_PI/180.0; // convert to rad
  xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
  x_e2I_.t() = x_e2n.t();
  x_e2I_.q() = x_e2n.q() * q_n2I_;

  // initialize the estimated ref altitude state
  x().ref = ref_lla(2);
  ref_lat_radians_ = ref_lla(0);
  ref_lon_radians_ = ref_lla(1);

  ref_lla_set_ = true;

}

void EKF::cleanUpMeasurementBuffers()
{
  // Remove all measurements older than our oldest state in the measurement buffer
  while ((*meas_.begin())->t < xbuf_.begin().x.t)
    meas_.erase(meas_.begin());
  while (imu_meas_buf_.front().t < xbuf_.begin().x.t)
    imu_meas_buf_.pop_front();
  while (mocap_meas_buf_.front().t < xbuf_.begin().x.t)
    mocap_meas_buf_.pop_front();
  while (gnss_meas_buf_.front().t < xbuf_.begin().x.t)
    gnss_meas_buf_.pop_front();
}

void EKF::setGroundTempPressure(const double& temp, const double& press)
{
  ground_temperature_ = temp;
  ground_pressure_ = press;
}

void EKF::checkIsFlying()
{
  bool okay_to_check = enable_arm_check_ ? armed_ : true;
  if (okay_to_check && x().a.norm() > is_flying_threshold_)
  {
    std::cout << "Now Flying!  Go Go Go!" << std::endl;
    is_flying_ = true;
  }
}


}
}
