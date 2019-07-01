#include "ekf/ekf.h"

#define T transpose()

using namespace Eigen;

namespace roscopter
{
namespace ekf
{

EKF::EKF() :
  xbuf_(100),
  I_Big_(dxMat::Identity())
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

  // Measurement Flags
  get_yaml_node("enable_out_of_order", filename, enable_out_of_order_);
  get_yaml_node("use_truth", filename, use_truth_);
  get_yaml_node("use_gnss", filename, use_gnss_);
  get_yaml_node("use_alt", filename, use_alt_);

  // load initial state
  Vector3d ref_lla;
  double ref_heading;
  get_yaml_eigen("ref_lla", filename, ref_lla);
  get_yaml_node("ref_heading", filename, ref_heading);
  ref_lla.head<2>() *= M_PI/180.0; // convert to rad
  quat::Quatd q_n2I = quat::Quatd::from_euler(0, 0, M_PI/180.0 * ref_heading);
  xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
  x_e2I_.t() = x_e2n.t();
  x_e2I_.q() = x_e2n.q() * q_n2I;
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
  x().a = -gravity;
  x().w.setZero();
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
  A_ = I_Big_ + A_*dt;
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
    logs_[LOG_STATE]->logVectors(x().arr);
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

  ///TODO: Partial Update
  x() += K * res;
  dxMat ImKH = I_Big_ - K*H;
  P() = ImKH*P()*ImKH.T + K*R*K.T;

  CHECK_NAN(P());
  return false;
}

void EKF::imuCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
  ///TODO: make thread-safe (wrap in mutex)
  if (enable_out_of_order_)
  {
    imu_meas_buf_.push_back(meas::Imu(t, z, R));
    meas_.insert(meas_.end(), &imu_meas_buf_.back());
    run(); // For now, run on the IMU heartbeat (could be made multi-threaded)
  }
  else
    propagate(t, z, R);

  if (enable_log_)
  {
    logs_[LOG_IMU]->log(t);
    logs_[LOG_IMU]->logVectors(z);
  }

}

void EKF::gnssCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
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
    logs_[LOG_REF]->logVectors(z.arr());
  }
}


void EKF::gnssUpdate(const meas::Gnss &z)
{
  Vector3d w = x().w - x().bg;
  Vector3d gps_pos_I = x().p + x().q.rota(p_b2g_);
  Vector3d gps_vel_b = x().v + w.cross(p_b2g_);
  Vector3d gps_vel_I = x().q.rota(gps_vel_b);

  Vector6d zhat;
  zhat << x_e2I_.transforma(gps_pos_I),
          x_e2I_.rota(gps_vel_I);
  Vector6d r = z.z - zhat; // residual

  Matrix3d R_I2e = x_e2I_.q().R().T;
  Matrix3d R_b2I = x().q.R().T;
  Matrix3d R_e2b = R_I2e * R_b2I;

  typedef ErrorState E;

  Matrix<double, 6, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = R_I2e; // dpE/dpI
  H.block<3,3>(0, E::DQ) = -R_e2b * skew(p_b2g_);
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
  Vector6d r = zhat - z.z;

  typedef ErrorState E;
  Matrix<double, 6, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = I_3x3;
  H.block<3,3>(3, E::DQ) = I_3x3; // Check this, it's probably wrong

  /// TODO: Saturate r
  if (use_truth_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_MOCAP_RES]->log(z.t);
    logs_[LOG_MOCAP_RES]->logVectors(r, z.z.arr(), zhat.arr());
  }
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


}
}
