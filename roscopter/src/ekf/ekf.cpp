#include "ekf/ekf.h"

namespace roscopter
{

EKF::EKF(){}

void EKF::init(Matrix<double, xZ,1>& x0, Matrix<double, dxZ,1> &P0, Matrix<double, dxZ,1> &Qx,
               Matrix<double, dxZ,1> &lambda, uVector &Qu, std::string log_directory, bool use_drag_term,
               bool partial_update, int cov_prop_skips, double gating_threshold, std::string prefix)
{
  x_.resize(LEN_STATE_HIST);
  P_.resize(LEN_STATE_HIST);
  t_.resize(LEN_STATE_HIST, -1);

  u_.clear();
  zbuf_.clear();

  i_ = 0;

  xp_.setZero();
  Qx_.setZero();
  x_[i_].block<(int)xZ, 1>(0,0) = x0;
  P_[i_].block<(int)dxZ, (int)dxZ>(0,0) = P0.asDiagonal();
  Qx_.block<(int)dxZ, (int)dxZ>(0,0) = Qx.asDiagonal();
  lambda_.block<(int)dxZ, 1>(0,0) = lambda;
  
  Qu_ = Qu.asDiagonal();
  
  Lambda_ = dx_ones_ * lambda_.transpose() + lambda_*dx_ones_.transpose() - lambda_*lambda_.transpose();
  
  use_drag_term_ = use_drag_term;
  partial_update_ = partial_update;
  start_t_ = NAN; // indicate that we need to initialize the filter
    
  gating_threshold_ = gating_threshold;
  
  if (log_directory.compare("~") != 0)
    init_logger(log_directory);
  
  K_.setZero();
  H_.setZero();
}

void EKF::set_x0(const Matrix<double, xZ, 1>& _x0)
{
  x_[i_].topRows(xZ) = _x0;
}


EKF::~EKF()
{
  if (log_)
  {
    for (std::vector<std::ofstream>::iterator it=log_->begin(); it!=log_->end(); ++it)
    {
      (*it) << endl;
      (*it).close();
    }
  }
}

void EKF::set_imu_bias(const Vector3d& b_g, const Vector3d& b_a)
{
  x_[i_].block<3,1>((int)xB_G,0) = b_g;
  x_[i_].block<3,1>((int)xB_A,0) = b_a;
}


const xVector& EKF::get_state() const
{
  return x_[i_];
}


const dxMatrix& EKF::get_covariance() const
{
  return P_[i_];
}

const dxVector EKF::get_covariance_diagonal() const
{
  dxVector ret = P_[i_].diagonal();
  return ret;
}

void EKF::propagate_state(const uVector &u, const double t, bool save_input)
{
  if (save_input)
  {
    u_.push_front(std::pair<double, uVector>{t, u});
  }

  if (std::isnan(start_t_))
  {
    start_t_ = t;
    t_[i_] = t;
    return;
  }

  double dt = t - t_[i_];
  if (dt < 1e-6)
    return;

  NAN_CHECK;

  // Calculate Dynamics and Jacobians
  dynamics(x_[i_], u, true, true);

  NAN_CHECK;
  int ip = (i_ + 1) % LEN_STATE_HIST; // next state history index

  // Propagate State and Covariance
  boxplus(x_[i_], dx_*dt, x_[ip]);
  A_ = I_big_ + A_*dt;
  P_[ip] = A_ * P_[i_]* A_.transpose() + G_ * Qu_ * G_.transpose() + Qx_;
  t_[ip] = t;
  i_ = ip;

  NAN_CHECK;  
  
  log_state(t, x_[i_], P_[i_].diagonal(), u, dx_);
}


bool EKF::NaNsInTheHouse() const
{
  if ((x_[i_].array() != x_[i_].array()).any() || (P_[i_].array() != P_[i_].array()).any())
  {
    std::cout << "x:\n" << x_[i_] << "\n";
    std::cout << "P:\n" << P_[i_] << "\n";
    return true;
  }
  else
    return false;
}

bool EKF::BlowingUp() const
{
  if ( ((x_[i_]).array() > 1e6).any() || ((P_[i_]).array() > 1e6).any())
    return true;
  else
    return false;
}



}








