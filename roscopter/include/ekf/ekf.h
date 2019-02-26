#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include "unsupported/Eigen/MatrixFunctions"

#include <deque>
#include <set>
#include <map>
#include <functional>
#include <fstream>
#include <chrono>
#include <iostream>
#include <random>
#include <tuple>

#include "geometry/quat.h"
#include "geometry/xform.h"

using namespace quat;
using namespace xform;
using namespace std;
using namespace Eigen;

#define NO_NANS(mat) (mat.array() == mat.array()).all()

#ifndef NDEBUG
#define NAN_CHECK if (NaNsInTheHouse()) { std::cout << "NaNs In The House at line " << __LINE__ << "!!!\n"; exit(0); }
#define CHECK_MAT_FOR_NANS(mat) if ((K_.array() != K_.array()).any()) { std::cout << "NaN detected in " << #mat << " at line " << __LINE__ << "!!!\n" << mat << "\n"; exit(0); }
#else
#define NAN_CHECK {}
#define CHECK_MAT_FOR_NANS(mat) {}
#endif

#define MAX_X 17
#define MAX_DX 16

#define LEN_STATE_HIST 250
#define LEN_MEAS_HIST 2

typedef Matrix<double, MAX_X, 1> xVector;
typedef Matrix<double, MAX_DX, 1> dxVector;
typedef Matrix<double, MAX_X, MAX_X> xMatrix;
typedef Matrix<double, MAX_DX, MAX_DX> dxMatrix;
typedef Matrix<double, MAX_DX, 6> dxuMatrix;
typedef Matrix<double, 6, 1> uVector;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 4, 1> zVector;
typedef Matrix<double, 3, MAX_DX> hMatrix;

namespace roscopter
{

class EKF;

typedef void (EKF::*measurement_function_ptr)(const xVector& x, zVector& h, hMatrix& H) const;

static const Vector3d gravity = [] {
  Vector3d tmp;
  tmp << 0, 0, 9.80665;
  return tmp;
}();

static const Vector3d khat = [] {
  Vector3d tmp;
  tmp << 0, 0, 1.0;
  return tmp;
}();

class EKF
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html

  enum : int{
    xPOS = 0,
    xVEL = 3,
    xATT = 6,
    xB_A = 10,
    xB_G = 13,
    xMU = 16,
    xZ = 17
  };

  enum : int{
    uA = 0,
    uG = 3,
    uTOTAL = 6
  };

  enum : int {
    dxPOS = 0,
    dxVEL = 3,
    dxATT = 6,
    dxB_A = 9,
    dxB_G = 12,
    dxMU = 15,
    dxZ = 16
  };

  typedef enum {
    ACC,
    ALT,
    ATT,
    POS,
    VEL,
    TOTAL_MEAS
  } measurement_type_t;

  typedef enum {
    MEAS_SUCCESS,
    MEAS_GATED,
    MEAS_NAN,
    MEAS_INVALID,
    MEAS_NEW_FEATURE
  } meas_result_t;

private:
  typedef enum {
    LOG_STATE = TOTAL_MEAS,
    LOG_FEATURE_IDS,
    LOG_INPUT,
    LOG_XDOT,
    LOG_GLOBAL,
    LOG_CONF,
    LOG_KF,
    LOG_DEBUG,
    TOTAL_LOGS
  } log_type_t;


  // State, Covariance History
  int i_;
  std::vector<xVector, aligned_allocator<xVector>> x_;
  std::vector<dxMatrix, aligned_allocator<dxMatrix>> P_;
  std::vector<double> t_;

  // Input Buffer
  typedef std::deque<std::pair<double, uVector>, aligned_allocator<std::pair<double, uVector>>> uBuf;
  uBuf u_;

  // Measurement Buffer
  typedef struct
  {
    double t;
    measurement_type_t type;
    zVector z;
    Matrix3d R;
    bool active;
    int zdim;
    int rdim;
    bool handled;
  } measurement_t;
  typedef std::deque<measurement_t, aligned_allocator<measurement_t>> zBuf;
  zBuf zbuf_;

  // State and Covariance and Process Noise Matrices
  dxMatrix Qx_;
  Matrix<double, 6, 6> Qu_;

  // Partial Update Gains
  dxVector lambda_;
  dxMatrix Lambda_;

  // Internal bookkeeping variables
  double start_t_;

  // Matrix Workspace
  dxMatrix A_;
  dxuMatrix G_;
  dxVector dx_;
  const dxMatrix I_big_ = dxMatrix::Identity();
  const dxMatrix Ones_big_ = dxMatrix::Constant(1.0);
  const dxVector dx_ones_ = dxVector::Constant(1.0);
  xVector xp_;
  Matrix<double, MAX_DX, 3>  K_;
  zVector zhat_;
  hMatrix H_;

  // EKF Configuration Parameters
  bool use_drag_term_;
  bool partial_update_;
  double gating_threshold_;

  // Log Stuff
  std::vector<std::ofstream>* log_ = nullptr;

public:

  EKF();
  ~EKF();
#ifdef MC_SIM
  void load(string ekf_file, string common_file, bool use_logger=true, string prefix="");
#endif
  void init(Matrix<double, xZ,1> &x0, Matrix<double, dxZ,1> &P0, Matrix<double, dxZ,1> &Qx,
            Matrix<double, dxZ,1> &lambda, uVector &Qu, std::string log_directory, bool use_drag_term,
            bool partial_update, int cov_prop_skips, double gating_threshold, string prefix="");

  inline double now() const
  {
    std::chrono::microseconds now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
    return (double)now.count()*1e-6;
  }

  // Errors
  bool NaNsInTheHouse() const;
  bool BlowingUp() const;

  // Getters and Setters
  const xVector &get_state() const;
  const dxMatrix &get_covariance() const;
  const dxVector get_covariance_diagonal() const;

  void set_x0(const Matrix<double, xZ, 1>& _x0);
  void set_imu_bias(const Vector3d& b_g, const Vector3d& b_a);
  void set_drag_term(const bool use_drag_term) {use_drag_term_ = use_drag_term;}
  bool get_drag_term() const {return use_drag_term_;}

  // State Propagation
  void boxplus(const xVector &x, const dxVector &dx, xVector &out) const;
  void boxminus(const xVector& x1, const xVector &x2, dxVector& out) const;
  void step(const uVector& u, const double t);
  void propagate_state(const uVector& u, const double t, bool save_input=true);
  void dynamics(const xVector &x, const uVector& u, dxVector& xdot, dxMatrix& dfdx, dxuMatrix& dfdu);
  void dynamics(const xVector &x, const uVector& u, bool state = true, bool jac = true);

  // Measurement Updates
  void handle_measurements(std::vector<int> *gated_feature_ids=nullptr);
  meas_result_t add_measurement(const double t, const VectorXd& z, const measurement_type_t& meas_type, const MatrixXd& R, bool active=true);
  meas_result_t update(measurement_t &meas);
  void h_acc(const xVector& x, zVector& h, hMatrix& H) const;
  void h_alt(const xVector& x, zVector& h, hMatrix& H) const;
  void h_att(const xVector& x, zVector& h, hMatrix& H) const;
  void h_pos(const xVector& x, zVector& h, hMatrix& H) const;
  void h_vel(const xVector& x, zVector& h, hMatrix& H) const;

  // Logger
  void log_state(const double t, const xVector& x, const dxVector& P, const uVector& u, const dxVector& dx);
  void log_measurement(const measurement_type_t type, const double t, const int dim, const MatrixXd& z, const MatrixXd& zhat, const bool active);
  void init_logger(std::string root_filename, string prefix="");
  void disable_logger();
};

static std::vector<std::string> measurement_names = [] {
  std::vector<std::string> tmp;
  tmp.resize(EKF::TOTAL_MEAS);
  tmp[EKF::ACC] = "ACC";
  tmp[EKF::ALT] = "ALT";
  tmp[EKF::ATT] = "ATT";
  tmp[EKF::POS] = "POS";
  tmp[EKF::VEL] = "VEL";
  return tmp;
}();

static std::vector<measurement_function_ptr> measurement_functions = [] {
  std::vector<measurement_function_ptr> tmp;
  tmp.resize(EKF::TOTAL_MEAS);
  tmp[EKF::ACC] = &EKF::h_acc;
  tmp[EKF::ALT] = &EKF::h_alt;
  tmp[EKF::ATT] = &EKF::h_att;
  tmp[EKF::POS] = &EKF::h_pos;
  tmp[EKF::VEL] = &EKF::h_vel;
  return tmp;
}();

}




