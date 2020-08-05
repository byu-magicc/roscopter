#pragma once

#include <deque>
#include <functional>

#include <Eigen/Core>
#include <geometry/xform.h>

#include "ekf/state.h"
#include "ekf/meas.h"
#include "roscopter_utils/logger.h"
#include "roscopter_utils/gnss.h"
#include "roscopter_utils/yaml.h"

/// TO ADD A NEW MEASUREMENT
/// Add a new Meas type to the meas.h header file and meas.cpp
/// Add a new callback like mocapCallback()...
/// Add a new update function like mocapUpdate()...
/// Add new cases to the update function
/// Profit.

/// TO ADD A NEW STATE
/// Add an index in the ErrorState and State objects in state.cpp/state.h
/// Make sure the SIZE enums are correct
/// Add relevant Jacobians and Dynamics to measurement update functions and dynamics
/// Profit.

namespace roscopter
{

#define CHECK_NAN(mat) \
  if ((mat.array() != mat.array()).any())\
{\
  std::cout << #mat << std::endl << mat << std::endl;\
  throw std::runtime_error(#mat " Has NaNs" + std::to_string(__LINE__));\
}

#define PRINTMAT(mat) std::cout << #mat << std::endl << mat << std::endl;

namespace ekf
{


static const Eigen::Vector3d gravity = (Eigen::Vector3d() << 0, 0, 9.80665).finished();

class EKF
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const dxMat I_BIG;

  EKF();
  ~EKF();

  State& x() { return xbuf_.x(); } // The current state object
  const State& x() const { return xbuf_.x(); }
  dxMat& P() { return xbuf_.P(); } // The current covariance
  const dxMat& P() const { return xbuf_.P(); }

  void initialize(double t);
  void load(const std::string& filename);
  void initLog(const std::string& filename);

  void run();
  void update(const meas::Base *m);

  void setArmed() { armed_ = true; }
  void setDisarmed() { armed_ = false; }

  bool refLlaSet() { return ref_lla_set_; }

  void setGroundTempPressure(const double& temp, const double& press);
  bool groundTempPressSet() { return (ground_pressure_ != 0) && (ground_temperature_ != 0); }

  bool isFlying() { return is_flying_; }
  void checkIsFlying();

  bool measUpdate(const Eigen::VectorXd &res, const Eigen::MatrixXd &R, const Eigen::MatrixXd &H);
  void propagate(const double& t, const Vector6d &imu, const Matrix6d &R);
  void dynamics(const State &x, const Vector6d& u, ErrorState &dx, bool calc_jac=false);
  void errorStateDynamics(const State& x, const ErrorState& dx, const Vector6d& u,
                          const Vector6d& eta, ErrorState& dxdot);

  meas::MeasSet::iterator getOldestNewMeas();
  void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R);
  void baroCallback(const double& t, const double& z, const double& R,
                    const double& temp);
  void rangeCallback(const double& t, const double& z, const double& R);
  void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R);
  void mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R);

  void baroUpdate(const meas::Baro &z);
  void rangeUpdate(const meas::Range &z);
  void gnssUpdate(const meas::Gnss &z);
  void mocapUpdate(const meas::Mocap &z);
  void zeroVelUpdate(double t);

  void setRefLla(Eigen::Vector3d ref_lla);

  void cleanUpMeasurementBuffers();

  void initLog();
  void logState();
  void logCov();
  enum {
    LOG_STATE,
    LOG_COV,
    LOG_GNSS_RES,
    LOG_MOCAP_RES,
    LOG_ZERO_VEL_RES,
    LOG_BARO_RES,
    LOG_RANGE_RES,
    LOG_IMU,
    LOG_LLA,
    LOG_REF,
    NUM_LOGS
  };
  std::vector<std::string> log_names_ {
    "state",
    "cov",
    "gnss_res",
    "mocap_res",
    "zero_vel_res",
    "baro_res",
    "range_res",
    "imu",
    "lla",
    "ref"
  };
  bool enable_log_;
  std::vector<Logger*> logs_;
  std::string log_prefix_;

  double is_flying_threshold_;
  bool enable_arm_check_;
  bool is_flying_;
  bool armed_;

  // Constants
  xform::Xformd x0_;
  Eigen::Vector3d p_b2g_;
  xform::Xformd x_e2I_;
  quat::Quatd q_n2I_;
  Eigen::Matrix4d R_zero_vel_;

  bool ref_lla_set_;
  double ref_lat_radians_;
  double ref_lon_radians_;
  double P0_yaw_;
  double ground_pressure_;
  double ground_temperature_;
  bool update_baro_;
  double update_baro_vel_thresh_;

  // Matrix Workspace
  dxMat A_;
  dxMat Qx_;
  Matrix6d Qu_;
  dxuMat B_;
  dxuMat K_;
  ErrorState dx_;

  // Partial Update
  dxVec lambda_vec_;
  dxMat lambda_mat_;

  // State buffer
  StateBuf xbuf_;

  // Measurement Buffers
  bool use_mocap_;
  bool use_baro_;
  bool use_range_;
  bool use_gnss_;
  bool use_zero_vel_;
  bool enable_out_of_order_;
  bool enable_partial_update_;
  meas::MeasSet meas_;
  std::deque<meas::Imu, Eigen::aligned_allocator<meas::Imu>> imu_meas_buf_;
  std::deque<meas::Mocap, Eigen::aligned_allocator<meas::Mocap>> mocap_meas_buf_;
  std::deque<meas::Gnss, Eigen::aligned_allocator<meas::Gnss>> gnss_meas_buf_;
  std::deque<meas::ZeroVel, Eigen::aligned_allocator<meas::ZeroVel>> zv_meas_buf_;
};

}

}
