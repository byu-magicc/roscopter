#pragma once

#include <deque>
#include <functional>

#include <Eigen/Core>
#include <geometry/xform.h>

#include "ekf/state.h"
#include "ekf/meas.h"
#include "logger.h"

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
    throw std::runtime_error(#mat " Has NaNs" + std::to_string(__LINE__));\
}

namespace ekf
{


static const Eigen::Vector3d gravity = (Eigen::Vector3d() << 0, 0, 9.80665).finished();

class EKF
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EKF();

    State& x() { return xbuf_.x(); } // The current state object
    const State& x() const { return xbuf_.x(); }
    dxMat& P() { return xbuf_.P(); } // The current covariance
    const dxMat& P() const { return xbuf_.P(); }

    void load(const std::string& filename);
    void run();
    void update(const meas::Base *m);

    bool measUpdate(const Eigen::VectorXd &res, const Eigen::MatrixXd &R, const Eigen::MatrixXd &H);
    void propagate(const double& t, const Vector6d &imu, const Matrix6d &R);
    void dynamics(const State &x, const Vector6d& u, ErrorState &dx, bool calc_jac=false);
    void errorStateDynamics(const State& x, const ErrorState& dx, const Vector6d& u,
                            const Vector6d& eta, ErrorState& dxdot);

    meas::MeasSet::iterator getOldestNewMeas();
    void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R);
    void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R);
    void mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R);

    void gnssUpdate(const meas::Gnss &z);
    void mocapUpdate(const meas::Mocap &z);

    void cleanUpMeasurementBuffers();


    void initLog();
    void logState();
    void logCov();
    enum {
        LOG_STATE,
        LOG_COV,
        LOG_ZV_RES,
        LOG_IMU
    };
    std::vector<std::string> log_names_ {
        "state",
        "cov",
        "gnss_res",
        "imu"
    };
    std::vector<Logger*> logs_;
    std::string log_prefix_;

    // Constants
    Eigen::Vector3d p_b2g_;

    // Matrix Workspace
    dxMat A_;
    dxMat Qx_;
    dxMat I_Big_;
    dxuMat B_;
    dxuMat K_;
    duMat Qu_;
    ErrorState dx_;

    // State buffer
    StateBuf xbuf_;

    // Measurement Buffers
    meas::MeasSet meas_;
    std::deque<meas::Imu, Eigen::aligned_allocator<meas::Imu>> imu_meas_buf_;
    std::deque<meas::Mocap, Eigen::aligned_allocator<meas::Mocap>> mocap_meas_buf_;
    std::deque<meas::Gnss, Eigen::aligned_allocator<meas::Gnss>> gnss_meas_buf_;
    std::deque<meas::ZeroVel, Eigen::aligned_allocator<meas::ZeroVel>> zv_meas_buf_;
};

}

}
