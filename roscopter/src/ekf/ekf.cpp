#include "ekf/ekf.h"

#define T transpose()

using namespace Eigen;

namespace roscopter
{
namespace ekf
{

void EKF::propagate(const double &t, const Vector6d &imu, const Matrix6d &R)
{
    double dt = t - x().t;
    assert(dt >= 0);
    if (dt < 1e-6)
        return;

    dynamics(x(), imu, dx_, true);

    // do the state propagation
    xbuf_.next().x = x() + dx_ * dt;
    xbuf_.next().x.t = t;
    xbuf_.next().x.imu = imu;
    xbuf_.next().x.x_e2I = x().x_e2I;

    // discretize jacobians (first order)
    A_ = I_Big_ + A_*dt;
    B_ = B_*dt;
    CHECK_NAN(P());
    CHECK_NAN(A_);
    CHECK_NAN(B_);
    CHECK_NAN(Qu_);
    CHECK_NAN(Qx_);
    xbuf_.next().P = A_*P()*A_.T + B_*Qu_*B_.T + Qx_*dt*dt; // covariance propagation
    CHECK_NAN(xbuf_.next().P);
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
    else
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
    MatrixXd ImKH = I_Big_.topLeftCorner(size, size) - K*H;
    P() = ImKH*P()*ImKH.T + K*R*K.T;

    CHECK_NAN(P());
    return false;
}

void EKF::imuCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
    ///TODO: make thread-safe (wrap in mutex)
    imu_meas_buf_.push_back(meas::Imu(t, z, R));
    meas_.insert(meas_.end(), &imu_meas_buf_.back());

    run(); // For now, run on the IMU heartbeat (could be made multi-threaded)
}

void EKF::gnssCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
    gnss_meas_buf_.push_back(meas::Gnss(t, z, R));
    meas_.insert(meas_.end(), &gnss_meas_buf_.back());
}

void EKF::mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R)
{
    mocap_meas_buf_.push_back(meas::Mocap(t, z, R));
    meas_.insert(meas_.end(), &mocap_meas_buf_.back());
}


void EKF::gnssUpdate(const meas::Gnss &z)
{
    Vector3d w = x().w - x().bg;
    Vector3d gps_pos_I = x().p + x().q.rota(p_b2g_);
    Vector3d gps_vel_b = x().v + w.cross(p_b2g_);
    Vector3d gps_vel_I = x().q.rota(gps_vel_b);

    Vector6d zhat;
    zhat << x().x_e2I.transforma(gps_pos_I),
            x().x_e2I.rota(gps_vel_I);
    Vector6d r = z.z - zhat; // residual

    Matrix3d R_I2e = x().x_e2I.q().R().T;
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
    measUpdate(r, z.R, H);
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
    measUpdate(r, z.R, H);
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
