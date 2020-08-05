#include "ekf/ekf.h"

using namespace Eigen;
using namespace xform;
using namespace quat;
using namespace std;

#define T transpose()

namespace roscopter::ekf
{

void EKF::dynamics(const State &x, const Vector6d& u, ErrorState &dx, bool calc_jac)
{
    Vector3d accel = u.head<3>() - x.ba;
    Vector3d omega = u.tail<3>() - x.bg;

    dx.p = x.q.rota(x.v);
    dx.q = omega;
    dx.v = accel + x.q.rotp(gravity) - omega.cross(x.v);
    dx.ba.setZero();
    dx.bg.setZero();
    dx.bb = 0.;
    dx.ref = 0.;

    CHECK_NAN(dx.arr);
    if (calc_jac)
    {
        Matrix3d R = x.q.R();
        typedef ErrorState DX;
        typedef meas::Imu U;

        A_.setZero();
        B_.setZero();
        A_.block<3,3>(DX::DP, DX::DQ) = -R.T * skew(x.v);
        A_.block<3,3>(DX::DP, DX::DV) = R.T;

        A_.block<3,3>(DX::DQ, DX::DQ) = -skew(omega);
        A_.block<3,3>(DX::DQ, DX::DBG) = -I_3x3;
        B_.block<3,3>(DX::DQ, U::W) = I_3x3;

        A_.block<3,3>(DX::DV, DX::DV) = -skew(omega);
        A_.block<3,3>(DX::DV, DX::DQ) = skew(x.q.rotp(gravity));
        A_.block<3,3>(DX::DV, DX::DBA) = -I_3x3;
        A_.block<3,3>(DX::DV, DX::DBG) = -skew(x.v);
        B_.block<3,3>(DX::DV, U::A) = I_3x3;
        B_.block<3,3>(DX::DV, U::W) = skew(x.v);

        CHECK_NAN(A_); CHECK_NAN(B_);
    }
}

void EKF::errorStateDynamics(const State& xhat, const ErrorState& xt, const Vector6d& u,
                             const Vector6d& eta, ErrorState& dxdot)
{
    auto eta_a = eta.head<3>();
    auto eta_w = eta.tail<3>();
    auto z_a = u.head<3>();
    auto z_w = u.tail<3>();

    State x = xhat + xt;
    Vector3d a = z_a - x.ba + eta_a;
    Vector3d ahat = z_a - xhat.ba;
    Vector3d w = z_w - x.bg + eta_w;
    Vector3d what = z_w - xhat.bg;
    dxdot.arr.setZero();
    dxdot.p = x.q.rota(x.v) - xhat.q.rota(xhat.v);
    dxdot.q = w - x.q.rotp(xhat.q.rota(what));
    dxdot.v = x.q.rotp(gravity) + a - w.cross(x.v) - (xhat.q.rotp(gravity) + ahat - what.cross(xhat.v));
}

}
