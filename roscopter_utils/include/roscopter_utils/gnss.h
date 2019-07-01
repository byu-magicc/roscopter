#pragma once

#include <Eigen/Core>
#include "geometry/xform.h"

namespace roscopter
{

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3;
static constexpr double A = 6378137.0;       // WGS-84 Earth semimajor axis (m)
static constexpr double B = 6356752.314245;  // Derived Earth semiminor axis (m)
static constexpr double F = (A - B) / A;     // Ellipsoid Flatness
static constexpr double F_INV = 1.0 / F;     // Inverse flattening
static constexpr double A2 = A * A;
static constexpr double B2 = B * B;
static constexpr double E2 = F * (2 - F);    // Square of Eccentricity

static void ecef2lla(const Eigen::Vector3d& ecef, Eigen::Vector3d& lla)
{
    static const double e2 = F * (2.0 - F);

    double r2 = ecef.x()*ecef.x() + ecef.y()*ecef.y();
    double z=ecef.z();
    double v;
    double zk;
    do
    {
        zk = z;
        double sinp = z / std::sqrt(r2 + z*z);
        v = A / std::sqrt(1.0 - e2*sinp*sinp);
        z = ecef.z() + v*e2*sinp;
    }
    while (std::abs(z - zk) >= 1e-4);

    lla.x() = r2 > 1e-12 ? std::atan(z / std::sqrt(r2)) : (ecef.z() > 0.0 ? M_PI/2.0 : -M_PI/2.0);
    lla.y() = r2 > 1e-12 ? std::atan2(ecef.y(), ecef.x()) : 0.0;
    lla.z() = std::sqrt(r2+z*z) - v;
}

static Eigen::Vector3d ecef2lla(const Eigen::Vector3d& ecef)
{
    Eigen::Vector3d lla;
    ecef2lla(ecef, lla);
    return lla;
}

static void lla2ecef(const Eigen::Vector3d& lla, Eigen::Vector3d& ecef)
{
    double sinp=sin(lla[0]);
    double cosp=cos(lla[0]);
    double sinl=sin(lla[1]);
    double cosl=cos(lla[1]);
    double e2=F*(2.0-F);
    double v=A/sqrt(1.0-e2*sinp*sinp);

    ecef[0]=(v+lla[2])*cosp*cosl;
    ecef[1]=(v+lla[2])*cosp*sinl;
    ecef[2]=(v*(1.0-e2)+lla[2])*sinp;
}

static Eigen::Vector3d lla2ecef(const Eigen::Vector3d& lla)
{
    Eigen::Vector3d ecef;
    lla2ecef(lla, ecef);
    return ecef;
}

static quat::Quatd q_e2n(const Eigen::Vector3d& lla)
{
    quat::Quatd q1, q2;
    q1 = quat::Quatd::from_axis_angle(e_z, lla(1));
    q2 = quat::Quatd::from_axis_angle(e_y, -M_PI/2.0 - lla(0));
    return q1 * q2;
}

static void x_ecef2ned(const Eigen::Vector3d& ecef, xform::Xformd& X_e2n)
{
    X_e2n.q() = q_e2n(ecef2lla(ecef));
    X_e2n.t() = ecef;
}

static xform::Xformd x_ecef2ned(const Eigen::Vector3d& ecef)
{
    xform::Xformd X_e2n;
    x_ecef2ned(ecef, X_e2n);
    return X_e2n;
}

static Eigen::Vector3d ned2ecef(const xform::Xformd x_e2n, const Eigen::Vector3d& ned)
{
    return x_e2n.transforma(ned);
}

static void ned2ecef(const xform::Xformd x_e2n, const Eigen::Vector3d& ned, Eigen::Vector3d& ecef)
{
    ecef = x_e2n.transforma(ned);
}

static Eigen::Vector3d ecef2ned(const xform::Xformd x_e2n, const Eigen::Vector3d& ecef)
{
    return x_e2n.transformp(ecef);
}

static void ecef2ned(const xform::Xformd x_e2n, const Eigen::Vector3d& ecef, Eigen::Vector3d& ned)
{
    ned = x_e2n.transformp(ecef);
}

static void lla2ned(const Eigen::Vector3d& lla0, const Eigen::Vector3d& lla, Eigen::Vector3d& ned)
{
    xform::Xformd x_e2n;
    x_e2n.q() = q_e2n(lla0);
    x_e2n.t() = lla2ecef(lla0);
    ecef2ned(x_e2n, lla2ecef(lla), ned);
}

static Eigen::Vector3d lla2ned(const Eigen::Vector3d& lla0, const Eigen::Vector3d& lla)
{
    Eigen::Vector3d ned;
    lla2ned(lla0, lla, ned);
    return ned;
}

static void ned2lla(const Eigen::Vector3d& lla0, const Eigen::Vector3d& ned, Eigen::Vector3d&lla)
{
    xform::Xformd x_e2n;
    x_e2n.q() = q_e2n(lla0);
    x_e2n.t() = lla2ecef(lla0);
    ecef2lla(ned2ecef(x_e2n, ned), lla);
}

static Eigen::Vector3d ned2lla(const Eigen::Vector3d& lla0, const Eigen::Vector3d& ned)
{
    Eigen::Vector3d lla;
    ned2lla(lla0, ned, lla);
    return lla;
}
}
