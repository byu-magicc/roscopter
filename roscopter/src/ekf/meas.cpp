#include "ekf/meas.h"

namespace roscopter::ekf::meas
{

Base::Base()
{
    type = BASE;
    handled = false;
}

std::string Base::Type() const
{
    switch (type)
    {
    case BASE:
        return "Base";
        break;
    case GNSS:
        return "Gnss";
        break;
    case IMU:
        return "Imu";
        break;
    case BARO:
        return "Baro";
        break;
    case RANGE:
        return "Range";
        break;
    case MOCAP:
        return "Mocap";
        break;
    case ZERO_VEL:
        return "ZeroVel";
        break;
    }
}

bool basecmp(const Base* a, const Base* b)
{
    return a->t < b->t;
}



Imu::Imu(double _t, const Vector6d &_z, const Matrix6d &_R)
{
    t = _t;
    z = _z;
    R = _R;
    type = IMU;
}

Baro::Baro(double _t, const double &_z, const double &_R, const double& _temp)
{
    t = _t;
    z(0) = _z;
    R(0) = _R;
    temp = _temp;
    type = RANGE;
}

Range::Range(double _t, const double &_z, const double &_R)
{
    t = _t;
    z(0) = _z;
    R(0) = _R;
    type = RANGE;
}

Gnss::Gnss(double _t, const Vector6d& _z, const Matrix6d& _R) :
    p(z.data()),
    v(z.data()+3)
{
    t = _t;
    type = GNSS;
    z = _z;
    R = _R;
}

Mocap::Mocap(double _t, const xform::Xformd &_z, const Matrix6d &_R) :
    z(_z),
    R(_R)
{
    t = _t;
    type = MOCAP;
}

ZeroVel::ZeroVel(double _t)
{
    t = _t;
    type = ZERO_VEL;
}
}
