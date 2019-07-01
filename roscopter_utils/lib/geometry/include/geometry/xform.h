#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <iostream>

#include "geometry/support.h"
#include "geometry/quat.h"

namespace xform
{

template <typename T>
class Xform
{
private:

  typedef Eigen::Matrix<T, 2, 1> Vec2;
  typedef Eigen::Matrix<T, 3, 1> Vec3;
  typedef Eigen::Matrix<T, 4, 1> Vec4;
  typedef Eigen::Matrix<T, 5, 1> Vec5;
  typedef Eigen::Matrix<T, 6, 1> Vec6;
  typedef Eigen::Matrix<T, 7, 1> Vec7;

  typedef Eigen::Matrix<T, 3, 3> Mat3;
  typedef Eigen::Matrix<T, 4, 4> Mat4;
  typedef Eigen::Matrix<T, 6, 6> Mat6;
  T buf_[7];

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Map<Vec7> arr_;
  Eigen::Map<Vec3> t_;
  quat::Quat<T> q_;

  Xform() :
    arr_(buf_),
    t_(arr_.data()),
    q_(arr_.data() + 3)
  {
    arr_.setZero();
    arr_(3) = (T)1.0;
  }

  Xform(const Eigen::Ref<const Vec7> arr) :
    arr_(const_cast<T*>(arr.data())),
    t_(arr_.data()),
    q_(arr_.data() + 3)
  {}

  Xform(const Xform& X) :
    arr_(buf_),
    t_(arr_.data()),
    q_(arr_.data() + 3)
  {
    arr_ = X.arr_;
  }

  Xform(const T* data) :
    arr_(const_cast<T*>(data)),
    t_(arr_.data()),
    q_(arr_.data() + 3)
  {}

  Xform(const Vec3& t, const quat::Quat<T>& q) :
    arr_(buf_),
    t_(arr_.data()),
    q_(arr_.data() + 3)
  {
    // copies arguments into contiguous (owned) memory
    t_ = t;
    q_ = q;
  }

  Xform(const Vec3& t, const Mat3& R) :
    arr_(buf_),
    t_(arr_.data()),
    q_(arr_.data() + 3)
  {
    q_ = quat::Quat<T>::from_R(R);
    t_ = t;
  }

  T* data()
  {
      return arr_.data();
  }

  const T* data() const
  {
      return arr_.data();
  }

//  Xform(const Mat4& X) :
//    arr_(buf_),
//    t_(arr_.data()),
//    q_(arr_.data() + 3)
//  {
//    q_ = Quat<T>::from_R(X.block<3,3>(0,0));
//    t_ = X.block<3,1>(0, 3);
//  }

  inline Eigen::Map<Vec3>& t() { return t_;}
  inline const Eigen::Map<Vec3>& t() const { return t_;}
  inline quat::Quat<T>& q() { return q_;}
  inline const quat::Quat<T>& q() const { return q_;}
  inline Eigen::Map<Vec7>& arr() { return arr_; }
  inline const Eigen::Map<Vec7>& arr() const { return arr_; }
  inline void setq(const quat::Quat<T>& q) {q_ = q;}
  inline void sett(const Vec3&t) {t_ = t;}

  Xform operator* (const Xform& X) const
  {
      return otimes(X);
  }

  Xform& operator*= (const Xform& X)
  {
    t_ = t_ + q_.rotp(X.t_);
    q_ = q_ * X.q_;
    return *this;
  }

  Xform& operator=(const Xform& X)
  {
      t_ = X.t_;
      q_ = X.q_;
      return *this;
  }

  Xform& operator=(const Vec7& v)
  {
    t_ = v.template segment<3>(0);
    q_ = quat::Quat<T>(v.template segment<4>(3));
    return *this;
  }

  Xform operator+ (const Vec6& v) const
  {
    return boxplus(v);
  }

  template<typename T2>
  Eigen::Matrix<T2,6,1> operator- (const Xform<T2>& X) const
  {
    return boxminus(X);
  }

  Xform& operator+=(const Vec6& v)
  {
    arr_ = boxplus(v).elements();
    return *this;
  }

  template<typename T2>
  Xform<T2> cast() const
  {
    Xform<T2> x;
    x.arr_ = arr_.template cast<T2>();
    return x;
  }

  Vec7 elements() const
  {
    Vec7 out;
    out.template block<3,1>(0,0) = t_;
    out.template block<4,1>(3,0) = q_.arr_;
    return out;
  }

  Mat4 Mat() const
  {
    Mat4 out;
    out.block<3,3>(0,0) = q_.R();
    out.block<3,1>(0,3) = t_;
    out.block<1,3>(3,0) = Eigen::Matrix<T,1,3>::Zero();
    out(3,3) = 1.0;
  }

  static Xform Identity()
  {
    Xform out;
    out.t_.setZero();
    out.q_ = quat::Quat<T>::Identity();
    return out;
  }

  static Xform Random()
  {
    Xform out;
    out.t_.setRandom();
    out.q_ = quat::Quat<T>::Random();
    return out;
  }

  static Xform exp(const Eigen::Ref<const Vec6>& v)
  {
    Vec3 u = v.template block<3,1>(0,0);
    Vec3 omega = v.template block<3,1>(3,0);
    T th = omega.norm();
    quat::Quat<T> q_exp = quat::Quat<T>::exp(omega);
    if (th > 1e-4)
    {
      Mat3 wx = quat::Quat<T>::skew(omega);
      T B = ((T)1. - cos(th)) / (th * th);
      T C = (th - sin(th)) / (th * th * th);
      return Xform((I_3x3 + B*wx + C*wx*wx).transpose() * u, q_exp);
    }
    else
    {
      return Xform(u, q_exp);
    }
  }

  static Vec6 log(const Xform& X)
  {
    Vec6 u;
    Vec3 omega = quat::Quat<T>::log(X.q_);
    u.template block<3,1>(3,0) = omega;
    T th = omega.norm();
    if (th > 1e-8)
    {
      Mat3 wx = quat::Quat<T>::skew(omega);
      T A = sin(th)/th;
      T B = ((T)1. - cos(th)) / (th * th);
      Mat3 V = I_3x3 - (1./2.)*wx + (1./(th*th)) * (1.-(A/(2.*B)))*(wx* wx);
      u.template block<3,1>(0,0) = V.transpose() * X.t_;
    }
    else
    {
      u.template block<3,1>(0,0) = X.t_;
    }
    return u;
  }

  Mat6 Adj() const
  {
    Mat6 out;
    Mat3 R = q_.R();
    out.template block<3,3>(0,0) = R;
    out.template block<3,3>(0,3) = quat::Quat<T>::skew(t_)*R;
    out.template block<3,3>(3,3) = R;
    out.template block<3,3>(3,0) = Mat3::Zero();
    return out;
  }

  Xform inverse() const{
    Xform out(-q_.rotp(t_), q_.inverse());
    return out;
  }

  template <typename Tout=T, typename T2>
  Xform<Tout> otimes(const Xform<T2>& X2) const
  {
    Xform<Tout> X;
    Eigen::Matrix<Tout,3,1> t = (Tout)2.0*X2.t_.cross(q_.bar());
    X.t_ = t_+ X2.t_ - q_.w()* t + t.cross(q_.bar());
    X.q_ = q_.template otimes<Tout,T2>(X2.q_);
    return X;
  }

  template<typename Tout=T, typename Derived>
  Eigen::Matrix<Tout, 3, 1> transforma(const Derived& v) const
  {
    static_assert(Derived::RowsAtCompileTime == Eigen::Dynamic
                  || Derived::RowsAtCompileTime == 3,
                  "Can only transform 3x1 vectors");
    static_assert(Derived::ColsAtCompileTime == Eigen::Dynamic
                  || Derived::ColsAtCompileTime == 1,
                  "Can only transform 3x1 vectors");

    return q_.template rota<Tout>(v) + t_;
  }

  template<typename Tout=T, typename Derived>
  Eigen::Matrix<Tout, 3, 1> transformp(const Derived& v) const
  {
    static_assert(Derived::RowsAtCompileTime == 3,
                  "Can only transform 3x1 vectors");
    static_assert(Derived::ColsAtCompileTime == 1,
                  "Can only transform 3x1 vectors");

    return q_.template rotp<Tout>(v - t_);
  }

  template<typename Tout=T, typename Derived>
  Eigen::Matrix<Tout, 3, 1> rota(const Derived& v) const
  {
    return q_.template rota<Tout>(v);
  }

  template<typename Tout=T, typename Derived>
  Eigen::Matrix<Tout, 3, 1> rotp(const Derived& v) const
  {
      return q_.template rotp<Tout>(v);
  }

  Xform& invert()
  {
    t_ = -q_.rotp(t_);
    q_.invert();
    return *this;
  }

  template <typename Tout=T, typename T2>
  Xform<Tout> boxplus(const Eigen::Matrix<T2, 6, 1>& delta) const
  {
    return otimes<Tout, T2>(Xform<T2>::exp(delta));
  }

  template<typename T2>
  Eigen::Matrix<T2,6,1> boxminus(const Xform<T2>& X) const
  {
    return Xform<T2>::log(X.inverse().otimes(*this));
  }

};

template <typename T>
inline std::ostream& operator<< (std::ostream& os, const Xform<T>& X)
{
  os << "t: [ " << X.t_(0,0) << ", " << X.t_(1,0) << ", " << X.t_(2,0) <<
        "] q: [ " << X.q_.w() << ", " << X.q_.x() << "i, " << X.q_.y() << "j, " << X.q_.z() << "k]";
  return os;
}

typedef Xform<double> Xformd;

}
