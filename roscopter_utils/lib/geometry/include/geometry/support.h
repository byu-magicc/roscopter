#pragma once

#include <random>

#include <Eigen/Core>

typedef Eigen::Matrix<double, 1, 1> Vector1d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;

typedef Eigen::Matrix<double, 1, 1> Matrix1d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;


static const Eigen::Matrix<double, 2, 3> I_2x3 = [] {
  Eigen::Matrix<double, 2, 3> tmp;
  tmp << 1.0, 0, 0,
         0, 1.0, 0;
  return tmp;
}();

static const Eigen::Matrix3d I_3x3 = [] {
  Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity();
  return tmp;
}();

static const Eigen::Matrix2d I_2x2 = [] {
  Eigen::Matrix2d tmp = Eigen::Matrix2d::Identity();
  return tmp;
}();


static const Eigen::Vector3d e_x = [] {
  Eigen::Vector3d tmp;
  tmp << 1.0, 0, 0;
  return tmp;
}();

static const Eigen::Vector3d e_y = [] {
  Eigen::Vector3d tmp;
  tmp << 0, 1.0, 0;
  return tmp;
}();

static const Eigen::Vector3d e_z = [] {
  Eigen::Vector3d tmp;
  tmp << 0, 0, 1.0;
  return tmp;
}();

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived>& v)
{
  Eigen::Matrix<typename Derived::Scalar, 3, 3> mat;
  typename Derived::Scalar zr(0.0);
  mat << zr, -v(2), v(1),
         v(2), zr, -v(0),
         -v(1), v(0), zr;
  return mat;
}

template <typename Derived>
void setNormalRandom(Eigen::MatrixBase<Derived>& M, std::normal_distribution<double>& N, std::default_random_engine& g)
{
  for (int i = 0; i < M.rows(); i++)
  {
    for (int j = 0; j < M.cols(); j++)
    {
      M(i,j) = N(g);
    }
  }
}

template <typename Derived>
Derived randomNormal(std::normal_distribution<typename Derived::Scalar>& N, std::default_random_engine& g)
{
  Derived out;
  for (int i = 0; i < Derived::RowsAtCompileTime; i++)
  {
    for (int j = 0; j < Derived::ColsAtCompileTime; j++)
    {
      out(i,j) = N(g);
    }
  }
  return out;
}

template <typename T, int R, int C>
Eigen::Matrix<T, R, C> randomUniform(std::uniform_real_distribution<T>& N, std::default_random_engine& g)
{
  Eigen::Matrix<T,R,C> out;
  for (int i = 0; i < R; i++)
  {
    for (int j = 0; j < C; j++)
    {
      out(i,j) = N(g);
    }
  }
  return out;
}

template <typename T>
int sign(T in)
{
  return (in >= 0) - (in < 0);
}

template <typename T>
inline T random(T max, T min)
{
  T f = (T)rand() / RAND_MAX;
  return min + f * (max - min);
}
