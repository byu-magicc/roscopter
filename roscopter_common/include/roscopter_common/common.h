/*! \file eigen.h
  * \author David Wheeler
  * \date June 2014
  *
  * \brief Useful functions for dealing with eigen matrices.
  *
*/

#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <boost/array.hpp>
#include <ros/ros.h>

namespace roscopter_common
{

class Quaternion
{

public:

  Quaternion();
  ~Quaternion();
  Quaternion(double w, double x, double y, double z);
  Quaternion(double roll, double pitch, double yaw);

  double x_;
  double y_;
  double z_;
  double w_;

  Quaternion operator*(const Quaternion &q2);
  friend std::ostream& operator<<(std::ostream &os, const Quaternion &q);
  Quaternion inv();
  double mag();
  void normalize();
  double phi();
  double theta();
  double psi();
  void convertFromEigen(Eigen::Vector4d q);
  Eigen::Vector4d convertToEigen();
  Eigen::Matrix3d rot();
  Eigen::Vector3d rotateVector(Eigen::Vector3d v);

};

Quaternion exp_q(const Eigen::Vector3d delta);
Eigen::Vector3d log_q(const Quaternion q);
Eigen::Matrix3d skew(const Eigen::Vector3d vec);
Eigen::Matrix3d R_v2_to_b(double phi);
Eigen::Matrix3d R_v1_to_v2(double theta);
Eigen::Matrix3d R_v_to_v1(double psi);
Eigen::Matrix3d R_v_to_b(double phi, double theta, double psi);
Eigen::Matrix3d R_cb2c();
double saturate(double value, double max, double min);

template <class Derived>
bool vectorToMatrix(Eigen::MatrixBase<Derived>& mat, std::vector<double> vec)
{
  ROS_ASSERT(vec.size() == mat.rows()*mat.cols());
  if(vec.size() != mat.rows()*mat.cols())
    return false;
  for(unsigned i=0; i < mat.rows(); i++)
  {
    for(unsigned j=0; j < mat.cols(); j++)
    {
      mat(i,j) = vec[mat.cols()*i+j];
    }
  }
  return true;
}


template <class Derived>
bool vectorToMatrix(Eigen::MatrixBase<Derived>& mat, boost::array<double,36ul> vec)
{
  ROS_ASSERT(vec.size() == mat.rows()*mat.cols());
  if(vec.size() != mat.rows()*mat.cols())
    return false;
  for(unsigned i=0; i < mat.rows(); i++)
  {
    for(unsigned j=0; j < mat.cols(); j++)
    {
      mat(i,j) = vec[mat.cols()*i+j];
    }
  }
  return true;
}


/*!
  \brief Fills the Eigen Matrix with the contents of vector<double> row by row.

  This is implemented in the header to avoid having to specifically define the template
  types in the cpp file.
  See http://www.parashift.com/c++-faq/separate-template-fn-defn-from-decl.html
  */

template <class Derived>
void importMatrixFromParamServer(ros::NodeHandle nh, Eigen::MatrixBase<Derived>& mat, std::string param)
{
  std::vector<double> vec;
  if(!nh.getParam(param, vec))
  {
    ROS_WARN("Could not find %s/%s on server. Zeros!",nh.getNamespace().c_str(),param.c_str());
    mat.setZero();
    return;
  }
  ROS_ERROR_COND(!vectorToMatrix(mat,vec),"Param %s/%s is the wrong size" ,nh.getNamespace().c_str(),param.c_str());
}



/*
void test(boost::array<double> vec)
{

}
*/

template <class Derived, std::size_t N>
bool matrixToArray(const Eigen::MatrixBase<Derived> &mat, boost::array<double,N> &vec)
{
  ROS_ASSERT(vec.size() == mat.rows()*mat.cols());
  if(vec.size() != mat.rows()*mat.cols())
    return false;
  for(size_t i=0; i < mat.rows(); i++)
  {
    for(size_t j=0; j < mat.cols(); j++)
    {
      vec[mat.cols()*i+j] = mat(i,j);
    }
  }
  return true;
}

template <class Derived>
void verifyDimensions(const Eigen::MatrixBase<Derived> &mat, std::string name, int rows, int cols)
{
  ROS_ASSERT_MSG( mat.rows() == rows &&  mat.cols() == cols ,
                  "%s is %dx%d. Expecting %dx%d",name.c_str(),(int) mat.rows(), (int) mat.cols(),rows,cols);
}

} //end namespace roscopter_common

#endif // COMMON_H