/*! \file eigen.h
  * \author David Wheeler
  * \date June 2014
  *
  * \brief Useful functions for dealing with eigen matrices.
  *
*/

#ifndef ROS_COPTER_EIGEN_H
#define ROS_COPTER_EIGEN_H

#include <eigen3/Eigen/Core>
#include <boost/array.hpp>
#include <ros/ros.h>

namespace ros_copter
{

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

} //end namespace

#endif