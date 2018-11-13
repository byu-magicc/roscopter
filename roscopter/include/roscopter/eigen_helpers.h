#pragma once

#include <ros/ros.h>
#include <Eigen/Core>


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
void importMatrixFromParamServer(ros::NodeHandle nh, Eigen::MatrixBase<Derived>& mat, std::string param)
{
  std::vector<double> vec;
  ROS_FATAL_COND(!nh.getParam(param, vec), "Could not find %s/%s on server.",nh.getNamespace().c_str(),param.c_str());
  ROS_FATAL_COND(!vectorToMatrix(mat,vec), "Param %s/%s is the wrong size - param is %d (total) while matrix is %dx%d",
                  nh.getNamespace().c_str(),param.c_str(), (int)vec.size(), (int)mat.rows(), (int)mat.cols());
}
