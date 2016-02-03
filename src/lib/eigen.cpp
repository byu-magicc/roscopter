#include "lib/eigen.h"

namespace ros_copter
{

Eigen::MatrixXd N(Eigen::MatrixXd input)
{

}


/*
template <class Derived>
void vectorToMatrix(Eigen::MatrixBase<Derived>& mat, std::vector<double> vec)
{
  ROS_ASSERT(vec.size() == mat.rows()*mat.cols());
  for(unsigned i=0; i < mat.rows(); i++)
  {
    for(unsigned j=0; j < mat.cols(); j++)
    {
      mat(i,j) = vec[mat.cols()*i+j];
    }
  }
}
*/

} // end namespace