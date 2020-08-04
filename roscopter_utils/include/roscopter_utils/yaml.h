#pragma once
#include <stdexcept>
#include <iostream>
#include <experimental/filesystem>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace roscopter
{

template <typename T>
bool get_yaml_node(const std::string key, const std::string filename, T& val, const std::string node_namespace = "", bool print_error = true) 
{
  // Try to load the YAML file
  YAML::Node node;
  try
  {
    node = YAML::LoadFile(filename);
    if (node_namespace != ""){
      node = node[node_namespace];
    }
  }
  catch (...)
  {
    std::cout << "Failed to Read yaml file " << filename << std::endl;
  }

  // Throw error if unable to load a parameter
  if (node[key])
  {
    val = node[key].as<T>();
    return true;
  }
  else
  {
    if (print_error)
    {
      throw std::runtime_error("Unable to load " + key + " from " + filename);
    }
    return false;
  }
}

template <typename Derived1>
bool get_yaml_eigen(const std::string key, const std::string filename, Eigen::MatrixBase<Derived1>& val,const std::string node_namespace = "", bool print_error=true)
{
    // Try to load the YAML file
  YAML::Node node;
  try
  {
    node = YAML::LoadFile(filename);
    if (node_namespace != ""){
      node = node[node_namespace];
    }
  }
  catch (...)
  {
    std::cout << "Failed to Read yaml file " << filename << std::endl;
  }

  std::vector<double> vec;
  if (node[key])
  {
    vec = node[key].as<std::vector<double>>();
    if (vec.size() == (val.rows() * val.cols()))
    {
      int k = 0;
      for (int i = 0; i < val.rows(); i++)
      {
        for (int j = 0; j < val.cols(); j++)
        {
          val(i,j) = vec[k++];
        }
      }
      return true;
    }
    else
    {
      throw std::runtime_error("Eigen Matrix Size does not match parameter size for " + key + " in " + filename +
                               ". Requested " + std::to_string(Derived1::RowsAtCompileTime) + "x" + std::to_string(Derived1::ColsAtCompileTime) +
                               ", Found " + std::to_string(vec.size()));
      return false;
    }
  }
  else if (print_error)
  {
    throw std::runtime_error("Unable to load " + key + " from " + filename);
  }
  return false;
}

template <typename Derived>
bool get_yaml_diag(const std::string key, const std::string filename, Eigen::MatrixBase<Derived>& val,const std::string node_namespace = "", bool print_error=true)
{
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> diag;
  if (get_yaml_eigen(key, filename, diag, node_namespace, print_error))
  {
    val = diag.asDiagonal();
    return true;
  }
  return false;
}

template <typename T>
bool get_yaml_priority(const std::string key, const std::string file1, const std::string file2, T& val , const std::string node_namespace = "")
{
  if (get_yaml_node(key, file1, val,node_namespace, false))
  {
    return true;
  }
  else
  {
    return get_yaml_node(key, file2, val,node_namespace, true);
  }
}

template <typename Derived1>
bool get_yaml_priority_eigen(const std::string key, const std::string file1, const std::string file2, Eigen::MatrixBase<Derived1>& val , const std::string node_namespace = "")
{
  if (get_yaml_eigen(key, file1, val,node_namespace, false))
  {
    return true;
  }
  else
  {
    return get_yaml_eigen(key, file2, val,node_namespace, true);
  }
}
}
