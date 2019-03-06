/*
Basic functions to import matrices from the server.
Originally written by David Wheeler from BYU

*/

#ifndef BASIC_FUNTIONS_H
#define BASIC_FUNTIONS_H

#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <tf/tf.h>

namespace basic_functions
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
  void vectorToDiagMatrix(Eigen::MatrixBase<Derived>& mat, std::vector<double> vec)
  {
    ROS_ASSERT(vec.size() == mat.rows());
    mat.setZero();
    for(unsigned i=0; i < mat.rows(); i++)
    {
      mat(i,i) = vec[i];
    }
  }

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
    else if(vec.size() == mat.rows()*mat.cols())
    {
      //ROS_WARN("Reading %s/%s from server. (Full)",nh.getNamespace().c_str(),param.c_str());
      vectorToMatrix(mat,vec);
    }
    else if(vec.size() == mat.rows())
    {
      //ROS_WARN("Reading %s/%s from server. (Diagonal)",nh.getNamespace().c_str(),param.c_str());
      vectorToDiagMatrix(mat,vec);
    }
    else
    {
      ROS_ERROR("Param %s/%s is the wrong size. %f not %f or %f" ,nh.getNamespace().c_str(),param.c_str(),(double) vec.size(),(double) mat.rows(),(double) mat.rows()*mat.cols());
    }
  }

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









}



#endif
