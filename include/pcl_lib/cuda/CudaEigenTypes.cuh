#pragma once
#define PCL_LIB_CUDA_EIGEN_TYPES_CUH

// overrides to use eigen on the gpu
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#define EIGEN_USE_GPU

#include <cuda_runtime.h>
#include <Eigen/Core>


// Aliasing Don't align is reccomended as cuda vectorization behaves differently to cpu vectorization
using Vector3iDNA = Eigen::Matrix<int, 3, 1, Eigen::DontAlign>;
using Vector3dDNA = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Vector6dDNA = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Matrix6dDNA = Eigen::Matrix<double, 6, 6, Eigen::DontAlign>;
using Matrix36dDNA = Eigen::Matrix<double, 3, 6, Eigen::DontAlign>;
using Matrix63dDNA = Eigen::Matrix<double, 6, 3, Eigen::DontAlign>;



struct Coorespondences {
  Vector3dDNA query;
  Vector3dDNA source;
  bool actualized;''
};


struct LinearSystem{
  Matrix6dDNA A;
  Vector6dDNA B;
}


using LinearSystem = HessianAndVector;
