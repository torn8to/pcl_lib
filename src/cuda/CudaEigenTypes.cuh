#ifndef PCL_LIB_CUDA_EIGEN_TYPES_CUH
#define PCL_LIB_CUDA_EIGEN_TYPES_CUH

#include <Eigen/Core>

// Eigen::DontAlign versions for GPU (POD); shared by MotionCompensation and LieAlgebra
using Vector3dDNA = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Vector6dDNA = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;

#endif
