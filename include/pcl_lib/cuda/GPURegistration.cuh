#pragma once

#include "GPUVoxelHashMap.cuh"
#include "LieAlgebra.cuh"
#include "Transform.cuh"
#include <cuda/std/numeric>

#include <stdgpu/unordered_map.cuh>
#include <stdgpu/unordered_set.cuh>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// GPU Eigen mostly used for memory layouts
using Vector6dDNA = Matrix<double, 6, 1, Eigen::DontAlign>;
using Vector3dDNA = Matrix<double, 3, 1, Eigen::DontAlign>;
using Vector3iDNA = Matrix<int, 3, 1, Eigen::DontAlign>;
using Matrix6dDNA = Matrix<double, 6, 6, Eigen::DontAlign>;
using Matrix36dDNA = Matrix<double, 3, 6, Eigen::DontAlign>;
} // namespace Eigen

struct Jacobian3x6 {
  double mat[18];
  static constexpr int rows = 3;
  static constexpr int cols = 6;

  __device__ __inline__ double &operator()(int row, int col) { return mat[row * cols + col]; }

  __device__ __inline__ const double &operator()(int row, int col) const {
    return mat[row * cols + col];
  }

  __device__ __inline__ double get(int row, int col) const { return mat[row * cols + col]; }

  // Initialize with zeros
  __device__ __inline__ void zero() {
#pragma unroll
    for (int i = 0; i < 18; i++) {
      mat[i] = 0.0;
    }
  }
  __device__ __inline__ void setViaHat(double R[9]) {
    mat[3] = -R[0];
    mat[4] = -R[1];
    mat[5] = -R[2];
    mat[9] = -R[3];
    mat[10] = -R[4];
    mat[11] = -R[5];
    mat[15] = -R[6];
    mat[16] = -R[7];
    mat[17] = -R[8];
  }

  __device__ __inline__ void setIdentity() {
    mat[0] = 1;
    mat[7] = 1;
    mat[14] = 1;
  }

  __device__ __inline__ void add(Jacobian3x6 other) {
#pragma unroll
    for (int i = 0; i < 18; i++) {
      mat[i] += other.mat[i];
    }
  }
  /**
   * @brief Multiplies the jacobian by the transpose of the other jacobian
   * transpose this jacobian and multiply by it by other
   * @param other  the
   * @param result
   */
  __device__ __inline__ double[18] transpose() {

    // transpose this jacobian
    double temp_mat[18];
#pragma unroll cols
    for (int i = 0; i < cols; i++) {
#pragma unroll rows
      for (int j = 0; j < rows; j++) {
        temp_mat[i * rows + j] += mat[j * cols + i];
      }
    }
    return temp_mat;
  }

  __device__ __inline__ void multiplyScalar(double scalar) {
#pragma unroll 18
    for (int i = 0; i < 18; i++) {
      mat[i] *= scalar;
    }
  }
};

__device__ struct HessianAndVector {
  double mat[36];
  double vec[6];
  static constexpr int rows = 6;
  static constexpr int cols = 6;

  __device__ __forceinline__ double &operator()(int row, int col) { return mat[row * cols + col]; }

  __device__ __forceinline__ void vecSet(int index, double value) { vec[index] = value; }

  __device__ __forceinline__ void zero() {
#pragma unroll 36
    for (int i = 0; i < 36; i++) {
      mat[i] = 0.0;
    }
  }

  __device__ __inline__ void
  computeFromJacobianResidual(Jacobian3x6 jacobian, Eigen::Vector3dDNA residual, double weight) {
    double transposed_jacobian_mat[18] = jacobian.transpose();
    jacobian.multiplyScalar(weight);
#pragma unroll 6
    for (int i = 0; i < 6; i++) {
#pragma unroll 6
      for (int j = 0; j < 6; j++) {
#pragma unroll 3
        for (int k = 0; k < 3; k++) {
          mat[i * cols + j] += transposed_jacobian_mat[i * jacobian.rows + k] * jacobian(k, j);
        }
      }
    }
#pragma unroll
    for (int i = 0; i < 6; i++) {
      vec[i] += transposed_jacobian_mat[i * cols + j] * jacobian(i, j);
    }
  }
};

__global__ struct jacobian_residual {
  Jacobian3x6 jacobian;
  Eigen::Vector3dDNA residual;
};

__global__ struct LinearSystem {
  HessianAndVector hessian_and_vector;
};

__global__ struct Coorespondences {
  Eigen::Vector3dDNA query;
  Eigen::Vector3dDNA source;
}

class GPURegistration {
public:
  explicit GPURegistration(int max_iterations, double convergence)
      : max_num_iterations_(max_iterations), convergence_(convergence) {}

  Sophus::SE3d alignPointsToMap(const std::vector<Eigen::Vector3d> &points,
                                const GPUVoxelHashMap<PointsPerVoxel, InitialCapacity> &voxel_map,
                                const Sophus::SE3d &initial_guess,
                                double max_correspondence_distance, double kernel_scale);

private:
  int max_num_iterations_;
  double convergence_;
};
