#pragma once

#include "GPUVoxelHashMap.cuh"
#include "Transform.cuh"
#include <cuda/std/numeric>

#include <stdgpu/unordered_map.cuh>
#include <stdgpu/unordered_set.cuh>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>


namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
} // namespace Eigen

using Vector6dDNA = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Vector3dDNA = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Vector3iDNA = Eigen::Matrix<int, 3, 1, Eigen::DontAlign>;
using Matrix6dDNA = Eigen::Matrix<double, 6, 6, Eigen::DontAlign;



template <int PointsPerVoxel, int InitialCapacity> class GPURegistration {
public:
  explicit GPURegistration(int max_iterations, double convergence)
      : max_num_iterations_(max_iterations), convergence_(convergence) {}

  Sophus::SE3d alignPointsToMap(const std::vector<Eigen::Vector3d> &points,
                                const GPUVoxelHashMap<PointsPerVoxel, InitialCapacity> &voxel_map,
                                const Sophus::SE3d &initial_guess,
                                double max_correspondence_distance,
                                double kernel_scale);

private:
  int max_num_iterations_;
  double convergence_;
};
