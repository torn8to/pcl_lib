#pragma once

#include "GPUVoxelHashMap.cuh"
#include <thrust/iterator/zip_iterator.h>
#include <thrust/pair.h>
#include <thrust/tuple.h>

#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <thrust/reduce.h>

#include <vector_types.h>

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

template <int PointsPerVoxel, int InitialCapacity> class GPURegistration {
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
