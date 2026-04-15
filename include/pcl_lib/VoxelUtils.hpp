#pragma once
#include "PointToVoxel.hpp"
#include "VoxelMap.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <unordered_map>
#include <vector>

namespace cloud {
/**
 * @brief Voxel downsample the point to the voxel resolution
 * @param cloud The point cloud to downsample
 * @param voxel_size The size of the voxel
 * @param max_points_per_voxel The maximum number of points per voxel
 * @return The downsampled point cloud
 */
std::vector<Eigen::Vector3d> voxelDownsample(std::vector<Eigen::Vector3d> &cloud,
                                             double voxel_size) {
  std::unordered_map<cloud::Voxel, Eigen::Vector3d> voxel_filter;
  voxel_filter.reserve(43103);
  std::for_each(cloud.begin(), cloud.end(), [&](const auto point) {
    voxel_filter.insert({cloud::PointToVoxel(point, voxel_size), point});
  });
  std::vector<Eigen::Vector3d> pruned_cloud;
  pruned_cloud.reserve(voxel_filter.size());
  for (const auto &kv : voxel_filter) {
    pruned_cloud.push_back(kv.second);
  }
  return pruned_cloud;
}

struct AdaptiveSamplingParams {
  int min_sampled_points = 2000;
  int max_sampled_points = 7000;
  bool point_sampling_cap = true;
  double last_voxel_size = 0.5;
  int max_iterations = 3;
};

/**
 * @brief a sampling strategy that uses number of sampled for degenerative environments such as
 * sampling in environments with different size constraints, with a point sampling cap to reduce the
 * ovehead icp can have when sampling
 * @param the cloud to sample from
 * @param default_voxel_size the voxel size we are assuming we are priming for the voxels
 * @param params the adaptive sampling params that show the params
 * @returns downsmpled cloud
 */
std::vector<Eigen::Vector3d> adaptiveDegeneracyAwareSampling(std::vector<Eigen::Vector3d> &cloud,
                                                             const double default_voxel_size,
                                                             AdaptiveSamplingParams &params) {
  double voxel_size = default_voxel_size;
  std::vector<Eigen::Vector3d> downsampled_cloud;
  for (int i = 0; i < params.max_iterations; ++i) {
    downsampled_cloud = voxelDownsample(cloud, voxel_size);
    if (downsampled_cloud.size() < params.min_sampled_points) {
      voxel_size *= 1.33;
    } else if (downsampled_cloud.size() > params.max_sampled_points) {
      voxel_size *= 0.75;
    } else
      break;
  }
  params.last_voxel_size = voxel_size;
  return downsampled_cloud;
}

} // namespace cloud
