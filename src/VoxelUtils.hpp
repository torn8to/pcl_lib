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

struct AdaptiveSamplingParams{
  int min_sampled_points = 1500;
  int max_sampled_points = 7000;
  bool point_sampling_cap = true;
  double float_last_voxel_size = 0.5;
  int max_iterations = 5
}


std::vector<Eigen::Vector3d> adativeDegeneracyAvoidantSampling(
    std::vector<Eigen::Vector3d>& cloud,
    double default_voxel_size,
    AdaptiveSamplingParams& params){
  double voxel_size = default_voxel_size;
  std::vector<Eigen::Vector3d> downsampled_cloud;
  for(int i = 0; i < max_ iteraitons; ++i){
    downsampled_cloud = voxelDownsample(cloud, voxel_size)
    if(downsampled_cloud.size() < min_sampled_points) {
      voxel_size *= 1.33;
    }
    else if(downsampled_cloud.size() > max_sampled_points){
      voxel_size *= 0.75;
    }
    else break;
  }
  return downsampled_cloud;
}



} // namespace cloud
