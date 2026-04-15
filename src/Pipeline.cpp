#include <pcl_lib/Pipeline.hpp>

namespace cloud {

Pipeline::Pipeline(const PipelineConfig &config)
    : registration_(config.num_iterations, config.convergence, config.num_threads),
      current_position_(Sophus::SE3d()), voxel_factor_(config.voxel_factor),
      max_distance_(config.max_distance), voxel_resolution_alpha_(config.voxel_resolution_alpha),
      voxel_resolution_beta_(config.voxel_resolution_beta),
      imu_integration_enabled_(config.imu_integration_enabled),
      max_points_per_voxel_(config.max_points_per_voxel),
      voxel_map_((config.max_distance / config.voxel_factor) * config.voxel_resolution_alpha,
                 config.max_distance, config.max_points_per_voxel),
      threshold(config.initial_threshold, config.min_motion_threshold, config.max_distance),
      adaptive_sampling_params_(config.params) {}

Pipeline::~Pipeline() {
  // Cleanup if needed
}

std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>>
Pipeline::odometryUpdate(std::vector<Eigen::Vector3d> &cloud,
                         const std::optional<Sophus::SE3d> &external_input) {
  std::vector<Eigen::Vector3d> cloud_voxel_odom;
  cloud_voxel_odom =
      adaptiveDegeneracyAwareSampling(cloud, voxel_resolution_beta_, adaptive_sampling_params_);
  std::vector<Eigen::Vector3d> cloud_voxel_mapping = voxelDownsample(
      cloud, (voxel_resolution_alpha_ *
              (adaptive_sampling_params_.last_voxel_size /
               voxel_resolution_beta_))); // scaling map update by adaptive_voxel_size
  Sophus::SE3d initial_guess;
  if (external_input.has_value() &&
      !voxel_map_.empty()) { // avoid prefilling map from non zero position
    initial_guess = external_input.value();
  } else {
    initial_guess = current_position_ * pose_diff_;
  }
  const double sigma = threshold.computeThreshold(); // adpative thresholding for kiss icp
  Sophus::SE3d new_position = registration_.alignPointsToMap(cloud_voxel_odom, voxel_map_,
                                                             initial_guess, 3.0 * sigma, sigma);

  const auto model_error = new_position.inverse() * initial_guess;
  threshold.updateModelDeviation(model_error);

  std::vector<Eigen::Vector3d> cloud_voxel_mapping_transformed =
      voxel_map_.transform_cloud(cloud_voxel_mapping, new_position);

  std::vector<Eigen::Vector3d> cloud_voxel_odom_transformed =
      voxel_map_.transform_cloud(cloud_voxel_odom, new_position);

  pose_diff_ = position().inverse() * new_position;

  voxel_map_.addPoints(cloud_voxel_mapping_transformed);
  voxel_map_.removePointsFarFromOrigin(new_position.translation());

  updatePosition(new_position);
  return std::make_tuple(new_position, cloud_voxel_mapping);
}

/**
 * @brief Adds a set of points to the voxel map.
 *
 * This method inserts the provided points into the internal voxel map structure,
 * updating the map with new data. The points are typically assumed to be in the
 * global or map frame and will be voxelized and managed according to the map's
 * configuration. assumes points have been already trnasformed.
 *
 * @param points The vector of 3D points to add to the map.
 */
void Pipeline::addToMap(const std::vector<Eigen::Vector3d> &points) {
  voxel_map_.addPoints(points);
}

Sophus::SE3d Pipeline::position() const { return current_position_; }

void Pipeline::updatePosition(const Sophus::SE3d transformation_matrix) {
  current_position_ = transformation_matrix;
}

} // namespace cloud
