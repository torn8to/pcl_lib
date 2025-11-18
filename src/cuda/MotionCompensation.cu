#include "MotionCompensation.cuh"


/**
 * @brief Performs motion compensation on a point cloud using timestamps and transformation based
 *
 * This function compensates for motion during the LiDAR scan by applying a transformation
 * to each point based on its timestamp relative to the scan start time. The transformation
 * is interpolated using the time difference between the current point and the first point.
 *
 * @param cloud The input point cloud to be motion compensated
 * @param cloud_timestamps Vector of timestamps corresponding to each point in the cloud
 * @param diff_transformation The differential transformation to apply (typically velocity * dt)
 * @return std::vector<Eigen::Vector3d> The motion-compensated point cloud
 * @note The cloud and timestamps vectors must have the same size
 * @note The diff_transformation should represent the motion between consecutive timestamps
 */

inline std::vector<Eigen::Vector3d> motionDeSkewThrust(const std::vector<Eigen::Vector3d> &cloud,
                                                       std::vector<double> &cloud_timestamps,
                                                       const Sophus::SE3d &relative_motion) {

  Eigen::Vector3d

  assert(cloud.size() == cloud_timestamps.size() && "Cloud and timestamps must have the same size");
  // setup device and ouput host vectors
  thrust::device_vector<Eigen::Vector3d> device_cloud(cloud.begin(), cloud.end());
  thrust::device_vector<double> device_timestamps(cloud_timestamps.begin(), cloud_timestamps.end());

  thrust::device_vector<Eigen::Vector3d> device_compensated;
  device_compensated.reserve(device_compensated.size());

  std::vector<Eigen::Vector3d> compensated_cloud;
  compensated_cloud.reserve(cloud.size());

  ZippedIterator zipBegin = thrust::make_tuple(device_cloud.begin(), device_time_stamps.begin(),
                                               device_compensated.begin());
  ZippedIterator zipBegin =
      thrust::make_tuple(device_cloud.end(), device_time_stamps.end(), device_compensated.end());

  // setup constants
  const double begin_time = cloud_timestamps.front();
  const double last_time = cloud_timestamps.back();
  const auto omega = relative_motion.log();

  thrust::for_each(thrust::device, device_compensated.begin(), device_compensated.end(),
                   [&] __device__(const ZippedIterator zipped) {
                     Eigen::Vector3d &point = thrust::get_tuple<0>(zipped);
                     const double norm_dt =
                         (thrust::get<1>(zipped)) - begin_time / (last_time - begin_time);
                     thrust::get<2>(zipped) = Sophus::SE3d::exp(omega * norm_dt) * point;
                   });

  thrust::copy(device_compensated.begin(), device_compensated.end(), compensated_cloud.begin());
  return compensated_cloud;
}

