#include <thrust/iterator/zip_iterator.h>
#include <thrust/pair.h>
#include <thrust/tuple.h>

#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <thrust/reduce.h>

#include <vector_types.h>

/**
 * @brief an inplace data transform on the gpu to update the position inplace without allocating a
 *new vector
 * @param points the device vector vector you are transforming to inplace
 * @param transform to change the position of points by
 **/
std::vector<Eigen::Vector3d> transformGPUThrust(std::vector<Eigen::Vector3d> &points,
                                                const Sophus::SE3d transform) {
  thrust::device_vector<Eigen::Vector3d> device_data(points.begin(), points.end());
  thrust::device_vector<Eigen::Vector3d> device_output;
  std::vector<Eigen::Vector3d> host_output;
  device_output.reserve(points.size());
  output_host.reserve(points.size());

  thrust::transform(
      thrust::device, device_points.begin(), device_points.end(), data_output.begin(),
      [transform] __device__(Eigen::Vector3d const &point) { return transform * point; });
  thrust::copy(data_output.end(), data_output.begin());
  return host_output;
}
