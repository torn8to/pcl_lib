#include "GPURegistration.cuh"

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
} // namespace Eigen

using Correspondences = thrust::device_vector<thrust::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = thrust::pair<Eigen::Matrix6d, Eigen::Vector6d>;

namespace {

__device__ __forceinline__ double square(double x) { return x * x; }

void transform_inplace(thrust::device_vector<Eigen::Vector3d> data, const Sophus::SE3d transform) {
  thrust::transform(
      thrust::device, data.begin(), data.end(), data.begin(),
      [transform] __device__(Eigen::Vector3d const &point) { return transform * point; });
}

template <int PointsPerVoxel, int InitialCapacity>
Correspondences
DataAssociation(thrust::device_vector<Eigen::Vector3d> &points,
                const GPUVoxelHashMap<PointsPerVoxel, InitialCapacity> &voxel_map,
                const GPURegistration<PointsPerVoxel, InitialCapacity> &registration,
                const double distance_threshold) {

  const std::array<Voxel, 27> voxel_shifts{
      {Voxel{0, 0, 0},   Voxel{1, 0, 0},   Voxel{-1, 0, 0},  Voxel{0, 1, 0},   Voxel{0, -1, 0},
       Voxel{0, 0, 1},   Voxel{0, 0, -1},  Voxel{1, 1, 0},   Voxel{1, -1, 0},  Voxel{-1, 1, 0},
       Voxel{-1, -1, 0}, Voxel{1, 0, 1},   Voxel{1, 0, -1},  Voxel{-1, 0, 1},  Voxel{-1, 0, -1},
       Voxel{0, 1, 1},   Voxel{0, 1, -1},  Voxel{0, -1, 1},  Voxel{0, -1, -1}, Voxel{1, 1, 1},
       Voxel{1, 1, -1},  Voxel{1, -1, 1},  Voxel{1, -1, -1}, Voxel{-1, 1, 1},  Voxel{-1, 1, -1},
       Voxel{-1, -1, 1}, Voxel{-1, -1, -1}}};

  thrust::device_vector<Eigen::Vector3d> sources;
  thrust::device_vector<Eigen::Vector3d> targets;

  thrust::device_vector<Eigen::Vector3d> closest_neighbor;
  thrust::device_vector<double> distances;

  sources.reserve(points.size());
  targets.reserve(points.size());
  closest_neighbor.reserve(points.size());
  distances.reserve(points.size());

  const auto map_view = voxel_map->map.get_data_view();
  double resolution = registration->resololution();

  Correspondences correspondences;
  correspondences.reserve(points.size());

  thrust::transform(thrust::device, points.begin(), points.end(),
                    thrust::make_zip_iterator(closest_neighbor.begin(), distances.begin()),
                    [=] __device__(const Eigen::Vector3d &point) {
                      Eigen::Vector3i voxel = PointToVoxel(point, resolution);
                      Eigen::Vector3d closest_point = Eigen::Vector3d::Zero();
                      double minimum_distance = cuda::std::numeric_limits<double>::max();
                      for (int i = 0; i < 27; i++) {
                        auto it = map_view.find(voxel + voxel_shifts[i]);
                        if (it == map_view.end())
                          continue;
                        auto &voxel_data = it->second();
                        for (std::size_t j = 0; j < voxel_data.size(); j++) {
                          double distance = (point - voxel_data[j]).norm();
                          if (distance < minimum_distance) {
                            closest_point = point;
                            minimum_distance = distance;
                          }
                        }
                      }
                      return thrust::make_pair(closest_point, minimum_distance);
                    });

  thrust::copy_if(thrust::device,
                  thrust::make_zip_iterator(thrust::make_tuple(
                      points.begin(), closest_neighbor.begin(), distances.begin())),
                  thrust::make_zip_iterator(
                      thrust::make_tuple(points.end(), closest_neighbor.end(), distances.end())),
                  thrust::make_zip_iterator(thrust::make_tuple(sources.begin(), targets.begin())),
                  [=] __device__(const thrust::tuple<Eigen::Vector3d, Eigen::Vector3d, double> t) {
                    return thrust::get<2>(t) < distance_threshold;
                  });

  thrust::copy(
      thrust::make_zip_iterator(thrust::make_tuple(sources.begin(), targets.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(sources.end(), targets.end())),
      correspondences.begin(),
      [=] __device__(const thrust::tuple<Eigen::Vector3d, Eigen::Vector3d> correspondence) {
        return thrust::make_pair(thrust::get<0>(correspondence), thrust::get<1>(correspondence));
      });

  return correspondences;
}

LinearSystem BuildLinearSystem(Correspondences &correspondences, const double kernel_scale) {
  auto compute_jacobian_and_residual = [] __device__(const auto &correspondence) {
    const Eigen::Vector3d &source = correspondence.first;
    const Eigen::Vector3d &target = correspondence.second;
    const Eigen::Vector3d residual = source - target;
    Eigen::Matrix36d J_r;
    J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);
    return thrust::make_tuple(J_r, residual);
  };

  auto GM_weight = [=] __device__(const double &residual2) {
    return square(kernel_scale) / square(kernel_scale + residual2);
  };

  thrust::device_vector<LinearSystem> linear_systems;
  linear_systems.reserve(correspondences.size());
  thrust::transform(
      thrust::device, correspondences.begin(), correspondences.end(), linear_systems.begin(),
      [=] __device__(const auto &correspondences) -> LinearSystem {
        const thrust::tuple<Eigen::Matrix36d, Eigen::Vector3d> jr_result =
            compute_jacobian_and_residual(correspondences);
        const Eigen::Matrix36d &j_r = thrust::get<0>(jr_result);
        const Eigen::Vector3d &residual = thrust::get<1>(jr_result);
        const double w = GM_weight(residual.squaredNorm());
        return LinearSystem(j_r.transpose() * w * j_r, j_r.transpose() * w * residual);
      });

  LinearSystem result =
      thrust::reduce(linear_systems.begin(), linear_systems.end(), LinearSystem(),
                     [] __host__ __device__(const LinearSystem &a, const LinearSystem &b) {
                       LinearSystem result;
                       result.first = a.first + b.first;
                       result.second = a.second + b.second;
                       return result;
                     });

  return result;
}

} // namespace

template <int PointsPerVoxel, int InitialCapacity>
Sophus::SE3d GPURegistration<PointsPerVoxel, InitialCapacity>::alignPointsToMap(
    const std::vector<Eigen::Vector3d> &points,
    const GPUVoxelHashMap<PointsPerVoxel, InitialCapacity> &voxel_map,
    const Sophus::SE3d &initial_guess, double max_correspondence_distance, double kernel_scale) {

  // allocate points device
  // transform points on device
  // on gpu associate data
  // on gpu Build and solve linear system
  // return to host with result and evaluate convergence
  thrust::device_vector<Eigen::Vector3d> device_points(points.begin(), points.end());
  transform_inplace(device_points, initial_guess);
  Sophus::SE3d delta_icp = Sophus::SE3d();
  LinearSystem result;

  for (int i = 0; i < max_num_iterations_; i++) {
    Correspondences correspondences = DataAssociation<PointsPerVoxel, InitialCapacity>(
        device_points, voxel_map, this, max_correspondence_distance);
    LinearSystem result = BuildLinearSystem(correspondences, kernel_scale);
    const Eigen::Matrix6d &JTJ = result.first;
    const Eigen::Vector6d &JTr = result.second;
    const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
    const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
    transform_inplace(device_points, estimation);
    delta_icp = delta_icp * estimation;
    if (dx.norm() < convergence_)
      break;
  }
  // free non thrust allocated memory
  return initial_guess * delta_icp;
}
