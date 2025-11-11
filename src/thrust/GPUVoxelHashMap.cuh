#pragma ONCE
#define EIGEN_USE_GPU

#include <cuco/dynamic_map.cuh>
#include <cuco/hash_functions.cuh>

#include <cuda/std/inplace_vector>
#include <cuda/std/limits>

#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <thrust/functional.h>
#include <thrust/logical.h>

#include <cooperative_groups.h>
#include <cuda_runtime.h>

#include <Eigen/Dense>
#include <vector>

// Use a packed voxel key that fits in 8 bytes

using Voxel = Eigen::Vector3i;

// Sentinel values for empty slots
struct VoxelKey_hash {
  __host__ __device__ std::size_t operator()(Eigen::Vector3i const &point) {
    return static_cast<std::size_t>(point.x() * 83492791 ^ point.y() * 6291469 ^
                                    point.z() * 12582917);
  }
};

struct VoxelKey_equal {
  __host__ __device__ bool operator()(Eigen::Vector3i const &lhs, Eigen::Vector3i const &rhs) {
    return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
  }
};

__host__ __device__ inline Eigen::Vector3i PointToVoxel(Eigen::Vector3d const &points,
                                                        double resolution) {
  return points.array().floor().cast<int32_t>();
}

template <int PointsPerVoxel, int InitialCapacity = 100'000> class GPUVoxelHashMap {
public:
  GPUVoxelHashMap(double voxel_resolution = 1.0, int initial_capacity = 100001,
                  double max_range = 100)
      : resolution_(voxel_resolution), capacity_(initial_capacity), max_range_(max_range),
        map_(initial_capacity, cuco::empty_key<Key>{Eigen::Vector3d{0, 0, 0}},
             cuco::empty_value<Value>{Value{}}) {}

  ~GPUVoxelHashMap() {}

  void empty();
  std::vector<Eigen::Vector3d> cloud();
  void addPoints(const std::vector<Eigen::Vector3d> &points);
  // std::pair<Eigen::Vector3d> firstNearestNeighborQuery(Eigen::Vector3d const&
  // point); moved this method to be A Kernel In GPURegistration
  void removePointsFarFromLocation(Eigen::Vector3d const &point);

private:
  using Key = Eigen::Vector3d;
  using Value = cuda::std::inplace_vector<Eigen::Vector3d, PointsPerVoxel>;
  // static constexpr Key empty_key_sentinel = VoxelKey{0, 0, 0};
  // static constexpr Value empty_value_sentinel = Value{};
  using probing_scheme_type = cuco::linear_probing<1, VoxelKey_hash>;
  using KeyEqual = VoxelKey_equal;

  cuco::dynamic_map<Key, Value, cuda::thread_scope_device> map_;

  double resolution_;
  double max_range_;
  int capacity_;
};

// Default alias preserving previous behavior (27 points per voxel)

using GPUVoxelHashMapDefault = GPUVoxelHashMap<27>;
