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

#include <stdgpu/unordered_map.h>
#include <stdgpu/unordered_set.h>

#include <Eigen/Dense>
#include <vector>

#define MAXPOINTSPERVOXEL 27

// TODO template this
__host__ __device__ struct VoxelData {
  int lock; // 0 unlocked and 1 locked
  int num_points;
  Eigen::Vector3d points[MAXPOINTSPERVOXEL];
};

// Use a packed voxel key that fits in 8 bytes
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

using Voxel = Eigen::Vector3i;
using VoxelSet = stdgpu::unordered_set<Voxel, Hash>;
using SparseVoxelMap = stdgpu::unordered_map<Key, Value, Hash, KeyEqual>;
using Key = Eigen::Vector3i;
using Value = VoxelData;
using KeyEqual = VoxelKey_equal;
using Hash = VoxelKey_hash;

template <int PointsPerVoxel, int InitialCapacity = 10'000'000> class GPUVoxelHashMap {
public:
  template <int PointsPerVoxel, int InitialCapacity>
  GPUVoxelHashMap(double voxel_resolution = 1.0, int initial_capacity = 100001,
                  max_points_per_voxel = 27, double max_range = 100)
      : resolution_(voxel_resolution), capacity_(initial_capacity), max_range_(max_range),
        map_(initial_capacity, cuco::empty_key<Key>{Eigen::Vector3d{0, 0, 0}},
             cuco::empty_value<Value>{Value{}}) {
    max_points_per_voxel_ =
        PointsPerVoxel < max_points_per_voxel ? PointsPerVoxel : max_points_per_voxel;
    gpu_map_ = stdgpu::unordered_map<Key, Value, Hash, KeyEqual>::createDeviceObject();
    resolution_spacing_ = (resolution_ * resolution_) /
                          max_points_per_voxel_; // setting this constant up now so it is not
                                                 // recomputed every time for gpu insertion
  }

  ~GPUVoxelHashMap() {}

  /**
   * @brief return check if the voxel map is empty returns
   * @return True if is empty false if it is not
   */
  bool empty() {
    return gpu_map_.empty(); // host function
  }

  std::vector<Eigen::Vector3d> cloud();
  void removePointsFarFromLocation(Eigen::Vector3d const &point);
  void addPoints(const std::vector<Eigen::Vector3d> &points);
  // std::pair<Eigen::Vector3d> firstNearestNeighborQuery(Eigen::Vector3d const&
  // point); moved this method to be A Kernel In GPURegistration

private:
  SparseVoxelMap<Key, Value, Hash, KeyEqual> gpu_map_;

  int max_points_per_voxel;
  double resolution_;
  double resolution_spacing;
  double max_range_;
  int capacity_;
};

// Default alias preserving previous behavior (27 points per voxel)

using GPUVoxelHashMapDefault = GPUVoxelHashMap<27>;
