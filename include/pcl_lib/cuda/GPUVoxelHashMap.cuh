#pragma ONCE
#include "CudaEigenTypes.cuh"
#include "utils.cuh"
#include <cuda/std/limits>
#include <cuda_runtime.h>
#include <stdgpu/unordered_map.cuh>
#include <algorithm>


template <int max_points_per_voxel = 27>
struct VoxelData {
  int lock; // 0 unlocked and 1 locked
  int num_points;
  Vector3dDNA points[max_points_per_voxel];
  static constexpr int max_points = max_points_per_voxel;
};

using Voxel = Vector3iDNA;
using DefaultVoxelData = VoxelData<27>;

// Use a packed voxel key that fits in 8 bytes
// Sentinel values for empty slots
struct VoxelKey_hash {
  __host__ __device__ std::size_t operator()(const Voxel &point) const {
    std::size_t h = 0;
    h^= point.x() * 83492791;
    h^= point.y() * 6291469;
    h^= point.z() * 12582917;
    return h;
  }
};

struct VoxelKey_equal {
  __host__ __device__ bool operator()(Voxel const &lhs, Voxel const &rhs) const {
    return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
  };
};

struct VoxelFlag{
  Vector3iDNA key;
  bool flag;
};

template <typename voxel_data = DefaultVoxelData>
class GPUSparseVoxelMap {
public:
  using Key = Voxel;
  using KeyEqual = VoxelKey_equal;
  using Hash = VoxelKey_hash;
  using Value = voxel_data; //templated value assigned
  using VoxelData = Value;
  using VoxelMap = stdgpu::unordered_map<Key, Value, Hash, KeyEqual>;
  using ConstIterator = typename VoxelMap::const_iterator;

  GPUSparseVoxelMap(double voxel_resolution = 1.0,
                  int initial_capacity = 100001,
                  double max_range = 100);
  ~GPUSparseVoxelMap();
  bool empty();
  void clear();
  std::vector<Eigen::Vector3d> cloud();
  void removePointsFarFromLocation(Eigen::Vector3d const &point);
  void addPoints(const std::vector<Eigen::Vector3d> &points);
  std::vector<Eigen::Vector3d> getVoxelPoints(const Eigen::Vector3i &voxel);

  VoxelMap gpu_map_;
private:
  static constexpr int max_points_per_voxel = VoxelData::max_points;
  VoxelFlag* flag_;
  double voxel_resolution_;
  double resolution_spacing_;
  double max_range_;
  const int capacity_;
};

// Default alias preserving previous behavior (27 points per voxel)

using GPUSparseVoxelMapDefault = GPUSparseVoxelMap<DefaultVoxelData>;