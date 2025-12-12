#pragma ONCE
#define EIGEN_USE_GPU

#include <cuda/std/limits>

#include <cooperative_groups.h>
#include <cuda_runtime.h>

#include <stdgpu/unordered_map.cuh>

#include <vector>
#include <Eigen/Dense>

#define MAXPOINTSPERVOXEL 27

using Eigen::Vector6dDNA = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Eigen::Vector3dDNA = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Eigen::Vector3iDNA = Eigen::Matrix<int, 3, 1, Eigen::DontAlign>;

// Use a packed voxel key that fits in 8 bytes
// Sentinel values for empty slots
struct VoxelKey_hash {
  __global__ __device__ std::size_t operator()(Eigen::Vector3iDNA const &point) {
    return static_cast<std::size_t>(point.x() * 83492791 ^ point.y() * 6291469 ^
                                    point.z() * 12582917);
  }
};

struct VoxelKey_equal {
  
    __global__ __inline__ bool operator()(Eigen::Vector3iDNA lhs, Eigen::Vector3iDNA rhs) {
    return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
  }
};

// needs a change prolly no compile gpu
__global__ __inline__ Eigen::Vector3iDNA PointToVoxel(Eigen::Vector3dDNA* const &points, double resolution) {
  return points[0]array().floor().cast<int>();
}

using Voxel = Eigen::Vector3iDNA;
using VoxelSet = stdgpu::unordered_set<Voxel, Hash>;
using SparseVoxelMap = stdgpu::unordered_map<Key, Value, Hash, KeyEqual>;
using Key = Eigen::Vector3i;
using Value = VoxelData;
using KeyEqual = VoxelKey_equal;
using Hash = VoxelKey_hash;

__device__ struct VoxelData {
  int lock; // 0 unlocked and 1 locked
  int num_points;
  Eigen::Vector3dDNA points[MAXPOINTSPERVOXEL];
};


 class GPUVoxelHashMap {
public:
  GPUVoxelHashMap(double voxel_resolution = 1.0, int initial_capacity = 100001,
                  max_points_per_voxel = MAXPOINTSPERVOXEL, double max_range = 100)
      : resolution_(voxel_resolution), capacity_(initial_capacity), max_range_(max_range),
        map_(initial_capacity, cuco::empty_key<Key>{Eigen::Vector3d{0, 0, 0}},
             cuco::empty_value<Value>{Value{}}) {
    max_points_per_voxel_ = max_points_per_voxel; ;
    gpu_map_ = stdgpu::unordered_map<Key, Value, Hash, KeyEqual>::createDeviceObject(initial_capacity);
    resolution_spacing_ = (resolution_ * resolution_) /
                          max_points_per_voxel_; // setting this constant up now
  }

  ~GPUVoxelHashMap() {
    stdgpu::unordered_map::destroyDeviceObject(gpu_map_);
  }

  /**
   * @brief return check if the voxel map is empty returns
   * @return True if is empty false if it is not
   */
  bool empty() {
    return gpu_map_.empty(); 
  }

  std::vector<Eigen::Vector3d> cloud();
  void removePointsFarFromLocation(Eigen::Vector3d const &point);
  void addPoints(const std::vector<Eigen::Vector3d> &points);

  // methods for testing
  std::vector<Eigen::Vector3d> getVoxelPoints(Eigen::Vector3i &voxel);

  // std::pair<Eigen::Vector3d> firstNearestNeighborQuery(Eigen::Vector3d const&
  // point); moved this method to be A Kernel In GPURegistration

  SparseVoxelMap gpu_map_;

private:

  int max_points_per_voxel; 
  double resolution_;
  double resolution_spacing_;
  double max_range_;
  int capacity_;
};