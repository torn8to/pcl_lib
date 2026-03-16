#include "GPUVoxelHashMap.cuh"
#include <cuda_runtime.h>

namespace {
  // spinlock code using nvidia atomics
  // returns 0 for initial lock acquire
 __device__ __inline__ int acquireLock(int* lock){
   while(atomicCAS(lock,0, 1) != 0){
    // spin this boi right round
  }
}
__device__ __inline__ void releaseLock(int* lock){
  atomicExch(lock, 0); 
}

__device__ Voxel pointToVoxel(const Vector3dDNA point,
                              const double voxel_resolution) {
  return Voxel(
    __double2int_rd(point.x()/voxel_resolution),
    __double2int_rd(point.y()/voxel_resolution),
    __double2int_rd(point.z()/voxel_resolution)
  );
}

template<typename MapType>
__global__ void addPointsToVoxelKernel(typename MapType::VoxelMap map,
                                Vector3dDNA* points,
                                const double resolution_spacing,
                                const int N){
  unsigned int gtidx = blockDim.x * blockIdx.x + threadIdx.x;
  if (gtidx >= N) {
    return;
  }

  Vector3dDNA point = points[gtidx];
  Voxel voxel = pointToVoxel(point, resolution_spacing);

  if (!map.contains(voxel)) {
    typename MapType::VoxelData vd{};
    typename MapType::VoxelMap::value_type pair{voxel, vd};
    map.insert(pair);
  }

  // Access the voxel data stored in the map
  auto it = map.find(voxel);
  if (it == map.end()) {
    return;
  }

  typename MapType::VoxelData &voxel_data = it->second;
  acquireLock(&voxel_data.lock);

  if (voxel_data.num_points == MapType::VoxelData::max_points) {
    releaseLock(&voxel_data.lock);
    return;
  }

  if (voxel_data.num_points != 0) {
    voxel_data.points[voxel_data.num_points++] = point;
    releaseLock(&voxel_data.lock);
    return;
  }

  //TODO need to collision_checking out of the global memory
  //#PRAGMA unroll
  bool in_free_space = true;
  for (int i = 0; i < voxel_data.num_points; ++i) {
    if ((voxel_data.points[i] - point).squaredNorm() < resolution_spacing) {
      in_free_space = false;
    }
  }

  if (in_free_space) {
    voxel_data.points[voxel_data.num_points++] = point;
  }
  releaseLock(&voxel_data.lock);
}



template <typename MapType>
__global__ void findVoxelKernel(typename MapType::VoxelData map,
                                     Vector3iDNA voxel,
                                     Vector3dDNA* points
                                     int* num_points){
  auto it = map.find(voxel);
  num_points* = it->second.num_points;
  points* = it->second.points;
}

} // namespace end
//
//
GPUSparseVoxelMap::GPUSparseVoxelMap(
                  double voxel_resolution ,
                  int initial_capacity , double max_range):
                  capacity_(initial_capacity),
                  max_range_(max_range),
                  voxel_resolution_(1.0){
  resolution_spacing_ = (voxel_resolution_ * voxel_resolution_) / max_points_per_voxel;
  gpu_map_ = VoxelMap::createDeviceObject(capacity_);
}


GPUSparseVoxelMap::~GPUSparseVoxelMap(){
  VoxelMap::destroyDeviceObject(gpu_map_);
}

void GPUSparseVoxelMap::addPoints(const std::vector<Eigen::Vector3d> &points) {
  std::size_t number_of_points = points.size();

  Vector3dDNA* device_points;
  cudaMalloc((void**) &device_points, number_of_points * sizeof(Vector3dDNA));
  CUDA_CHECK(cudaMemcpy(device_points,
                        points.data(),
                        number_of_points * sizeof(Vector3dDNA),
                        cudaMemcpyHostToDevice));

  unsigned int threads_per_block = 256;
  unsigned int blocks = (number_of_points + threads_per_block - 1) / threads_per_block;

  addPointsToVoxelKernel<GPUSparseVoxelMap><<<blocks, threads_per_block>>>(
    gpu_map_,
    device_points,
    voxel_resolution_,
    number_of_points);
}

std::vector<Eigen::Vector3d> GPUSparseVoxelMap::getVoxelPoints(const Eigen::Vector3i voxel){
  Vector3dDNA *gpu_points;
  int *gpu_num_points;

  Eigen::Vector3d *points_raw = (Eigen::Vector3d) malloc(max_points_per_voxel* sizeof(Eigen::Vector3d));
  int  *num_points = (int*) malloc(sizeof(int));
  CUDA_CHECK(cudaMalloc((void**) &gpu_num_points, sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**) &points,
                        max_points_per_voxel * sizeof(Vector3dDNA)));
  findVoxelKernel<GPUSparseVoxelMap><1,1>(
    points,
    num_points,
    voxel
  );
  CUDA_CHECK(cudaMemcpy(gpu_num_points,
                        num_points,
                        sizeof(int),
                        cudaMemcpyDeviceToHost));

  if(num_points[0] == 0){
    std::vector<Eigen::Vector3d> voxel_points;
    return voxel_points
  }
  cudaMemcpy(points_raw,
             gpu_points,
             num_points[0] * sizeof(Eigen::Vector3d)
             cudaMemcpyDeviceToHost);

  std::vector<Eigen::Vector3d> voxel_points(points_raw, num_points[0]);
  return voxel_points;
}
