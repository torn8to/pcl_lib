#include "GPUVoxelHashMap.cuh"
#include <cuda_runtime.h>

//cuda code in the anonymous namespace


namespace {
 /*
  *@ brief used to prevent a thread from executing on the voxel data when it is 
  */

 __device__ __inline__ void acquireLock(int* lock){
   while(atomicCAS(lock,0, 1) != 0){
    // spin this boi right round
  }
}
/*
 *@brief releas the lock on the voxel data
 */
__device__ __inline__ void releaseLock(int* lock){
  atomicExch(lock, 0);
}

/*
 *@brief releas the lock on the voxel data
 * 
 */

__device__  __inline__ Eigen::Vector3iDNA pointToVoxel(const Vector3dDNA point,
                                               const double voxel_resolution) {
  return Eigen::Vector3iDNA(
    __double2int_rd(point.x()/voxel_resolution),
    __double2int_rd(point.y()/voxel_resolution),
    __double2int_rd(point.z()/voxel_resolution)
  );
}


__global__ void addPointToVoxelKernel(Vector3dDNA* points,
                                const double resolution_spacing){
  unsigned int tidx = blockDim.x * blockIdx.x + threadIdx.x;
  Eigen::Vector3iDNA voxel = pointToVoxel(points[tidx], resolution_spacing); 
  acquireLock(&voxel_data[tidx]->lock);
  // if full  do nothing and releaseLock() if empty add the point at zero and post increment
  if(voxel_data[tidx]->points_counter == MAXPOINTSPERVOXEL){
    releaseLock();
    return;
  }

  if(!voxel_data->points_counter == 0){
    voxel_data[tidx]->points[voxel_data->num_points++] = point;
    releaseLock();
    return;
  }
  __syncwarps();

  bool in_free_space = true;
  for(int i = 0; i < voxel_data[tidx].points_counter; ++i){
    if((voxel_data[tidx]->points[i] - point).squaredNorm() < resolution_spacing){
      in_free_space = false;
    }
  }

  if(in_free_space){ 
    voxel_data[tidx]->points[voxel_data->num_points++] = point;
  }
  releaseLock(&voxel_data->lock);
}


__global__ void checkIfVoxelExistsAndAddVoxel(Vector3dDNA* points,
                                         stdgpu::unordered_map<Eigen::Vector3iDNA, VoxelData> map_,
                                         int N){
  //unsigned int tidx = threadIdx.x;
  unsigned int gtidx = blockIdx.x * blockDim.x + threadIdx.x;
  if(gtidx < N){
  Eigen::Vector3i voxel = pointToVoxel(&points[gtidx]);

  // seems redundant but isnt as the lock does not need to be acquired contain already 
  // existing voxels most of the time voxels are already inserted but in cases it isn't this is not recomended
  if(!map_.map.contains[voxel]){
    VoxelData* vd = &map_.map.contains(vd);
    acquireLock(&map_.lock);
    if(!map_.map.contains[voxel]){
      VoxelData vd;
      map_.insert(voxel, vd);
    }
    releaseLock(&map_.lock);
  __syncwarps();
  };
}


}

void GPUVoxelHashMap::addPoints(const std::vector<Eigen::Vector3d> &points) {
  std::size_t number_of_points = points.size();

  Vector3dDNA* device_points;
  cudaMalloc((void**) &device_points, number_of_points * sizeof(Vector3dDNA));

  Vector3iDNA* device_voxels;
  cudaMalloc((void**) &device_voxels, number_of_points * sizeof(Vector3iDNA));

  bool* unappended_voxels;
  cudaMalloc((void**) &actualized_coorespondences, number_of_points * sizeof(bool));


  cudaMemcpy(&device_points, points.data(), number_of_points*sizeof(Vector3dDNA)  cudaMemcpyHostToDevice);

  unsigned int threads_per_block = 1024;
  unsigned int blocks_ = (number_of_points + 1 + threads_per_block)/ threads_per_block;


  // first step is 
  checkIfVoxelExistsAndAddVoxel<<<blocks, threads_per_block>>>(gpu_voxel_map_,
                                  device_points,
                                  device_voxels,
                                  number_of_points);


  addPointsToVoxelKernel<<<blocks, threads_per_block>>>(
    device_points,
    device_voxels
    gpu_voxel_map_,
    device_points,
    number_of_points);
}

/**
* @brief 
*
* @param  the voxel you are looking up
* @return a vector of of the points in the voxel if the voxel does not exist it return an empty vector
*/
std::vector<Eigen::Vector3d> getVoxelPoints(const Eigen::Vector3i voxel){
std::vector<Eigen::Vector3d> voxel_points;
voxel_points.reserve(max_points_per_voxel_);

Vector3iDNA* device_voxel
cudaMalloc((void**) device_voxel, sizeof(Vector3iDNA));

Vector3dDNA* device_voxel
cudaMalloc((void**) device_voxel, sizeof(Vector3dDNA) * max_points_per_voxel_);

int host_num_points;
int* device_num_points;
cudaMalloc((void**) device_voxel, sizeof(int));

getVoxelPointKernel<<<1,1,>>>(device_voxel,
                        device_points,
                        device_num_points);


cudaMemcpy(device_points, &host_num_points, sizeof(int), cudaMemcpyDeviceToHost);
if (host_num_points == 0){
  cudaMemcpy(points, voxel_points.data(), sizeof(Eigen::Vector3d) * host_num_points, cudaMemcpyDeviceToHost);
}

cudaFree(device_voxel);
cudaFree(device_points);
cudaFree(device_num_points);

voxel_points.shrink_to_fit();
return voxel_points;
}
