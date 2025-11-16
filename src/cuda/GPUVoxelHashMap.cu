#include "GPUVoxelHashMap.cuh"
#include <cuda_runtime.h>

//cuda code in the anonymous namespace


namespace {
 /*
  *@ brief used to prevent a thread from executing on the voxel data when it is 
  */

  __device Eigen::Vector3i voxelize(){

  }

 __device__ acquireLock(int* lock){
   while(atomicCas(lock,0, 1) != 0){
    // spin this boi right round
  }
}
/*
 *@brief releas the lock on the voxel data
 */
__device__ releaseLock(int* lock){
  atomicExch(lock, 0);
}

/*
 *@brief releas the lock on the voxel data
 * 
 */

__host__ __device__ inline Eigen::Vector3i PointToVoxel(Eigen::Vector3d const &points,
                                                        double resolution) {
  return (points/resolution).cast<int>();
}


__device__  void addPointToVoxel(VoxelData *voxel_data, Eigen::Vector3d &point, const float resolution_spacing){
  acquireLock(&voxel_data->lock);
  if(!voxel_data.points_counter == 0){
    voxel_data->points[voxel_data->num_points++] = point;
  }
  bool in_free_space = true;
  for(int i = 0, voxel_data.points_counter){
    if((voxel_data->points[i] - point).squaredNorm() < resolution_spacing){
      in_free_space = false;
    }
  }
  if(in_free_space && voxel_data->points_counter < MAXPOINTSPERVOXEL){ 
    voxel_data->points[voxel_data->num_points++] = point;
  }
  releaseLock(&voxel_data->lock);
}

__device__ add_non_inserted_voxels_from_set(Eigen::Vector3d points, Eigen::Vector3i* voxels,VoxelsSet voxel_set_, voxel_mapint *valid_indicies,  int N){
  unsigned int tidx = threadIdx.x;
  unsigned int gtidx = blockIdx.x * blockDim.x + threadIdx.x;
  if(gtidx < N){
    voxels[gtidx] = pointToVoxel(points[gtidx]);
    voxel_set_.insert(voxels[gtidx])
    __syncwarps();

     
  }



}


}

template <int PointsPerVoxel, int InitialCapacity>
void GPUVoxelHashMap<PointsPerVoxel, InitialCapacity>::addPoints(const std::vector<Eigen::Vector3d> &points) {
  Eigen::Vector3d* device_points;

  cudaMalloc((void*)*device_points, sizeof(Eigen::Vector3d),);
 
  
  


  cudaDeviceSynchronize();
  /**
   * auto map_view = map_.get_device_view();
  thrust::device_vector<Eigen::Vector3d> points_device(points.begin(), points.end());
  double resolution_spacing =
      std::sqrt((resolution_ * resolution_) / static_cast<double>(PointsPerVoxel));
  thrust::for_each(
      thrust::device, points_device.begin(), points_device.end(),
      [resolution_spacing, this, map_view] __device__(const Eigen::Vector3d &point) {
        const Voxel voxel = PointToVoxel(point, this->resolution_);
        auto query = map_view.find(voxel);
        if (query == map_view.end()) {
          Value voxel_vec;
          voxel_vec.push_back(point);
          map_view.insert({voxel, std::move(voxel_vec)});
        } else {
          auto &voxel_vector = query->second();
          if (!(voxel_vector.size() == voxel_vector.max_size())) {
            if (thrust::all_of(thrust::device, voxel_vector.begin(), voxel_vector.end(),
                               [&] __device__(const auto &vector_point) {
                                 return (vector_point - point).norm() > resolution_spacing;
                               })) {
              voxel_vector.push_back(point);
            }
          }
        }
      });
      **/
}

/**
    commented out and the code is used in registration as having the
    code here would cause an issue with vectorization as such that code has been
moved to regisstration

template <int PointsPerVoxel>
__device__ std::pair<Eigen::Vector3d, double>
GPUVoxelHashMap<PointsPerVoxel>::firstNearestNeighborQuery(Eigen::Vector3d
const& query_point) { Eigen::Vector3i voxel = PointToVoxel(query_point,
resolution_); Eigen::Vector3d closest_point = Eigen::Vector3d::Zero(); double
minimum_distance = cuda::std::numeric_limits<double>::max(); for(auto&
voxel_shift: voxel_shifts){ auto it = this->map.find(voxel + voxel_shift); if
(it == this->map.end()) continue; auto& voxel_data = it->second();
      thrust::for_each(thrust::device,
                       voxel_data.begin(),
                       voxel_data.end(),
                       [&] __device__ (const Eigen::Vector3d& point){
                         double distance = (query_point -point).norm();
                         if(distance < minimum_distance){
                            closest_point = point;
                            minimum_distance = distance;
                          }
                       });
    }
    return std::pair<Eigen::Vector3d, double>(closest_point, minimum_distance);
}
*/
