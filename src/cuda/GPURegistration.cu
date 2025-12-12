#include "GPURegistration.cuh"


static constexpr int shared_points_size = MAXPOINTSPERVOXEL * 27;

  __global__ struct LinearSystem{
    Eigen::Matrix6dDNA jacobian;
    Eigen::Vector6dDNA twist
  }

  __global__ struct Coorespondences{
    Eigen::Vector3dDNA query;
    Eigen::Vector3dDNA source;
  }

namespace {


__device__ __constant__ Vector3dDNA voxel_shift[27] = 
      {Vector3iDNA{0, 0, 0},   Vector3iDNA{1, 0, 0},   Vector3iDNA{-1, 0, 0},  Vector3iDNA{0, 1, 0},   Vector3iDNA{0, -1, 0},
       Vector3iDNA{0, 0, 1},   Vector3iDNA{0, 0, -1},  Vector3iDNA{1, 1, 0},   Vector3iDNA{1, -1, 0},  Vector3iDNA{-1, 1, 0},
       Vector3iDNA{-1, -1, 0}, Vector3iDNA{1, 0, 1},   Vector3iDNA{1, 0, -1},  Vector3iDNA{-1, 0, 1},  Vector3iDNA{-1, 0, -1},
       Vector3iDNA{0, 1, 1},   Vector3iDNA{0, 1, -1},  Vector3iDNA{0, -1, 1},  Vector3iDNA{0, -1, -1}, Vector3iDNA{1, 1, 1},
       Vector3iDNA{1, 1, -1},  Vector3iDNA{1, -1, 1},  Vector3iDNA{1, -1, -1}, Vector3iDNA{-1, 1, 1},  Vector3iDNA{-1, 1, -1},}

__device__ __forceinline__ double square(double x) { return x * x; }

__device__ __inline__ void firstNearestNeighborQuery(VoxelMap map_, 
                                                     Eigen::Vector3dDNA* points
                                                     Coorespondences* coorespondences,
                                                     bool* actualized_coorespondences,
                                                     const double max_correspondence_distance,
                                                     int N){
    __shared__ Eigen::Vector3dDNA shared_points[MAXPOINTSPERVOXEL];
    __shared__ double shared_min_distance[MAXPOINTSPERVOXEL];
    __shared__ Eigen::Vector3dDNA shared_source_point[1];
    __shared__ Eigen::Vector3dDNA shared_best_query_point[1];
    
    unsigned int tidx = threadIdx.x;
    unsigned int btidx = blockIdx.x;
    unsigned int gtidx = blockDim.x * blockIdx.x + threadIdx.x;


    if (tidx == 0){
      shared_source_point[0] = points[btidx];
    }
    __syncthreads(); 
    double min_distance = cuda::numeric::max<double>();
    int min_voxel_index = -1;
    int voxel_shift_index = -1;

    for(int i = 0; i < 27; ++i){
      auto it = map.find(points[btidx] + voxel_shift[i]);
      if(it != map.end()){
        VoxelData* voxeldata = &it->second;
        if(tidx < voxeldata->num_points){
          shared_points[tidx] = voxeldata->points[tidx];
          shared_min_distance[tidx] = (shared_source_point[0] - shared_points[tidx]).norm();
        } 
      }

      __syncthreads();

      if (tidx == 0){
        for(int i = 0; i < MAXPOINTSPERVOXEL; ++i){
          if(shared_min_distance[i] < min_distance){
            min_distance = shared_min_distance[i];
            min_voxel_index = i;
            voxel_shift_index = i;
          }
        }
      }
      __syncthreads();
    }

    if (tidx == 0){
      coorespondences[btidx].query = shared_source_point[0];
      coorespondences[btidx].source = shared_points[min_voxel_index];
    }
    __syncthreads();
  }

/**
 *
 * @brief Does DataAssocaiation using the neareast neighbors search on the voxel hashmap setting up the linear regression
 * @param points are they points you are using to query 
 * @param voxel_map the voxel map you are querying
 * @param Registration object controls some of the parameters of building
 * @param double distance_threshold is the max regerence distance for checking if a point is going to be assocciated
 * @return A list of point coorespondences
 */
__device__ __inline__ void DataAssociation(Eigen::Vector3dDNA* points,
                GpuVoxelHashMap* voxel_map,
                Coorespondences* coorespondences,
                const double distance_threshold) {
  unsigned int tidx = threadIdx.x



}

/**
 * @brief BHulilds the linear system and 
 *
 * @param coorespondences are point coorespondences to be evaluated
 * @param kernel sclae is a value to perform outlier_rejection and weighting
 */
LinearSystem BuildLinearSystem(Correspondences &correspondences, const double kernel_scale) {
  number_of_points

  ReduceLinearSystem

  return result;
}


LinearSystem EstimateStep

} // namespace

Sophus::SE3d GPURegistration::alignPointsToMap(
    const std::vector<Eigen::Vector3d> &points,
    const GPUVoxelHashMap &voxel_map,
    const Sophus::SE3d &initial_guess
    , double max_correspondence_distance,
     double kernel_scale) {

  std::size_t number_of_points = points.size();

  Vector3dDNA* device_points;
  cudaMalloc((void**) device_points, number_of_points * sizeof(vector3dDNA));

  Coorespondences* device_coorespondences;
  cudaMalloc((void**) device_correspondences, number_of_points * sizeof(Correspondences));

  LinearSystem* linear_system_input;
  cudaMalloc((void**) linear_system_input, number_of_points * sizeof(linearSystem));

  LinearSystem* linear_system_out;
  cudaMalloc((void**) linear_system_out, sizeof(linearSystem));

  cudaMemset(linear_system_input, 0, number_of_points * sizeof(LinearSystem));
  cudaMemset(linear_system_out, 0, number_of_points * sizeof(LinearSystem));

  cudaMemcpy(device_points, points.data(), sizeof(Eigen::Vector3dDNA) * number_of_points, cudaMemcpyHostToDevice);


  unsigned int threads_per_blocks_data_association = 32;
  unsigned int blocks_linear_system = number_of_points+1;

  unsigned int threads_per_blocks_linear_system = 4096;
  unsigned int blocks_linear_system = ;

  for(int i = 0; j < num_iterations_; ++i){
    DataAssociation<<<blocks_data_association, threads_per_block_data_association>>>(
      device_points,
      device_correspondence,
      actualized_correspondences,
      max_correspondence_distance,
      number_of_points
    );

    BuildLinearSystem<<<blocks_linear_sytem, threads_per_blocks_linear_system>>>(
      correspondences,
      linear_system_input,
      kernel_scale,
      number_of_points
    );
    cudaMemcpy(linear_system_out, linear_stystem_host, cudaMemcpyDeviceToHost);

    cudaMemset(device_coorespondences, 0, number_of_points * sizeof(CorrespondencesLinearSystem)); // reset values
    cudaMemset(linear_system_input, 0, number_of_points * sizeof(LinearSystem));

  }

  // free gpu registration memory
  cudaFree(device_points);
  cudaFree(device_correspondences);
  cudaFree(linear_system_input);
  cudaFree(linear_system_out);
  return initial_guess * delta_icp;
}
