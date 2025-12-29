#include "GPURegistration.cuh"

static constexpr Jacobian3x6 ZeroJacobian = Jacobian3x6().zero();

namespace {

__device__ __constant__ Eigen::Vector3iDNA voxel_shift[27] = {
    Eigen::Vector3iDNA(0, 0, 0),   Eigen::Vector3iDNA(1, 0, 0),   Eigen::Vector3iDNA(-1, 0, 0),
    Eigen::Vector3iDNA(0, 1, 0),   Eigen::Vector3iDNA(0, -1, 0),  Eigen::Vector3iDNA(0, 0, 1),
    Eigen::Vector3iDNA(0, 0, -1),  Eigen::Vector3iDNA(1, 1, 0),   Eigen::Vector3iDNA(1, -1, 0),
    Eigen::Vector3iDNA(-1, 1, 0),  Eigen::Vector3iDNA(-1, -1, 0), Eigen::Vector3iDNA(1, 0, 1),
    Eigen::Vector3iDNA(1, 0, -1),  Eigen::Vector3iDNA(-1, 0, 1),  Eigen::Vector3iDNA(-1, 0, -1),
    Eigen::Vector3iDNA(0, 1, 1),   Eigen::Vector3iDNA(0, 1, -1),  Eigen::Vector3iDNA(0, -1, 1),
    Eigen::Vector3iDNA(0, -1, -1), Eigen::Vector3iDNA(1, 1, 1),   Eigen::Vector3iDNA(1, 1, -1),
    Eigen::Vector3iDNA(1, -1, 1),  Eigen::Vector3iDNA(1, -1, -1), Eigen::Vector3iDNA(-1, 1, 1),
    Eigen::Vector3iDNA(-1, 1, -1)};

__device__ __forceinline__ double square(double x) { return x * x; }

__device__ __inline__ cuda::std::tuple<Coorespondence, bool>
firstNearestNeighborQuery(VoxelMap map_, 
                          Eigen::Vector3dDNA point,
                          const double max_correspondence_distance) {
  Coorespondence coorespondence;
  coorespondence.source = point;
  bool is_actualized = false;
  double min_distance = cuda::numeric::max<double>();

  #pragma unroll 27
  for (int i = 0; i < 27; ++i) {
    auto it = map.find(point + voxel_shift[i]);
    if (it != map.end()) {
      int num_points = it->second->num_points;
      Eigen::Vector3dDNA voxel_points[MAXPOINTSPERVOXEL] = it->second->points;
      for(int j = 0; j < num_points; ++j){
        double distance = (shared_source_point[0] - points[tidx]).norm() 
        if (min_distance > distance && distance < max_correspondence_distance) {
          coorespondence.query = points[tidx];
          min_distance = distance;
      }
        }
    }
  }
  if(min_distance < max_correspondence_distance){
    is_actualized = true;
  }
  return cuda::std:make_tuple(coorespondence, is_actualized)
}

/**
 *
 * @brief Does DataAssocaiation using the neareast neighbors search on the voxel hashmap setting up
 * the linear regression
 * @param points are they points you are using to query
 * @param voxel_map the voxel map you are querying
 * @param Registration object controls some of the parameters of building
 * @param double distance_threshold is the max regerence distance for checking if a point is going
 * to be assocciated
 * @return A list of point coorespondences
 */
__device__ __inline__ void DataAssociation(Eigen::Vector3dDNA *points, 
                                           GpuVoxelHashMap *voxel_map,
                                           const double max_correspondence_distance,
                                           const int N) {

  extern __shared__ HessianAndVector hessian_and_vector[1024];
  unsigned int tidx = threadIdx.x;
  unsigned int btidx = blockIdx.x;
  unsigned int gtidx = blockDim.x * blockIdx.x + threadIdx.x;
  if(gtidx< N){
    auto [coorespondence, actualized_coorespondence] =
      firstNearestNeighborQuery(voxel_map, points[gtidx], max_correspondence_distance);
      hessian_and_vector_shared[tidx] = hessian_and_vector(coorespondence);
    
  }
  else{
    hessian_and_vector_shared[tidx].zero();
  }
  __syncthreads(); 



}

__device__ __inline__ HessianAndVector BuildLinearSystem(Coorespondence correspondence,
                                  bool *actualized_coorespondences,
                                  const int N,
                                  const double kernel_scale) {
  Jacobian3x6 jacobian;
  HessianAndVector hv;
  double R[9] = {0};

  auto square = [] __device__(const double x) { return x * x; };
  auto GM_weight = [&] __device__(const double &residual2) {
    return square(kernel_scale) / square(kernel_scale + residual2);
  };

  double w = GM_weight(
        (shared_correspondences[tidx].source - shared_correspondences[tidx].query).squaredNorm());

  so3_hat(correspondence.query, R);
  jacobian.setIdentity();
  jacobian.setViaHat(R);
  hv.computeFromJacobianResidual(jacobian, residual, w);
  return hv
}

__device__ void FinalReduceLinearSystem(LinearSystem *linear_system, LinearSystem *final_linear_system,
                                   const int N, const double kernel_scale) {
  extern __shared__ LinearSystem shared_linear_system[1024];
  int tidx = threadIdx.x;
  int gtidx = blockDim.x * blockIdx.x + threadIdx.x;

  if (gtidx < N) {
    shared_linear_system[tidx] = linear_system[gtidx];
  } else {
    shared_linear_system[tidx].jacobian.zero();
    shared_linear_system[tidx].point = zero_vector;
  }
  __syncthreads();

  for (unsigned int j = blockDim.x / 2; j > 0; j >>= 1) {
    if (tidx < j) {
      shared_linear_system[tidx].hessian_and_vector.add(
          shared_linear_system[tidx + j].hessian_and_vector);
    }
    __syncthreads();
  }
  __syncwarps();
  final_linear_system[blockIdx.x] = shared_linear_system[0];
}

} // namespace

Sophus::SE3d GPURegistration::alignPointsToMap(const std::vector<Eigen::Vector3d> &points,
                                               const GPUVoxelHashMap &voxel_map,
                                               const Sophus::SE3d &initial_guess,
                                               double max_correspondence_distance,
                                               double kernel_scale) {
  size_t number_of_points = points.size();

  Vector3dDNA *device_points;
  cudaMalloc((void **)device_points, number_of_points * sizeof(vector3dDNA));

  Coorespondences *device_coorespondences;
  cudaMalloc((void **)device_correspondences, number_of_points * sizeof(Coorespondences));

  LinearSystem *linear_system_input;
  cudaMalloc((void **)linear_system_input, number_of_points * sizeof(LinearSystem));

  // never going to be fully used
  LinearSystem *linear_system_reduce_buffer;
  cudaMalloc((void **)linear_system_reduce_buffer, number_of_points * sizeof(LinearSystem));

  LinearSystem *linear_system_out;
  cudaMalloc((void **)linear_system_out, sizeof(linearSystem));

  cudaMemset(linear_system_input, 0, number_of_points * sizeof(LinearSystem));
  cudaMemset(linear_system_out, 0, sizeof(LinearSystem));

  cudaMemcpy(device_points, points.data(), sizeof(Eigen::Vector3dDNA) * number_of_points,
             cudaMemcpyHostToDevice);

  unsigned int threads_per_blocks_data_association =
      32; // pinned here as were doing 27 points per voxel threads needs to be pinned to efficiently
          // using shared memory
  unsigned int blocks_data_association = number_of_points / threads_per_blocks_data_association + 1;

  unsigned int threads_per_blocks_linear_system = 4096;
  unsigned int blocks_linear_system = number_of_points / threads_per_blocks_linear_system + 1;

  for (int i = 0; i < num_iterations_; ++i) {
    unsigned int linear_system_thread_blocks = blocks_linear_system;
    unsigned int number_of_points_reduce_count = number_of_points;


    BuildLinearSystem<<<blocks_linear_sytem, threads_per_blocks_linear_system>>>(
        correspondences, linear_system_input, kernel_scale, number_of_points);
    };


    ReduceLinearSystem<<<blocks_linear_system, threads_per_blocks_linear_system>>>(
          linear_system_input, linear_system_reduce_buffer, number_of_points_reduce_count);
      number_of_points_reduce_count = linear_system_thread_blocks;
      linear_system_thread_blocks = (linear_system_thread_blocks + 1) / 2;
    };

    cudaMemcpy(linear_system_out, linear_system_host, sizeof(linearSystem), cudaMemcpyDeviceToHost);
    cudaMemset(device_coorespondences, 0,
               number_of_points * sizeof(CorrespondencesLinearSystem)); // reset values
    cudaMemset(linear_system_input, 0, number_of_points * sizeof(LinearSystem));
  }

  // free gpu registration memory
  cudaFree(device_points);
  cudaFree(device_correspondences);
  cudaFree(linear_system_input);
  cudaFree(linear_system_reduce_buffer);
  cudaFree(linear_system_out);
  return initial_guess * delta_icp;
}
