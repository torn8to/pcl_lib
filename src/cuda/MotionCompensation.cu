#include "cuda/MotionCompensation.cuh"

namespace{
/**
* @brief GPU kernel to perform lie algebra motion compensation as is per


* @param points[in] A pointer to the points f tyupe Vector3dDNA using eigen
* @param output_points[out] A the result of motion compensation process Vector3dDNA
* @param timestamps[in] A the result of motion compensation
* @param start_time[in] A the result of motion compensation
* @param end_time[in] A the result of motion compensation
* @param N[in] is the amount of points
*/
__global__ void motionCompensate(Vector3dDNA *points, Vector3dDNA *output_points,
                                 double *timestamps, Vector6dDNA *omega, const double start_time,
                                 const double end_time, const int N) {
  unsigned int gtidx = blockDim.x * blockIdx.x + threadIdx.x;

  if (gtidx >= N) {
    return;
  }

  double R[9];
  double T[3];

  double norm_time = (timestamps[gtidx] - start_time) / (end_time - start_time);

  Vector6dDNA scaled_twist; 
  for (int i = 0; i < 6; i++) {
    scaled_twist[i] = omega[0][i] * norm_time;
  }

  se3_exp(&scaled_twist, R, T);
  se3_point_multiply(R, T, &points[gtidx], &output_points[gtidx]);
}

} // namespace

/**
 * @brief Performs motion compensation on a point cloud using timestamps and transformation based
 *
 * This function compensates for motion during the LiDAR scan by applying a transformation
 * to each point based on its timestamp relative to the scan start time. The transformation
 * is interpolated using the time difference between the current point and the first point.
 *
 * @param cloud The input point cloud to be motion compensated
 * @param cloud_timestamps Vector of timestamps corresponding to each point in the cloud
 * @param diff_transformation The differential transformation to apply (typically velocity * dt)
 * @return std::vector<Eigen::Vector3d> The motion-compensated point cloud
 * @note The cloud and timestamps vectors must have the same size
 * @note The diff_transformation should represent the motion between consecutive timestamps
 */



std::vector<Eigen::Vector3d> motionDeSkewGpu(const std::vector<Eigen::Vector3d> &cloud,
                                             const std::vector<double> &cloud_timestamps,
                                             const Sophus::SE3d &relative_motion) {

  assert(cloud.size() == cloud_timestamps.size() && "Cloud and timestamps must have the same size");
  std::size_t num_points = cloud.size();
  std::vector<Eigen::Vector3d> output_data(num_points);

  Vector3dDNA *points_pt;
  cudaMalloc((void **)&points_pt, sizeof(Vector3dDNA) * cloud.size());

  Vector3dDNA *points_out_pt;
  cudaMalloc((void **)&points_out_pt, sizeof(Vector3dDNA) * cloud.size());

  Vector6dDNA omega = relative_motion.log();
  Vector6dDNA *relative_motion_pt;
  cudaMalloc((void **)&relative_motion_pt, sizeof(Vector6dDNA));

  double *timestamps_pt;
  cudaMalloc((void **)&timestamps_pt, sizeof(double) * cloud_timestamps.size());

  // timestamps_for normalization of time
  double begin_time = cloud_timestamps.front();
  double last_time = cloud_timestamps.back();

  cudaMemcpy(points_pt, cloud.data(), sizeof(Eigen::Vector3d) * cloud.size(),
             cudaMemcpyHostToDevice);
  cudaMemcpy(timestamps_pt, cloud_timestamps.data(), sizeof(double) * cloud_timestamps.size(),
             cudaMemcpyHostToDevice);
  cudaMemcpy(relative_motion_pt, omega.data(), sizeof(Vector6dDNA), cudaMemcpyHostToDevice);

  unsigned int threads_per_blocks = 4096;
  unsigned int number_of_blocks = (num_points + threads_per_blocks - 1) / threads_per_blocks;

  dim3 blocksDim(number_of_blocks);
  dim3 threadsDim(threads_per_blocks);

  // put kernel method here
  motionCompensate<<<blocksDim, threadsDim>>>(points_pt, points_out_pt, timestamps_pt,
                                              relative_motion_pt, begin_time, last_time,
                                              num_points);

  cudaMemcpy(output_data.data(), points_out_pt, sizeof(Eigen::Vector3d) * cloud.size(),
             cudaMemcpyDeviceToHost);
  cudaFree(points_pt);
  cudaFree(timestamps_pt);
  cudaFree(points_out_pt);
  cudaFree(relative_motion_pt);

  return output_data;
}

std::vector<Eigen::Vector3d> motionDeSkewGpuAsync(const std::vector<Eigen::Vector3d> &cloud,
                                             const std::vector<double> &cloud_timestamps,
                                             const Sophus::SE3d &relative_motion) {

  assert(cloud.size() == cloud_timestamps.size() && "Cloud and timestamps must have the same size");
  std::size_t num_points = cloud.size();
  std::vector<Eigen::Vector3d> output_data(num_points);

  // Constants for chunking
  const size_t CHUNK_SIZE = 500000; // Adjust based on GPU memory/performance
  const int NUM_STREAMS = 4;
  
  // Register host memory for pinning (allows async memcpy)
  // We accept that this might fail on some systems, but for benchmark we assume success or ignore
  cudaHostRegister((void*)cloud.data(), cloud.size() * sizeof(Eigen::Vector3d), cudaHostRegisterDefault);
  cudaHostRegister((void*)cloud_timestamps.data(), cloud_timestamps.size() * sizeof(double), cudaHostRegisterDefault);
  cudaHostRegister((void*)output_data.data(), output_data.size() * sizeof(Eigen::Vector3d), cudaHostRegisterDefault);

  // BETTER APPROACH: Allocate N stream buffers on GPU of CHUNK_SIZE.
  
  Vector3dDNA *d_points[NUM_STREAMS];
  Vector3dDNA *d_points_out[NUM_STREAMS];
  double *d_timestamps[NUM_STREAMS];
  
  for(int i=0; i<NUM_STREAMS; ++i) {
      cudaMalloc((void **)&d_points[i], sizeof(Vector3dDNA) * CHUNK_SIZE);
      cudaMalloc((void **)&d_points_out[i], sizeof(Vector3dDNA) * CHUNK_SIZE);
      cudaMalloc((void **)&d_timestamps[i], sizeof(double) * CHUNK_SIZE);
  }

  Vector6dDNA omega = relative_motion.log();
  Vector6dDNA *relative_motion_pt;
  cudaMalloc((void **)&relative_motion_pt, sizeof(Vector6dDNA));
  cudaMemcpy(relative_motion_pt, omega.data(), sizeof(Vector6dDNA), cudaMemcpyHostToDevice); // Constant, sync is fine

  double begin_time = cloud_timestamps.front();
  double last_time = cloud_timestamps.back();

  cudaStream_t streams[NUM_STREAMS];
  for (int i = 0; i < NUM_STREAMS; ++i) {
      cudaStreamCreate(&streams[i]);
  }

  size_t num_chunks = (num_points + CHUNK_SIZE - 1) / CHUNK_SIZE;

  for (size_t i = 0; i < num_chunks; ++i) {
      int stream_idx = i % NUM_STREAMS;
      size_t offset = i * CHUNK_SIZE;
      size_t current_chunk_size = std::min(CHUNK_SIZE, num_points - offset);

      // Async H2D
      cudaMemcpyAsync(d_points[stream_idx], cloud.data() + offset, 
                      sizeof(Eigen::Vector3d) * current_chunk_size, 
                      cudaMemcpyHostToDevice, streams[stream_idx]);
      
      cudaMemcpyAsync(d_timestamps[stream_idx], cloud_timestamps.data() + offset, 
                      sizeof(double) * current_chunk_size, 
                      cudaMemcpyHostToDevice, streams[stream_idx]);

      // Kernel
      unsigned int threads_per_blocks = 512;
      unsigned int number_of_blocks = (current_chunk_size + threads_per_blocks - 1) / threads_per_blocks;
      
      motionCompensate<<<number_of_blocks, threads_per_blocks, 0, streams[stream_idx]>>>(
          d_points[stream_idx], d_points_out[stream_idx], d_timestamps[stream_idx], 
          relative_motion_pt, begin_time, last_time, current_chunk_size);

      // Async D2H
      cudaMemcpyAsync(output_data.data() + offset, d_points_out[stream_idx], 
                      sizeof(Eigen::Vector3d) * current_chunk_size, 
                      cudaMemcpyDeviceToHost, streams[stream_idx]);
  }

  // Cleanup
  for (int i = 0; i < NUM_STREAMS; ++i) {
      cudaStreamSynchronize(streams[i]);
      cudaStreamDestroy(streams[i]);
      cudaFree(d_points[i]);
      cudaFree(d_points_out[i]);
      cudaFree(d_timestamps[i]);
  }
  cudaFree(relative_motion_pt);

  // Unregister host memory
  cudaHostUnregister((void*)cloud.data());
  cudaHostUnregister((void*)cloud_timestamps.data());
  cudaHostUnregister((void*)output_data.data());

  return output_data;
}