#include "cuda/MotionCompensation.cuh"

namespace {
/**
 * @brief Compute the so3 hat
 *
 * @param twist[in] the twist were deriving the hat operator from
 * @param R[out] the sekew sekemmetic so23 aht matrix
 */
__device__ __inline__ void so3_hat(const Vector6dDNA *twist, double R[9]) {
  double wx = twist[0][0];
  double wy = twist[0][1];
  double wz = twist[0][2];

  R[0] = 0.0;
  R[1] = -wz;
  R[2] = wy;
  R[3] = wz;
  R[4] = 0.0;
  R[5] = -wx;
  R[6] = -wy;
  R[7] = wx;
  R[8] = 0.0;
}

/**
 * @brief Compute the so3 hat squared
 */
__device__ void so3_hat_squared(const Vector6dDNA *twist, double R_squared[9]) {
  double wx = twist[0][0];
  double wy = twist[0][1];
  double wz = twist[0][2];

  // R^2 = omega * omega^T - ||omega||^2 * I
  double wx2 = wx * wx;
  double wy2 = wy * wy;
  double wz2 = wz * wz;
  double norm_sq = wx2 + wy2 + wz2;

  R_squared[0] = wx * wx - norm_sq;
  R_squared[1] = wx * wy;
  R_squared[2] = wx * wz;
  R_squared[3] = wy * wx;
  R_squared[4] = wy * wy - norm_sq;
  R_squared[5] = wy * wz;
  R_squared[6] = wz * wx;
  R_squared[7] = wz * wy;
  R_squared[8] = wz * wz - norm_sq;
}

/**
 * @brief Compute se3 hat (rotation and translation components)
 */
__device__ __inline__ void se3_hat(const Vector6dDNA *twist, double R[9], double T[3]) {
  so3_hat(twist, R);
  T[0] = twist[0][3];
  T[1] = twist[0][4];
  T[2] = twist[0][5];
}

/**
 * @brief Compute SE3 exponential map using Rodrigues formula
 */
__device__ __inline__ void se3_exp(const Vector6dDNA *twist, double R[9], double T[3]) {
  // Extract rotation and translation parts
  double wx = twist[0][0];
  double wy = twist[0][1];
  double wz = twist[0][2];
  double vx = twist[0][3];
  double vy = twist[0][4];
  double vz = twist[0][5];

  double theta = sqrt(wx * wx + wy * wy + wz * wz);

  if (theta < 1e-8) {
    // Small angle approximation
    R[0] = 1.0;
    R[1] = -wz;
    R[2] = wy;
    R[3] = wz;
    R[4] = 1.0;
    R[5] = -wx;
    R[6] = -wy;
    R[7] = wx;
    R[8] = 1.0;

    T[0] = vx;
    T[1] = vy;
    T[2] = vz;
  } else {
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    double a = sin_theta / theta;
    double b = (1.0 - cos_theta) / (theta * theta);

    // hat
    double omega_hat[9];
    so3_hat(twist, omega_hat);

    // hat^2
    double omega_hat_sq[9];
    so3_hat_squared(twist, omega_hat_sq);

    // R = I + a*hat + b*hat^2
    for (int i = 0; i < 9; i++) {
      R[i] = (i % 4 == 0 ? 1.0 : 0.0) + a * omega_hat[i] + b * omega_hat_sq[i];
    }

    // V = I + ((1-cos(theta))/theta^2)*[omega]_x + ((theta-sin(theta))/theta^3)*[omega]_x^2
    double c = (theta - sin_theta) / (theta * theta * theta);

    // T = V * v
    T[0] = vx + b * (wy * vz - wz * vy) +
           c * (wx * (wx * vx + wy * vy + wz * vz) - vx * (wx * wx + wy * wy + wz * wz));
    T[1] = vy + b * (wz * vx - wx * vz) +
           c * (wy * (wx * vx + wy * vy + wz * vz) - vy * (wx * wx + wy * wy + wz * wz));
    T[2] = vz + b * (wx * vy - wy * vx) +
           c * (wz * (wx * vx + wy * vy + wz * vz) - vz * (wx * wx + wy * wy + wz * wz));
  }
}

/**
 * @brief Apply SE3 transformation: point_out = R * T + point_in
 * @param R[in] rotation amtrix of the se3 lie group
 * @param T[in] translation part of the se3 group
 * @param point_in[in] the point passed in
 * @param point_out[out] the point with the transformation applied
 */
__device__ void se3_point_multiply(const double R[9], const double T[3],
                                   const Vector3dDNA *point_in, Vector3dDNA *point_out) {
  double px = point_in[0][0];
  double py = point_in[0][1];
  double pz = point_in[0][2];

  point_out[0][0] = R[0] * px + R[1] * py + R[2] * pz + T[0];
  point_out[0][1] = R[3] * px + R[4] * py + R[5] * pz + T[1];
  point_out[0][2] = R[6] * px + R[7] * py + R[8] * pz + T[2];
}

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

  unsigned int threads_per_blocks = 512;
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