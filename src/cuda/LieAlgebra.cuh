#include "cuda/MotionCompensation.cuh"

namespace {
/**
 * @brief Compute the so3 hat
 *
 * @param twist[in] tangent space vector for se3
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
 * @brief Compute the so3 hat
 *
 * @param tangent[in] the vector for so3
 * @param R[out] the sekew sekemmetic so23 aht matrix
 */
__device__ __inline__ void so3_hat(const Vector3dDNA *tangent, double R[9]) {
  double wx = tangent[0][0];
  double wy = tangent[0][1];
  double wz = tangent[0][2];

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
__device__ __inline__ void so3_hat_squared(const Vector6dDNA *twist, double R_squared[9]) {
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
__device__ __inline__ void se3_point_multiply(const double R[9], const double T[3],
                                   const Vector3dDNA *point_in, Vector3dDNA *point_out) {
  double px = point_in[0][0];
  double py = point_in[0][1];
  double pz = point_in[0][2];

  point_out[0][0] = R[0] * px + R[1] * py + R[2] * pz + T[0];
  point_out[0][1] = R[3] * px + R[4] * py + R[5] * pz + T[1];
  point_out[0][2] = R[6] * px + R[7] * py + R[8] * pz + T[2];
}


} // namespace


/**
 * @brief Compute the so3 hat squared
 */
__device__ __inline__ void so3_hat_squared(const Vector6dDNA *twist, double R_squared[9]) {
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
__device__ __inline__ void se3_point_multiply(const double R[9], const double T[3],
                                   const Vector3dDNA *point_in, Vector3dDNA *point_out) {
  double px = point_in[0][0];
  double py = point_in[0][1];
  double pz = point_in[0][2];

  point_out[0][0] = R[0] * px + R[1] * py + R[2] * pz + T[0];
  point_out[0][1] = R[3] * px + R[4] * py + R[5] * pz + T[1];
  point_out[0][2] = R[6] * px + R[7] * py + R[8] * pz + T[2];
}


} // namespace
