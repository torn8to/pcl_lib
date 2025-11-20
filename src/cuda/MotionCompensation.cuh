#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <vector_types.h>

// Eigen::DontAlign version for the gpu version as it is POD
using Vector3dDNA = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Vector6dDNA = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;

std::vector<Eigen::Vector3d> motionDeSkewGpu(const std::vector<Eigen::Vector3d> &points,
                                             const std::vector<double> &cloud_timestamps,
                                             const Sophus::SE3d &relative_motion);
