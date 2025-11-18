#include <thrust/iterator/zip_iterator.h>
// #include <thrust/pair.h>
#include <thrust/tuple.h>

#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <thrust/reduce.h>


#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <vector_types.h>

//Eigen::DontAlign
using Vector3Gpu = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;



std::vector<Eigen::Vector3d> motionDeSkewThrust(std::vector<Eigen::Vector3d>, const Sophus::SE3d relative_motion);
