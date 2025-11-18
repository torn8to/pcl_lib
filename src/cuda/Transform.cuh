
#include <thrust/iterator/zip_iterator.h>
#include <thrust/pair.h>
#include <thrust/tuple.h>

#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <thrust/reduce.h>

#include <sophus/se3.hpp>
#include <Eigen/Core>

#include <vector_types.h>

std::vector<Eigen::Vector3d> transformGPUThrust(std::vector<Eigen::Vector3d> &points,const Sophus::SE3d transform);
