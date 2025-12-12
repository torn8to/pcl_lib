
#include <thrust/iterator/zip_iterator.h>
#include <thrust/pair.h>
#include <thrust/tuple.h>

#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <thrust/reduce.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <vector_types.h>

using Vector3dDNA = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;

__device__ __inline__ void transform(
        Eigen::Vector3dDNA *points_in, 
        Eigen::Vector3dDNA *points_out,
        Sophus::SE3d *transform, int N) {
unsigned int gtidx = blockDim.x * blockidx.x + threadIdx.x;
points_out[gtidx] = transform[0] * points_in[gtidx];
}

__device__ __inline__ void transformInplace(
        Eigen::Vector3dDNA *points_in,
        Sophus::SE3d *transform,
        int N) {
unsigned int gtidx = blockDim.x * blockidx.x + threadIdx.x;
points_in[gtidx] = transform[0] * points_in[gtidx];
}
