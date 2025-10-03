#include "GPUVoxelHashMap.cuh"
#include <cuda_runtime.h>



namespace{


}


template <int PointsPerVoxel, int InitialCapacity>
void GPUVoxelHashMap<PointsPerVoxel, InitialCapacity>::addPoints(const std::vector<Eigen::Vector3d>& points) {
    auto map_view = map_.get_device_view();
    thrust::device_vector<Eigen::Vector3d> points_device(points.begin(), points.end());
    double resolution_spacing = std::sqrt((resolution_ * resolution_) / static_cast<double>(PointsPerVoxel));
    thrust::for_each(thrust::device,
                    points_device.begin(),
                    points_device.end(), 
    [resolution_spacing, this, map_view] __device__ (const Eigen::Vector3d& point){
        const Voxel voxel = PointToVoxel(point, this->resolution_);
        auto query = map_view.find(voxel);
        if(query == map_view.end()){
            Value voxel_vec; 
            voxel_vec.push_back(point);
            map_view.insert({voxel, std::move(voxel_vec)});
        }
        else{
            auto& voxel_vector = query->second();
            if(!(voxel_vector.size() == voxel_vector.max_size())){
                if(thrust::all_of(
                    thrust::device,
                    voxel_vector.begin(),
                    voxel_vector.end(),
                    [&] __device__ (const auto& vector_point){
                        return (vector_point - point).norm() > resolution_spacing;
                    })
                ){
                        voxel_vector.push_back(point);
                }
            }
        }
    });
}

/** 
    commented out and the code is used in registration as having the 
    code here would cause an issue with vectorization as such that code has been moved to regisstration

template <int PointsPerVoxel>
__device__ std::pair<Eigen::Vector3d, double> GPUVoxelHashMap<PointsPerVoxel>::firstNearestNeighborQuery(Eigen::Vector3d const& query_point) {
    Eigen::Vector3i voxel = PointToVoxel(query_point, resolution_);
    Eigen::Vector3d closest_point = Eigen::Vector3d::Zero();
    double minimum_distance = cuda::std::numeric_limits<double>::max();
    for(auto& voxel_shift: voxel_shifts){
      auto it = this->map.find(voxel + voxel_shift);
      if (it == this->map.end()) continue;
      auto& voxel_data = it->second();
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

