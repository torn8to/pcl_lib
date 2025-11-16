#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <Eigen/Dense>
#include <sophus/se3.hpp>


#include "Pipeline.hpp"
#include "Registration.hpp"
#include "VoxelMap.hpp"
#include "Kitti360Loader.hpp"


bool save_points_toxyzbin(std::vector<Eigen::Vector3d> &points, std::string file_name){
  std::ofstream file(file_name, std::ios::binary);
  if(!file){
    printf("no file was loaded");
    return false;
  }

  size_t num_points = points.size();
  // write number of points and the send data
  file.write(reinterpret_cast<const char*>(num_points), sizeof(size_t)); 
  file.write(reinterpret_cast<const char*>(points.data()), sizeof(Eigen::Vector3d) * points.size());
  file.close();
  return true;
}

int main(){
  cloud::PipelineConfig cfg;
  cloud::Pipeline pipeline(cfg);
  
  std::string kitti_dir = "../KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data";
  LazyPointCloudLoader loader(kitti_dir);

  unsigned int start = 0;
  unsigned int iterations = 10;

  for(unsigned int i = start ; i < iterations; ++i){
    std::vector<Eigen::Vector3d> loaded_cloud =  loader.loadNext();
    printf("%u \n", i);
    pipeline.odometryUpdate(loaded_cloud, Sophus::SE3d(), false);
  }

  printf("number of iterations = %u", static_cast<unsigned int>(current_map.size()));
  std::vector<Eigen::Vector3d> current_map = pipeline.getMap();
  printf("number of points = %u", static_cast<unsigned int>(current_map.size()));
  save_points_toxyzbin(current_map, "data/frame_500Kitti360.bin");
  return 1;
}
