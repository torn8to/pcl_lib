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
  // Create directory if it doesn't exist
  std::filesystem::path file_path(file_name);
  std::filesystem::create_directories(file_path.parent_path());
  
  printf("Opening file: %s\n", file_name.c_str());
  fflush(stdout);
  std::ofstream file(file_name, std::ios::binary);
  if(!file){
    printf("Failed to open file: %s\n", file_name.c_str());
    return false;
  }

  size_t num_points = points.size();
  printf("Writing %zu points to file\n", num_points);
  fflush(stdout);
  
  // write number of points and the send data
  file.write(reinterpret_cast<const char*>(&num_points), sizeof(size_t)); 
  if (!file.good()) {
    printf("Error writing num_points\n");
    return false;
  }
  
  file.write(reinterpret_cast<const char*>(points.data()), sizeof(Eigen::Vector3d) * points.size());
  if (!file.good()) {
    printf("Error writing point data\n");
    return false;
  }
  
  file.close();
  printf("File written successfully\n");
  fflush(stdout);
  return true;
}

int main(){
  cloud::PipelineConfig cfg;
  cloud::Pipeline pipeline(cfg);
  
  std::string kitti_dir = "../KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data";
  LazyPointCloudLoader loader(kitti_dir);

  unsigned int start = 0;
  unsigned int iterations = 400;

  for(unsigned int i = start ; i < iterations; ++i){
    std::vector<Eigen::Vector3d> loaded_cloud =  loader.loadNext();
    printf("%u \n", i);
    pipeline.odometryUpdate(loaded_cloud, Sophus::SE3d(), false);
  }

  printf("About to call getMap()...\n");
  fflush(stdout);
  std::vector<Eigen::Vector3d> current_map = pipeline.getMap();
  printf("getMap() returned, size = %u\n", static_cast<unsigned int>(current_map.size()));
  fflush(stdout);
  save_points_toxyzbin(current_map, "data/frame_500Kitti360.bin");
  return 1;
}
