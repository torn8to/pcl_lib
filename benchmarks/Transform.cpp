#include <Eigen/Dense>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include "GpuPipeline.hpp"

#include "stats.hpp"
#include "timer.hpp"

#include "scripts/Kitti360Loader.hpp"

bool timings_to_bin(std::vector<double> &timings, std::string file_name) {
  // Create directory if it doesn't exist
  std::filesystem::path file_path(file_name);
  std::filesystem::create_directories(file_path.parent_path());

  printf("Opening file: %s\n", file_name.c_str());
  fflush(stdout);
  std::ofstream file(file_name, std::ios::binary);
  if (!file) {
    printf("Failed to open file: %s\n", file_name.c_str());
    return false;
  }

  size_t num_points = timings.size();
  printf("Writing %zu points to file\n", num_points);
  fflush(stdout);

  file.write(reinterpret_cast<const char *>(timings.data()), sizeof(double) * timings.size());
  if (!file.good()) {
    printf("Error writing timing data\n");
    return false;
  }

  printf("File written successfully\n");
  fflush(stdout);
  return true;
}

int main() {
  cloud::PipelineConfig cfg;
  cloud::Pipeline pipeline(cfg);

  std::string kitti_dir =
      "../KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points/data";
  LazyPointCloudLoader loader(kitti_dir);

  unsigned int start = 0;
  unsigned int iterations = 11000;

  std::vector<double> timings;
  timings.reserve(iterations);

  ManualTimer iteration_timer;
  ManualTimer overall_pipeline_timer;
  SlidingWindowAverage<double> swa(5);

  overall_pipeline_timer.start();
  for (unsigned int i = start; i < iterations; ++i) {
    std::vector<Eigen::Vector3d> loaded_cloud = loader.loadNext();

    iteration_timer.start();
    double avg = swa.get_average();

    printf("\r iteration_number Average time %f iteration %u", time_double, i);
    fflush(stdout);
  }
  auto full_duration = overall_pipeline_timer.stop();
  double time_double = static_cast<double>(full_duration) / 1000000;
  double avg_iteration_time = time_double / static_cast<double>(iterations);
  printf("pipeline_time overall_time: %f, avg_iteration_time: %f", time_double, avg_iteration_time);
  timings_to_bin(timings, "data/cpu_pipeline_timestamps.bin");
  fflush(stdout);
  return 1;
}