#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "src/Pipeline.hpp"
#include "src/VoxelMap.hpp"
#include "src/cuda/GPURegistration.cuh"
#include "src/cuda/GPUVoxelHashMap.cuh"

#include "Timer.hpp"

#include <cuda_runtime.h>

namespace fs = std::filesystem;

static void usage(const char *prog) {
  std::cerr << "Usage: " << prog << " --sequence <dir> --start <idx> [--repeat N] [--sigma S]"
            << std::endl;
}

static std::vector<Eigen::Vector3d> load_cloud_txt(const fs::path &file) {
  std::vector<Eigen::Vector3d> pts;
  std::ifstream ifs(file);
  if (!ifs)
    return pts;
  pts.reserve(200000);
  double x, y, z;
  while (ifs >> x >> y >> z) {
    pts.emplace_back(x, y, z);
  }
  return pts;
}

static std::vector<fs::path> list_frames(const fs::path &dir) {
  std::vector<fs::path> files;
  for (auto &p : fs::directory_iterator(dir)) {
    if (!p.is_regular_file())
      continue;
    // accept any extension; user can curate directory
    files.push_back(p.path());
  }
  std::sort(files.begin(), files.end());
  return files;
}

int main(int argc, char **argv) {
  fs::path seq_dir;
  int start = -1;
  int repeat = 10;
  double sigma = 0.5; // kernel scale; correspondence radius is 3*sigma

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--sequence" && i + 1 < argc) {
      seq_dir = argv[++i];
    } else if (a == "--start" && i + 1 < argc) {
      start = std::stoi(argv[++i]);
    } else if (a == "--repeat" && i + 1 < argc) {
      repeat = std::stoi(argv[++i]);
    } else if (a == "--sigma" && i + 1 < argc) {
      sigma = std::stod(argv[++i]);
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    }
  }

  if (seq_dir.empty() || start < 0) {
    usage(argv[0]);
    return 1;
  }

  std::vector<fs::path> frames = list_frames(seq_dir);
  if (frames.empty() || start >= static_cast<int>(frames.size())) {
    std::cerr << "No frames or start index out of range" << std::endl;
    return 1;
  }

  // Build map with Pipeline up to start frame (exclusive)
  cloud::PipelineConfig cfg; // defaults
  cloud::Pipeline pipeline(cfg);

  for (int i = 0; i < start; ++i) {
    std::vector<Eigen::Vector3d> cloud = load_cloud_txt(frames[i]);
    auto pruned = pipeline.removeFarPoints(cloud);
    pipeline.odometryUpdate(pruned, Sophus::SE3d(), false);
  }

  // Extract built map and construct separate CPU and GPU voxel maps
  std::vector<Eigen::Vector3d> map_points = pipeline.getMap();

  const double voxel_resolution =
      (cfg.max_distance / cfg.voxel_factor) * cfg.voxel_resolution_alpha;
  const int max_points_per_voxel = cfg.max_points_per_voxel;

  cloud::VoxelMap cpu_map(voxel_resolution, cfg.max_distance, max_points_per_voxel);
  cpu_map.addPoints(map_points);

  GPUVoxelHashMapDefault gpu_map(voxel_resolution,
                                 static_cast<int>(std::max<size_t>(map_points.size(), 100000)),
                                 cfg.max_distance);
  gpu_map.addPoints(map_points);

  // Source cloud to register against the fixed maps
  std::vector<Eigen::Vector3d> source = load_cloud_txt(frames[start]);
  source = pipeline.removeFarPoints(source);

  const double corr_radius = 3.0 * sigma;
  const Sophus::SE3d init_guess; // identity

  // CPU one-iteration ICP timing
  Registration cpu_icp(/*num_iterations=*/1, /*convergence=*/1e-4, /*num_threads=*/cfg.num_threads);
  auto t0 = std::chrono::steady_clock::now();
  for (int r = 0; r < repeat; ++r) {
    (void)cpu_icp.alignPointsToMap(source, cpu_map, init_guess, corr_radius, sigma);
  }
  auto t1 = std::chrono::steady_clock::now();
  const auto cpu_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

  // GPU one-iteration ICP timing
  GPURegistration<27, 100000> gpu_icp(/*max_iterations=*/1, /*convergence=*/1e-4);
  auto g0 = std::chrono::steady_clock::now();
  for (int r = 0; r < repeat; ++r) {
    (void)gpu_icp.alignPointsToMap(source, gpu_map, init_guess, corr_radius, sigma);
  }
  cudaDeviceSynchronize();
  auto g1 = std::chrono::steady_clock::now();
  const auto gpu_ms = std::chrono::duration_cast<std::chrono::microseconds>(g1 - g0).count();

  std::cout << "Frames: " << frames.size() << ", start=" << start
            << ", map_pts=" << map_points.size() << ", src_pts=" << source.size() << "\n";
  std::cout << "CPU 1-iter ICP: total " << cpu_ms / 1000.0 << " ms, avg "
            << (cpu_ms / double(repeat)) << " us\n";
  std::cout << "GPU 1-iter ICP: total " << gpu_ms / 1000.0 << " ms, avg "
            << (gpu_ms / double(repeat)) << " us\n";

  return 0;
}
