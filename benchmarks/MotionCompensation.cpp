#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>
#include <sophus/se3.hpp>

#include "MotionCompensation.hpp"
#include "cuda/MotionCompensation.cuh"
#include "timer.hpp" // Assuming this exists based on Transform.cpp

// Helper to generate random points
std::vector<Eigen::Vector3d> generateRandomCloud(size_t num_points) {
    std::vector<Eigen::Vector3d> cloud;
    cloud.reserve(num_points);
    std::mt19937 gen(42);
    std::uniform_real_distribution<> dis(-100.0, 100.0);
    for (size_t i = 0; i < num_points; ++i) {
        cloud.emplace_back(dis(gen), dis(gen), dis(gen));
    }
    return cloud;
}

// Helper to generate random timestamps
std::vector<double> generateTimestamps(size_t num_points, double duration) {
    std::vector<double> timestamps;
    timestamps.reserve(num_points);
    // Linear distribution for simplicity, or random? 
    // Usually timestamps are somewhat ordered in a scan, but let's just make them linear 0 to duration
    for (size_t i = 0; i < num_points; ++i) {
        timestamps.push_back((static_cast<double>(i) / num_points) * duration);
    }
    return timestamps;
}

int main() {
    std::vector<size_t> scales = {1000, 10000, 100000, 1000000, 2000000, 5000000, 10000000, 20000000};
    
    // Simulate some motion: 1 meter forward, 10 degrees yaw over 0.1 seconds
    Sophus::SE3d relative_motion(
        Sophus::SO3d::exp(Eigen::Vector3d(0, 0, 0.17)), // ~10 degrees z
        Eigen::Vector3d(1.0, 0.0, 0.0)
    );
    double scan_duration = 0.1;

    std::cout << "Scale\t\tCPU(ms)\tGPU(ms)\tGPU Async(ms)\n";
    std::cout << "--------------------------------\n";

    for (size_t scale : scales) {
        auto cloud = generateRandomCloud(scale);
        auto timestamps = generateTimestamps(scale, scan_duration);

        // CPU Benchmark
        auto start_cpu = std::chrono::high_resolution_clock::now();
        // The CPU signature: motionDeSkew(cloud, timestamps, relative_motion)
        // Check if timestamps is reference or const reference in header. 
        // Based on search it was `std::vector<double> &cloud_timestamps` (non-const ref? odd but okay).
        // Let's create a copy if needed or just use it.
        auto result_cpu = cloud::motionDeSkew(cloud, timestamps, relative_motion);
        auto end_cpu = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration_cpu = end_cpu - start_cpu;

        // GPU Benchmark
        // The GPU signature: motionDeSkewGpu(points, timestamps, relative_motion)
        auto start_gpu = std::chrono::high_resolution_clock::now();
        auto result_gpu = motionDeSkewGpu(cloud, timestamps, relative_motion);
        auto end_gpu = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration_gpu = end_gpu - start_gpu;

        std::cout << scale << "\t\t" << duration_cpu.count() << "\t" << duration_gpu.count();

        // Async GPU Benchmark
        auto start_gpu_async = std::chrono::high_resolution_clock::now();
        auto result_gpu_async = motionDeSkewGpuAsync(cloud, timestamps, relative_motion);
        auto end_gpu_async = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration_gpu_async = end_gpu_async - start_gpu_async;
        
        std::cout << " " << duration_gpu_async.count() << "\n";

        // Prevent optimization
        if (result_cpu.empty() || result_gpu.empty() || result_gpu_async.empty()) {
             std::cerr << "Error: empty result\n";
        }
    }

    return 0;
}
