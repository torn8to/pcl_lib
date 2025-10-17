#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <Eigen/Dense>

namespace fs = std::filesystem;

std::vector<Eigen::Vector4f> loadLidarBin(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + path);
    }

    std::vector<Eigen::Vector4f> points;
    Eigen::Vector4f p;
    while (file.read(reinterpret_cast<char*>(p.data()), sizeof(float) * 4)) {
        points.push_back(p);
    }
    return points;
}

loadKitti360Sequence(const std::string& dataset_dir,
                     const std::string& lidar_dir="dataset_dir",
                     int start_index = 0,
                     int n = -1)
{
    std::vector<std::vector<Eigen::Vector4f>> frames;
    if (start_index < 0 || start_index >= static_cast<int>(timestamps.size())) {
        throw std::out_of_range("start_index is out of range");
    }
    int end_index = std::min<int>(start_index + n, timestamps.size());
    frames.reserve(end_index - start_index);

    for (int i = start_index; i < end_index; ++i) {
        const std::string& ts = timestamps[i];
        fs:path path =  

        if (!fs::exists(path)) {
            std::cerr << "Warning: missing file " << path << "\n";
            continue;
        }

        std::cout << "Loading [" << i << "] " << ts << ".bin" << std::endl;
        auto cloud = loadLidarBin(path);
        std::cout << "  → " << cloud.size() << " points\n";
        frames.push_back(std::move(cloud));
    }

    return frames;
}


std::vector<Eigen::Vector3f> convertVec4fToVec3f(const std::vector<Eigen::Vector4f>& vec4f) {
    std::vector<Eigen::Vector3f> vec3f;
    vec3f.reserve(vec4f.size());
    for (const auto& v : vec4f) {
        vec3f.emplace_back(v.x(), v.y(), v.z());
    }
    return vec3f;
}
