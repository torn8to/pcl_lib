#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <Eigen/Dense>

namespace fs = std::filesystem;

class LazyPointCloudLoader {
private:
    std::vector<std::string> file_paths_;
    size_t current_index_;
    
public:
    LazyPointCloudLoader(const std::string& directory_path) 
        : current_index_(0) {
        loadFileList(directory_path);
    }
    
    // Load list of .bin files from directory
    void loadFileList(const std::string& directory_path) {
        file_paths_.clear();
        
        if (!fs::exists(std::filesystem::absolute(directory_path))) {
            throw std::runtime_error("Directory does not exist: " + directory_path);
        }
        
        for (const auto& entry : fs::directory_iterator(directory_path)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                // KITTI-360 typically uses .bin files for point clouds
                if (ext == ".bin" || ext == ".txt") {
                    file_paths_.push_back(entry.path().string());
                }
            }
        }
        
        // Sort files alphabetically
        std::sort(file_paths_.begin(), file_paths_.end());
        
        std::cout << "Found " << file_paths_.size() << " files" << std::endl;
    }
    
    // Load point cloud from binary file (KITTI format: x, y, z, intensity as floats)
    std::vector<Eigen::Vector3d> loadBinaryFile(const std::string& file_path) {
        std::ifstream file(file_path, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + file_path);
        }
        
        // Get file size
        file.seekg(0, std::ios::end);
        size_t file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        
        // KITTI point cloud format: each point is 4 floats (x, y, z, intensity)
        size_t num_points = file_size / (4 * sizeof(float));
        
        std::vector<Eigen::Vector3d> points;
        points.reserve(num_points);
        
        std::vector<float> buffer(4);
        for (size_t i = 0; i < num_points; ++i) {
            file.read(reinterpret_cast<char*>(buffer.data()), 4 * sizeof(float));
            
            // Convert to Vector3d (ignoring intensity)
            points.emplace_back(
                static_cast<double>(buffer[0]),
                static_cast<double>(buffer[1]),
                static_cast<double>(buffer[2])
            );
        }
        
        file.close();
        return points;
    }
    
    // Load point cloud from text file
    std::vector<Eigen::Vector3d> loadTextFile(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + file_path);
        }
        
        std::vector<Eigen::Vector3d> points;
        double x, y, z;
        
        while (file >> x >> y >> z) {
            points.emplace_back(x, y, z);
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        
        file.close();
        return points;
    }
    
    // Lazy load next point cloud
    std::vector<Eigen::Vector3d> loadNext() {
        if (current_index_ >= file_paths_.size()) {
            throw std::runtime_error("No more files to load");
        }
        
        std::string file_path = file_paths_[current_index_++];
        std::string ext = fs::path(file_path).extension().string();
        
        std::cout << "Loading file " << current_index_ << "/" 
                  << file_paths_.size() << ": " << file_path << std::endl;
        
        if (ext == ".bin") {
            return loadBinaryFile(file_path);
        } else {
            return loadTextFile(file_path);
        }
    }
    
    // Check if more files are available
    bool hasNext() const {
        return current_index_ < file_paths_.size();
    }
    
    // Reset iterator
    void reset() {
        current_index_ = 0;
    }
    
    // Get total number of files
    size_t size() const {
        return file_paths_.size();
    }
    
    // Get specific file by index
    std::vector<Eigen::Vector3d> loadAtIndex(size_t index) {
        if (index >= file_paths_.size()) {
            throw std::out_of_range("Index out of range");
        }
        
        std::string file_path = file_paths_[index];
        std::string ext = fs::path(file_path).extension().string();
        
        std::cout << "Loading file at index " << index << ": " 
                  << file_path << std::endl;
        
        if (ext == ".bin") {
            return loadBinaryFile(file_path);
        } else {
            return loadTextFile(file_path);
        }
    }
    
    // Get file path at index
    const std::string& getFilePath(size_t index) const {
        if (index >= file_paths_.size()) {
            throw std::out_of_range("Index out of range");
        }
        return file_paths_[index];
    }
};

// Example usage
/*
int main() {
    try {
        std::string kitti_dir = "/path/to/KITTI-360/data_3d_raw/2013_05_28_drive_0000_sync/velodyne_points";
        LazyPointCloudLoader loader(kitti_dir);
        
        // Example 1: Iterate through all files lazily
        std::cout << "\n=== Loading files lazily ===" << std::endl;
        while (loader.hasNext()) {
            auto points = loader.loadNext();
            
            std::cout << "  Loaded " << points.size() << " points" << std::endl;
            
            // Process points
            if (!points.empty()) {
                std::cout << "  First point: (" 
                          << points[0].x() << ", "
                          << points[0].y() << ", "
                          << points[0].z() << ")" << std::endl;
                          
                // Example: Calculate centroid
                Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
                for (const auto& pt : points) {
                    centroid += pt;
                }
                centroid /= points.size();
                std::cout << "  Centroid: (" 
                          << centroid.x() << ", "
                          << centroid.y() << ", "
                          << centroid.z() << ")" << std::endl;
            }
            
            // For demonstration, break after first file
            break;
        }
        
        std::cout << "\n=== Loading specific file ===" << std::endl;
        loader.reset();
        if (loader.size() > 0) {
            auto points = loader.loadAtIndex(0);
            std::cout << "Loaded " << points.size() << " points from first file" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
*/
