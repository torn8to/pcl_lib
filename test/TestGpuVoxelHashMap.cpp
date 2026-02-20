#include "cuda/GPUVoxelHashMap.cuh"
#include "VoxelMap.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <algorithms>

#include <vector>
#include <tuple>


class FuzzedVoxelMapPointInsertionMatchCPUGPU: public testing::Test {
  protected:
    FuzzedMotionCompensationBehaviorMatchTestCPUGPU(){}

    ~FuzzedMotionCompensationBehaviorMatchTestCPUGPU() override {}

    void SetUp() override{
      int num_points = 100000;
      test_data.reserve(num_tests);
      for(unsigned int i = 0; i < num_tests; ++i){
        Sophus::SE3d random_motion = generate_random_transform(5.0);
        test_data.emplace_back(random_test_points);
      }
    }

    void TearDown() override{
      
    }

    std::vector<std::vector<Eigen::Vector3d>> test_data;
    unsigned int num_tests = 10;
};

TEST_F(FuzzedVoxelMapbehaviorPointAddBulkMatchTestCPUGPU, TestCPUGPUApproxBehaviorMatch){
    VoxelMap host_map;
    GpuVoxelHashMap gpu_map;
    for(auto test_it =  test_data.begin(); test_it != test_data.end(); ++test_it){
      std::vector<Eigen::Vector3d> points = *test_it;
      // eval host_map and device_map similarity
      std::vector<Eigen::Vector3i> host_map_voxels = host_map.getVoxels();
      bool has_same_points =True;
      for(auto it =  host_map_voxels.begin(), it != host_map_voxels.end(); ++it){
        const auto voxel = *it;
        std::vector<Eigen::Vector3d> host_map_voxel_points = host_map.getVoxelPoints(voxel);
        std::vector<Eigen::Vector3d> device_map_voxel_points = device_map.getVoxelPoints(voxel);
        if(!pointsInVoxelsMatch(host_map_voxel_points)){
         has_same_points = false;
         break;
        }
      }
      EXPECT_TRUE(has_same_points);
    }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
