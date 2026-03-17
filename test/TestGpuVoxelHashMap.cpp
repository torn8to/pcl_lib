#include "cuda/GPUVoxelHashMap.cuh"
#include "VoxelMap.hpp"
#include "PointToVoxel.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <algorithm>

#include <vector>
#include <tuple>

class SimpleMultiPointSingleVoxelInsertion: public testing::Test{
  protected:
    Eigen::Vector3d vec0{0.5,0.5,0.5};
    Eigen::Vector3d vec1{0.65,0.65,0.65};
    std::vector<Eigen::Vector3d> vectors;
    double max_range = 100;
    double voxel_resolution = 1.0;
    int points_per_voxel= 27;
    GPUSparseVoxelMapDefault map;
    
    SimpleMultiPointSingleVoxelInsertion() {}
    ~SimpleMultiPointSingleVoxelInsertion() override {}

    void SetUp() override {
      map.clear();
    }
};

/*
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
*/

auto has_num_points = [](const std::vector<Eigen::Vector3d> &points, int n) {
  if (static_cast<int>(points.size()) != n) {
    std::cout << "Expected " << n << " points but got "
              << points.size() << " points\n";
    return false;
  }
  return true;
};


TEST_F(SimpleMultiPointSingleVoxelInsertion,VoxelMapEmptyTest){
  if (!map.empty()){
    FAIL();
  }
  std::vector<Eigen::Vector3d> single_point_vec;
  single_point_vec.emplace_back(vec0);
  map.addPoints(single_point_vec);
  ASSERT_FALSE(map.empty());
}

TEST_F(SimpleMultiPointSingleVoxelInsertion,VoxelMapClearTest){
  if (!map.empty()){
    FAIL();
  }
  std::vector<Eigen::Vector3d> single_point_vec;
  single_point_vec.emplace_back(vec0);
  map.addPoints(single_point_vec);
  ASSERT_FALSE(map.empty());
  map.clear();
  ASSERT_TRUE(map.empty());
}

TEST_F(SimpleMultiPointSingleVoxelInsertion, VoxelAddResolutionSpacingDifferentUpdate){
  if (!map.empty()){
    FAIL();
  }
  Eigen::Vector3d v1{0.5, 0.5, 0.5};
  Eigen::Vector3d v2{v1};
  const double voxel_resolution = 1.0;
  Eigen::Vector3i vox1 = cloud::PointToVoxel(v1, voxel_resolution);
  std::vector<Eigen::Vector3d> points_first;
  points_first.push_back(v1);
  std::vector<Eigen::Vector3d> points_second;
  points_second.push_back(v2);
  map.addPoints(points_first);
  std::vector<Eigen::Vector3d> post_first = map.getVoxelPoints(vox1);
  ASSERT_EQ(post_first.size(), 1);
  ASSERT_EQ(points_first[0], points_in[0]);
  map.addPoints(points_second);
  std::vector<Eigen::Vector3d> points_in = map.getVoxelPoints(vox1);
  ASSERT_EQ(points_in.size(), 2);
  ASSERT_EQ(points_first[0], points_in[0]);
  ASSERT_EQ(points_first[1], points_in[1]);
}

TEST_F(SimpleMultiPointSingleVoxelInsertion, VoxelpointAddResolutionSpacingSameTest){
  if (!map.empty()){
    FAIL();
  }
  Eigen::Vector3d v1{0.5, 0.5, 0.5};
  Eigen::Vector3d v2{v1};
  const double voxel_resolution = 1.0;
  Eigen::Vector3i vox1 = cloud::PointToVoxel(v1, voxel_resolution);
  std::vector<Eigen::Vector3d> points_first;
  points_first.push_back(v1);
  points_first.push_back(v2);
  map.addPoints(points_first);
  std::vector<Eigen::Vector3d> points_in = map.getVoxelPoints(vox1);
  ASSERT_EQ(points_in.size(), 2);
  ASSERT_EQ(points_first[0], points_in[0]);
  ASSERT_EQ(points_first[1], points_in[1]);
}

/*
TEST_F(SimpleMultiPointSingleVoxelInsertion, VoxelPointRemoveFarVoxel){
  if (!map.empty()){
    FAIL();
  }
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d point_out{100.5, 0.0, 0.0};
  Eigen::Vector3d point_in{1.5, 0.0, 0.0};
  const double voxel_resolution = 1.0;
  Eigen::Vector3i vox_out = cloud::PointToVoxel(point_out, voxel_resolution);
  Eigen::Vector3i vox_in = cloud::PointToVoxel(point_in, voxel_resolution);
  std::vector<Eigen::Vector3d> points;
  points.push_back(point_out);
  points.push_back(point_in);
  map.addPoints(points);
  map.removePointsFarFromLocation(position);
  std::vector<Eigen::Vector3d> in_voxel_points = map.getVoxelPoints(vox_in);
  ASSERT_EQ(in_voxel_points.size(),1);
  ASSERT_EQ(in_voxel_points[0], point_in);

  std::vector<Eigen::Vector3d> out_voxel_points = map.getVoxelPoints(vox_out);
  ASSERT_EQ(out_voxel_points.size(), 0);
}
  */


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
