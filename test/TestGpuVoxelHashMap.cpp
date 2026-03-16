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
    Eigen::Vector3d vec0{0.5,0.5,0.5}
    Eigen::Vector3d vec1{0.65,0.65,0.65}
    std::vector<Eigen::Vector3d> vectors;
    GPUSparseVoxelMap  map;
    
    SimpleMultiPointSingleVoxelInsertion(){}
    ~SimpleMultiPointSingleVoxelInsertion override(){}

    void setup() override{
      map = GPUSparseVoxelMap();
    }

    void TearDown() override{
      map.~GPUSparseVoxelMap();
    }
}

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

auto has_num_points = [](std::vector<Eigen::Vector3d>  &points, int n){
    return  points.size() == n;
};


TEST_F(SimpleMultiPointSingleVoxelInsertion,VoxelMapEmptyTest){
  if (!map.empty){
    FAIL()
  }
  std::vector<Eigen::Vector3d> single_point_vec = 
  single_point_vec.emplace_back(vec0);
  map.add_voxels_to_map(single_point_vec);
  ASSERT_FASLE(map.empty());
}

TEST_F(SimpleMultiPointSingleVoxelInsertion,VoxelMapClearTest){
  if (!map.empty){
    FAIL()
  }
  std::vector<Eigen::Vector3d> single_point_vec;
  point_vec.emplace_back(vec0);
  map.add_voxels_to_map(single_point_vec);
  ASSERT_FALSE(map.empty());
  map.clear();
  ASSERT_TRUE(map.empty());
}

TEST_F(SimpleMultiPointSingleVoxelInsertion, VoxelAddResolutionSpacingDifferentUpdate){
  if (!map.empty){
    FAIL()
  }
  Eigen::Vector3d vec1{0.5, 0.5, 0.5};
  Eigen::Vector3d vec2{vec1};
  Eigen::Vector3i vox1 = cloud::PointToVoxel(vec1);
  std::vector<Eigen::Vector3d> points_first;
  points_first.push_back(vec1);
  std::vector<Eigen::Vector3d> points_second;
  points_second.push_back(vec2);
  map.add_voxels_to_map(points_first);
  map.add_voxels_to_map(points_second);
  std::vector<Eigen::Vector3d> points_in = map.findVoxel(vox1);
  ASSERT_TRUE(has_num_points(points_in, 1)):
}

TEST_F(SimpleMultiPointSingleVoxelInsertion, VoxelpointAddResolutionSpacingSameTest){
  if (!map.empty){
    FAIL()
  }
  Eigen::Vector3d vec1{0.5, 0.5, 0.5};
  Eigen::Vector3d vec2{vec1};
  Eigen::Vector3i vox1 = cloud::PointToVoxel(vec1);
  std::vector<Eigen::Vector3d> points_first;
  points_first.push_back(vec1);
  points_first.push_back(vec2);
  map.add_voxels_to_map(points_first);
  std::vector<Eigen::Vector3d> points_in = map.findVoxel(vox1);
  ASSERT_TRUE(has_num_points(points_in, 1))
}

TEST_F(SimpleMultiPointSingleVoxelInsertion, VoxelPointRemoveFarVoxel){
  if (!map.empty){
    FAIL()
  }
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d point_out{100.5, 0.0, 0.0};
  Eigen::Vector3d point_in{1.5, 0.0, 0.0};
  Eigen::Vector3i vox_out = cloud::PointToVoxel(point);
  Eigen::Vector3i vox_in = cloud::PointToVoxel(point);
  std::vector<Eigen::Vector3d> points;
  points.push_back(vox_out);
  points.push_back(vox_in);
  map.addPoints(points);
  map.removeFarPoints(position);
  non removed point
  std::vector<Eigen::Vector3d> in_voxel_points = map.getVoxel(vox_in);
  ASSERT_TRUE(has_num_points(in_voxel_points,1));
  ASSERT_EQ(in_voxel_points[0], point_in));

  // removed points from 
  std::vector<Eigen::Vector3d> out_voxel_points = map.getVoxel(vox_out);
  ASSERT_TRUE(has_num_points(out_voxel_points, 0));
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
