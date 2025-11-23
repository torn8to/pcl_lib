#include "cuda/GPUVoxelHashMap.cuh"
#include "VoxelMap.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <algorithms>

#include <vector>
#include <tuple>

bool pointsInVoxelMatch(std::vector<Eigen::Vector3d> &source, std::vector<Eigen::Vector3d> &testable){
    if(source.size() == testable.size()){

    }
    std::for_each(source.begin(), source.end(), [&](const auto Eigen::Vector3d points){
        

    })
    return true;
    

  

}

// generate points in a sqaure
std::vector<Eigen::Vector3d>  generate_random_points(const unsigned int num_points = 1000,
                                                     const double max = 100.0,
                                                     const double min = 0.0){
  std::vector<Eigen::Vector3d> random_points;
  random_points.reserve(num_points);

  for( unsigned int i = 0; i < num_points; ++i){
    Eigen::Vector3d random_point = Eigen::Vector3d::Random() * (max-min);
    random_points.emplace_back(random_point);
  }
  return random_points;
}


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

TEST_F(FuzzedMotionCompensationBehaviorMatchTestCPUGPU, TestCPUGPUApproxBehaviorMatch){
    VoxelMap host_map;
    GpuVoxelHashMap gpu_map;

    for(auto test_it =  test_data.begin(); test_it != test_data.end(); ++test_it){
      std::vector<Eigen::Vector3d> points = *test_it;
    
      EXPECT_TRUE(vectorPointsAreApproximate(cpu_out_buffer, gpu_out_buffer));
    }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
