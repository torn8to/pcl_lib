#include "cuda/MotionCompensation.cuh"
#include "MotionCompensation.hpp"
#include "TestUtils.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <vector>
#include <tuple>



class FuzzedMotionCompensationBehaviorMatchTestCPUGPU: public testing::Test {
  protected:
    FuzzedMotionCompensationBehaviorMatchTestCPUGPU(){}

    ~FuzzedMotionCompensationBehaviorMatchTestCPUGPU() override {}

    void SetUp() override{
      int num_points = 10000;
      test_data.reserve(num_tests);
      for(unsigned int i = 0; i < num_tests; ++i){
        std::vector<Eigen::Vector3d> random_test_points = generate_random_points(num_points);
        std::vector<double> timestamps = generate_timestamps(0.0, 5.0, num_points);
        Sophus::SE3d random_motion = generate_random_transform(5.0);

        test_data.emplace_back(std::make_tuple(random_test_points, timestamps, random_motion));
      }
    }

    void TearDown() override{
      
    }

    std::vector<std::tuple<std::vector<Eigen::Vector3d>, std::vector<double>, Sophus::SE3d>> test_data;
    unsigned int num_tests = 10;
};

TEST_F(FuzzedMotionCompensationBehaviorMatchTestCPUGPU, TestCPUGPUApproxBehaviorMatch){
      for(auto test_it =  test_data.begin(); test_it != test_data.end(); ++test_it){
        std::vector<Eigen::Vector3d> points = std::get<0>(*test_it);
        std::vector<double> timestamps = std::get<1>(*test_it);
        Sophus::SE3d transform = std::get<2>(*test_it);
        std::size_t s =  points.size();

        std::vector<Eigen::Vector3d> cpu_out_buffer;
        cpu_out_buffer.reserve(s);
        std::vector<Eigen::Vector3d> gpu_out_buffer;
        gpu_out_buffer.reserve(s);

        gpu_out_buffer = motionDeSkewGpu(points, timestamps, transform);
        cpu_out_buffer = cloud::motionDeSkew(points, timestamps, transform);

        EXPECT_TRUE(vectorPointsAreApproximate(cpu_out_buffer, gpu_out_buffer));
      }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}





