#include "cuda/MotionCompensation.cuh"
#include "MotionCompensation.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <vector>
#include <tuple>


Sophus::SE3d generate_random_transform(const double random_translational_factor = 5.0){
  Eigen::Quaterniond random_quaternion = Eigen::Quaterniond::UnitRandom();
  Eigen::Vector3d random_translation = Eigen::Vector3d::Random() * random_translational_factor;
  return Sophus::SE3d(random_quaternion, random_translation);
}

std::vector<double> generate_timestamps(const double time_start = 0.0,
                                        const double time_end = 2.0,
                                        const double number_of_points = 1000){
  assert(number_of_points > 0);

  std::vector<double> timestamps_vec;
  timestamps_vec.reserve(number_of_points);
    
  if (number_of_points == 1) {
    timestamps_vec.push_back(time_start);
    return timestamps_vec;
  }
  
  double step = (time_end - time_start) / (number_of_points - 1);
  
  for (size_t i = 0; i < number_of_points; ++i) {
    timestamps_vec.push_back(time_start + i * step);
  }

  return timestamps_vec;
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

bool vectorPointsAreApproximate(const std::vector<Eigen::Vector3d> points1, const std::vector<Eigen::Vector3d> points2){
  assert(points1.size() == points2.size());
  auto it1 = points1.begin();
  auto it2 = points2.begin();

  for(; it1 != points1.end();){
    if(!(*it1).isApprox((*it2), 1e-4)){
      return false;
    }
    ++it1;
    ++it2;
  }
  return true;
}

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





