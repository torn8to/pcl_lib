#include "cuda/MotionCompensation.cuh"
#include "MotionCompensation.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <vector>
#include <tuple>
namespace {

Sophus::SE3d generate_random_transform(const double random_tranlational_factor = 5.0){
  Eigen::Quaterniond random_quaternion = Eigen::Quaterniond::unitRandom();
  Eigen::Vector3d random_translation = Eigen::Vector3d::Random() * factor;
  return Sophus::Se3d(random_quaternion, random_tranlslation);
}

// generate points in a sqaure
std::vector<Eigen::Vector3d>  generate_random_points(const unsigned int num_points = 1000,
                                                     const double max = 100.0,
                                                     const double min = 0.0){
  std::vector<Eigen::Vector3d> random_points;
  random_points.reserve(num_points);

  for( unsigned int i = 0; i < num_points; ++i){
    Eigen::Vector3d random_point = Eigen::Vector3d::Random * (max-min) + min;
    random_points.emplace_back(random_point);
  }
  return random_points;
}

bool vectorPointsAreApproximate(const std::vector<Eigen::Vector3d> points1, const std::vector<Eigen::Vector3d> points2){
  assert(points1.size() == points2.size(), "test points vectors are not the same size")
  auto it1 = points1.begin();
  auto it2 = points2.begin();
  for(; it1 != points1.end();){
    if(!*it1.isApprox(*it2, 1e-4)){
      return false;
    }

    ++it1;
    ++it2;
  }
  return true;
}

class FuzzedMotionCompensationBehaviorMatchTestCPUGPU: public testing::Test {
  protected:
    FuzzedMotionCompensationBehaviorMatchTest(){}

    ~FuzzedMotionCompensationBehaviorMatchTest() override {}

    void setUp() override{
      test_data.reserve(num_runs)
      for(unsigned int i = 0; i < num_tests; ++i){
        std::vector<Eigen::Vector3d> random_test_points = generate_random_points(10000);
        Sophus::SE3d random_motion = generate_random_transform(5.0);
        test_data.emplace_back(std::make_pair(random_test_points, random_motion));
      }
    }

    void TearDown() override{
      
    }

  std::vector<std::pair<std::vector<Eigen::Vector3d>,Sophus::SE3d>> test_data;
  unsigned int num_runs = 10;

  TEST_F(FuzzedMotionCompensationBehaviorMatchTestCPUGPU, TestCPUGPUApproxBehaviorMatch){
    

  }
}

} //anonymous namespace end

int main(int argc, char **argv){
  testing::InitGoogleTest(argc, argv);
  return RUN_ALL_TESTS();
}





