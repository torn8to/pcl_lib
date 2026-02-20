#pragma once
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <numeric>
#include <cmath>

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

bool pointsInVoxelsMatch(std::vector<Eigen::Vector3d> &source, std::vector<Eigen::Vector3d> &testable){
    if(source.size() == testable.size()) return false;
    std::for_each(source.begin(), source.end(), [&](const  Eigen::Vector3d &point){
        if(!std::any_of(testable.begin(),testable.end(), [&](const Eigen::Vector3d &t_point){
            return t_point == point;
        })) return false;
    });
    return true;
}