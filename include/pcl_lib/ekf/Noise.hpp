#pragma once

#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace esekf {

struct NoiseParams {
  double gyro_noise = 0.005;
  double gyro_bias_noise = 0.001;
  double accelerometer_noise = 0.05;
  double accelerometer_bias_noise = 0.001;
  double lidar_rot_cov = 0.02;
  double lidar_trans_cov = 0.02;
  double gravity_correction_noise = 0.001;

  static NoiseParams loadFromYaml(const std::string &yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);
    assert(config["gyro_noise"].Type() == YAML::NodeType::Scalar);
    assert(config["gyro_bias_noise"].Type() == YAML::NodeType::Scalar);
    assert(config["accelerometer_noise"].Type() == YAML::NodeType::Scalar);
    assert(config["accelerometer_bias_noise"].Type() == YAML::NodeType::Scalar);
    assert(config["lidar_rot_cov"].Type() == YAML::NodeType::Scalar);
    assert(config["lidar_trans_cov"].Type() == YAML::NodeType::Scalar);
    assert(config["gravity_correction_noise"].Type() == YAML::NodeType::Scalar);

    NoiseParams noise_params;
    noise_params.gyro_noise = config["gyro_noise"].as<double>();
    noise_params.gyro_bias_noise = config["gyro_bias_noise"].as<double>();
    noise_params.accelerometer_noise = config["accelerometer_noise"].as<double>();
    noise_params.accelerometer_bias_noise = config["accelerometer_bias_noise"].as<double>();
    noise_params.lidar_rot_cov = config["lidar_rot_cov"].as<double>();
    noise_params.lidar_trans_cov = config["lidar_trans_cov"].as<double>();
    noise_params.gravity_correction_noise = config["gravity_correction_noise"].as<double>();
    return noise_params;
  }

  static void loadToYaml(const std::string &yaml_file, const NoiseParams &noise_params) {
    std::ofstream ofs(yaml_file);
    YAML::Emitter out(ofs);
    out << YAML::BeginMap;
    out << YAML::Key << "gyro noise" << YAML::Value << noise_params.gyro_noise;
    out << YAML::Key << "gyro bias noise" << YAML::Value << noise_params.gyro_bias_noise;
    out << YAML::Key << "accelerometer noise" << YAML::Value << noise_params.accelerometer_noise;
    out << YAML::Key << "accelerometer bias noise" << YAML::Value
        << noise_params.accelerometer_bias_noise;
    out << YAML::Key << "lidar rot cov" << YAML::Value << noise_params.lidar_rot_cov;
    out << YAML::Key << "lidar trans cov" << YAML::Value << noise_params.lidar_trans_cov;
    out << YAML::Key << "gravity correction noise" << YAML::Value
        << noise_params.gravity_correction_noise;
    out << YAML::EndMap;
  }
};
} // namespace esekf
