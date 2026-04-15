#include <pcl_lib/Convert.hpp>
#include <pcl_lib/Pipeline.hpp>
#include <pcl_lib/PointUtils.hpp>
#include <pcl_lib/bags/Ros2LidarImuBag.hpp>
#include <pcl_lib/ekf/Esekf.hpp>
#include <pcl_lib/ekf/Noise.hpp>
#include <pcl_lib/ekf/State.hpp>
#include <pcl_lib/visualization/ekf_visualizer.hpp>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

using namespace esekf;
using namespace bags;
using namespace cloud;

int main(int argc, char *argv[]) {
  if (argc <= 2) {
    std::cerr << "Usage: " << argv[0] << " <bag_path> <config_yaml_file_path> [options]\n";
    std::cerr << "Options:\n";
    std::cerr << "  -I: Use iterative ESEKF\n";
    std::cerr << "  -R: Enable realtime processing simulation\n";
    std::cerr << "  -N <noise_yaml>: Supply noise parameters from YAML\n";
    return 1;
  }

  std::string bag_path = argv[1];
  std::string config_path = argv[2];
  bool iterative_filter_argument = false;
  bool real_time_processing = false;
  std::string noise_config_path = "";

  for (int i = 3; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-I") {
      std::cerr << "Using Iterative ESEKF\n";
      iterative_filter_argument = true;
    } else if (arg == "-N") {
      if (i + 1 < argc) {
        noise_config_path = argv[++i];
        std::cerr << "Loading Noise Params from: " << noise_config_path << "\n";
      } else {
        std::cerr << "Error: -N requires a file path\n";
        return 1;
      }
    } else if (arg == "-R") {
      real_time_processing = true;
      std::cerr << "Realtime processing enabled\n";
    }
  }

  std::unique_ptr<ESEKFInterface> ekf;
  if (iterative_filter_argument) {
    ekf = std::make_unique<IterESEKF>();
  } else {
    ekf = std::make_unique<ESEKF>();
  }

  if (!noise_config_path.empty()) {
    // Assuming NoiseParams has a loadFromYaml method or equivalent
    // The ESEKF classes have setNoiseParams
    if (auto e = dynamic_cast<ESEKF *>(ekf.get())) {
      e->setNoiseParams(NoiseParams::loadFromYaml(noise_config_path));
    } else if (auto ie = dynamic_cast<IterESEKF *>(ekf.get())) {
      ie->setNoiseParams(NoiseParams::loadFromYaml(noise_config_path));
    }
  }

  Pipeline lidar_odom_pipeline;
  Ros2LidarImuBag lidar_imu_bag(bag_path, config_path, "sqlite3");
  lioEKFviewer visualizer;

  auto lidar_msg_callback = [&](sensor_msgs::msg::PointCloud2::SharedPtr bag_msg) {
    std::vector<Eigen::Vector3d> points = cloud::convertMsgToCloud(bag_msg);

    // Check for transform from lidar frame to imu frame if needed
    Sophus::SE3d transform;
    if (lidar_imu_bag.has_transform(bag_msg->header.frame_id)) {
      transform = lidar_imu_bag.transform_lookup(bag_msg->header.frame_id);
    }

    std::vector<Eigen::Vector3d> transformed_points = cloud::transformPoints(points, transform);

    auto [lidar_odom, map_cloud] = lidar_odom_pipeline.odometryUpdate(transformed_points);

    esekf::Odom odom;
    // msg->header.stamp should be used correctly
    odom.time = rclcpp::Time(bag_msg->header.stamp).seconds();
    odom.position = lidar_odom.translation();
    odom.rotation = lidar_odom.so3();
    ekf->update(odom);

    // Visualization
    Eigen::Vector3d position = ekf->getPosition();
    Eigen::Quaterniond rotation = ekf->getRotation().unit_quaternion();
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = rotation.toRotationMatrix().cast<float>();
    pose.block<3, 1>(0, 3) = position.cast<float>();

    visualizer.odometry_update(pose);
    // Assuming lidar_odom_pipeline has get_map_points
    visualizer.draw_map_points(lidar_odom_pipeline.getMap());
  };

  auto imu_msg_callback = [&](sensor_msgs::msg::Imu::SharedPtr bag_msg) {
    ImuReading reading = ImuReading::fromRosMsg(bag_msg);
    ekf->propogate(reading);

    // Visualization
    Eigen::Vector3d position = ekf->getPosition();
    Eigen::Quaterniond rotation = ekf->getRotation().unit_quaternion();
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = rotation.toRotationMatrix().cast<float>();
    pose.block<3, 1>(0, 3) = position.cast<float>();
    visualizer.prediction_update(pose);
  };

  lidar_imu_bag.set_imu_callback(imu_msg_callback);
  lidar_imu_bag.set_lidar_callback(lidar_msg_callback);

  while (lidar_imu_bag.has_next()) {
    lidar_imu_bag.process_next();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}