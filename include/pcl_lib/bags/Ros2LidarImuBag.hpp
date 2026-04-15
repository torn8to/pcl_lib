#pragma once
#include "yaml-cpp/yaml.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "../Convert.hpp"
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <set>
#include <sophus/se3.hpp>
#include <unordered_map>
#include <vector>

namespace bags {

const std::set<std::string> valid_storage_ids = {"sqlite3", "mcap"};

struct Ros2LidarImuBagConfig {};

class Ros2LidarImuBag {
  using PointCloud2Callback = std::function<void(sensor_msgs::msg::PointCloud2::SharedPtr)>;
  using ImuCallback = std::function<void(sensor_msgs::msg::Imu::SharedPtr)>;

public:
  using TFlookupMap = std::unordered_map<std::string, Sophus::SE3d>;
  Ros2LidarImuBag(const std::string bag_path, const std::string config_path,
                  const std::string storage_id);
  rosbag2_storage::SerializedBagMessage get_next();
  bool has_next();
  bool has_transform(const std::string frame);
  Sophus::SE3d transform_lookup(const std::string frame);
  void process_next();
  void set_lidar_callback(PointCloud2Callback callback);
  void set_imu_callback(ImuCallback callback);

private:
  void process_yaml();
  void setup_bag();
  std::vector<std::string> _topics;
  std::string _storage_id;
  std::string _bag_path;
  std::string _config_path;
  YAML::Node _config;
  rosbag2_cpp::readers::SequentialReader _reader;
  std::chrono::time_point<std::chrono::high_resolution_clock> _current_time;
  TFlookupMap _transform_lookup;

  std::unordered_map<std::string, std::function<void(const rosbag2_storage::SerializedBagMessage &)>>
      callbacks_;
  PointCloud2Callback _lidar_callback;
  ImuCallback _imu_callback;
};

inline Ros2LidarImuBag::Ros2LidarImuBag(const std::string bag_path, const std::string config,
                                        const std::string storage_id) {
  assert(valid_storage_ids.count(storage_id) > 0);
  _storage_id = storage_id;
  _config_path = config;
  _bag_path = bag_path;
  process_yaml();
  setup_bag();

  std::cout << "--- Filtered Bag Information ---\n";
  auto metadata = _reader.get_metadata();
  size_t total_filtered_messages = 0;
  for (const auto &topic_info : metadata.topics_with_message_count) {
    if (std::find(_topics.begin(), _topics.end(), topic_info.topic_metadata.name) !=
        _topics.end()) {
      std::cout << "Topic: " << topic_info.topic_metadata.name
                << " | Type: " << topic_info.topic_metadata.type
                << " | Messages: " << topic_info.message_count << "\n";
      total_filtered_messages += topic_info.message_count;
    }
  }

  std::cout << "Total messages in filtered bag: " << total_filtered_messages << "\n";
  std::cout << "--------------------------------\n";
}

inline void Ros2LidarImuBag::process_yaml() {
  _config = YAML::LoadFile(_config_path);
  assert(_config["imu_topic"].IsSequence());
  assert(_config["lidar_topics"].IsSequence());
  assert(_config["transforms"].IsSequence());

  YAML::Node imu_topic = _config["imu_topic"];
  YAML::Node lidar_topic_seq = _config["lidar_topics"];
  YAML::Node transform_seq = _config["transforms"];
  int num_lidar_topics = lidar_topic_seq.size();
  int num_transforms = transform_seq.size();

  // callback messages
  auto lidar_msg_callback = [&](const rosbag2_storage::SerializedBagMessage &msg) {
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg =
        std::make_shared<sensor_msgs::msg::PointCloud2>();
    rclcpp::SerializedMessage extracted_serialized_message(*msg.serialized_data);
    serializer.deserialize_message(&extracted_serialized_message, cloud_msg.get());
    _lidar_callback(cloud_msg);
  };

  auto imu_msg_callback = [&](const rosbag2_storage::SerializedBagMessage &msg) {
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    sensor_msgs::msg::Imu::SharedPtr imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    rclcpp::SerializedMessage extracted_serialized_message(*msg.serialized_data);
    serializer.deserialize_message(&extracted_serialized_message, imu_msg.get());
    _imu_callback(imu_msg);
  };

  for (int i = 0; i < num_lidar_topics; ++i) {
    std::string lidar_topic_string = lidar_topic_seq[i].as<std::string>();
    _topics.push_back(lidar_topic_string);
    callbacks_.insert({lidar_topic_string, lidar_msg_callback});
  }

  for (int i = 0; i < num_transforms; ++i) {
    YAML::Node transform_entry = transform_seq[i];
    YAML::Node transform_name = transform_entry["name"];
    YAML::Node transform_node = transform_entry["quaternion"];
    YAML::Node position = transform_entry["position"];
    Eigen::Quaterniond quat{transform_node["w"].as<double>(), transform_node["x"].as<double>(),
                            transform_node["y"].as<double>(), transform_node["z"].as<double>()};

    Eigen::Vector3d pos{position["x"].as<double>(), position["y"].as<double>(),
                        position["z"].as<double>()};

    Sophus::SE3d transform{quat, pos};
    _transform_lookup.insert({_topics[i], transform}); // transform insertion via topic
    _transform_lookup.insert(
        {transform_name.as<std::string>(), transform}); // transform via tf name
    callbacks_.insert({transform_name.as<std::string>(), lidar_msg_callback});
  }
  std::string imu_topic_string = imu_topic.as<std::string>();
  _topics.push_back(imu_topic_string); // add imu topic for list
  callbacks_.insert({imu_topic_string, imu_msg_callback});
}

inline void Ros2LidarImuBag::setup_bag() {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = _bag_path;
  storage_options.storage_id = _storage_id;
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = _topics;
  _reader.open(storage_options, converter_options);
  _reader.set_filter(storage_filter);
  auto metadata = _reader.get_metadata();
}

inline bool Ros2LidarImuBag::has_next() { return _reader.has_next(); }

inline rosbag2_storage::SerializedBagMessage Ros2LidarImuBag::get_next() {
  auto msg = _reader.read_next();
  return *msg;
}

inline void Ros2LidarImuBag::process_next() {
  auto msg = _reader.read_next();
  std::string topic_name = msg->topic_name;
  // dependency injection to hide the rotten mess
  if (callbacks_.find(topic_name) != callbacks_.end()) {
    callbacks_[topic_name](*msg);
  }
}

inline void Ros2LidarImuBag::set_lidar_callback(PointCloud2Callback callback) {
  _lidar_callback = callback;
}

inline void Ros2LidarImuBag::set_imu_callback(ImuCallback callback) { _imu_callback = callback; }

inline bool Ros2LidarImuBag::has_transform(const std::string frame) {
  return _transform_lookup.find(frame) != _transform_lookup.end();
}

inline Sophus::SE3d Ros2LidarImuBag::transform_lookup(const std::string frame) {
  return _transform_lookup.find(frame)->second;
}

} // namespace bags
