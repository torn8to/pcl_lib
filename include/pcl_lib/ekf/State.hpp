#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace esekf {

struct State {
  double last_time = -1.0;
  Sophus::SO3d rotation = Sophus::SO3d();
  Sophus::SO3d rotation_correction = Sophus::SO3d();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 18, 18> P = Eigen::Matrix<double, 18, 18>::Zero();
};

struct ImuReading {
  double time;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d angular_velocity;

  static ImuReading fromRosMsg(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ImuReading reading;
    rclcpp::Time time_obj(msg->header.stamp);
    reading.time = time_obj.seconds();
    Eigen::Vector3d acceleration{msg->linear_acceleration.x, msg->linear_acceleration.y,
                                 msg->linear_acceleration.z};

    Eigen::Vector3d angular_velocity{msg->angular_velocity.x, msg->angular_velocity.y,
                                     msg->angular_velocity.z};

    reading.acceleration = acceleration;
    reading.angular_velocity = angular_velocity;
    return reading;
  }
};

struct Odom {
  double time;
  Eigen::Vector3d position;
  Sophus::SO3d rotation;

  static Odom fromRosMsg(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Odom data;
    rclcpp::Time time_obj(msg->header.stamp);
    data.time = time_obj.seconds();

    Eigen::Vector3d position{msg->pose.pose.position.x, msg->pose.pose.position.y,
                             msg->pose.pose.position.z};

    Eigen::Quaterniond quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    data.position = position;
    data.rotation = Sophus::SO3d{quat};
    return data;
  }
};

} // end namespace esekf
