#pragma once
#include <glk/lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/trajectory.hpp>
// #include <glk/voxelmap.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <guik/viewer/async_light_viewer.hpp>
#include <thread>

#include <cmath>
#include <string>
#include <vector>

class lioEKFviewer {
public:
  lioEKFviewer();
  lioEKFviewer(lioEKFviewer &&) = default;
  lioEKFviewer(const lioEKFviewer &) = default;
  lioEKFviewer &operator=(lioEKFviewer &&) = default;
  lioEKFviewer &operator=(const lioEKFviewer &) = default;
  inline ~lioEKFviewer() {}

  void prediction_update(const Eigen::Matrix4f pose);
  void odometry_update(const Eigen::Matrix4f pose);
  void draw_map_points(const std::vector<Eigen::Vector3d> &points);
  void draw_map_points(const std::vector<Eigen::Vector3f> &points);
  void draw_voxels(std::vector<Eigen::Vector3i> &voxels);

  std::vector<float> getPropogationError();

private:
  guik::AsyncLightViewer *viewer_;
  std::string plot_error_name;
  std::string plot_xz_name;
  std::string propogation_prefix;
  unsigned int propogation_count = 0;
  float imu_coord_scaling = 0.5;
  unsigned int last_propogation_count;
  unsigned int propogation_filter = 16;

  Eigen::Matrix4f last_propogation_mat;
  Eigen::Matrix4f map_transform;

  std::vector<float> propogation_error;
  std::vector<float> x_pos;
  std::vector<float> y_pos;
};

inline lioEKFviewer::lioEKFviewer() {
  plot_error_name = "sekf error";
  plot_xz_name = "xz_plot";
  propogation_prefix = "prop";
  viewer_ = guik::async_viewer();
  viewer_->setup_plot(plot_error_name, 300, 200);
  viewer_->setup_plot(plot_xz_name, 400, 400);
}

inline void lioEKFviewer::prediction_update(const Eigen::Matrix4f pose) {
  unsigned int prop_mod = ++propogation_count % propogation_filter;
  if (prop_mod > last_propogation_count) {
    auto imu_shader = guik::VertexColor(pose);
    imu_shader.scale(imu_coord_scaling);
    std::string name = propogation_prefix + std::to_string(prop_mod);
    viewer_->update_coord(name, imu_shader);

    if (prop_mod - 50 > 0) {
      std::string name_removed = propogation_prefix + std::to_string(prop_mod - 50);
      // viewer_->remove_drawable(name_removed);
    }
  }
  last_propogation_mat = pose;
}

inline void lioEKFviewer::odometry_update(const Eigen::Matrix4f pose) {
  auto coord_shader_setting = guik::VertexColor(pose);
  coord_shader_setting.scale(1.0);
  Eigen::Vector3d update_diff =
      (pose.block<3, 1>(0, 3) - last_propogation_mat.block<3, 1>(0, 3)).cast<double>();
  float absolute_error = std::abs(update_diff.norm());
  viewer_->update_coord("odom", coord_shader_setting);
  propogation_error.push_back(absolute_error);
  // viewer_->update_plot(plot_error_name, "error (m)", propogation_error);
}

inline void lioEKFviewer::draw_map_points(const std::vector<Eigen::Vector3d> &points) {
  std::vector<Eigen::Vector3f> pointsFloat;
  for (int i = 0; i < points.size(); ++i) {
    Eigen::Vector3f vec = points[i].cast<float>();
    pointsFloat.push_back(vec);
  }
  draw_map_points(pointsFloat);
}

inline void lioEKFviewer::draw_map_points(const std::vector<Eigen::Vector3f> &points) {
  auto point_shader = guik::Rainbow();
  point_shader.transform(map_transform);
  viewer_->update_points("map", points, point_shader);
}

inline void lioEKFviewer::draw_voxels(std::vector<Eigen::Vector3i> &voxels) {}

inline std::vector<float> lioEKFviewer::getPropogationError() { return propogation_error; }
