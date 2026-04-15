#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <vector>

#include "Noise.hpp"
#include "State.hpp"

namespace esekf {

class ESEKFInterface {
public:
  virtual void propogate(ImuReading reading) = 0;
  virtual void update(Odom update) = 0;
  virtual Eigen::Vector3d getPosition() = 0;
  virtual Eigen::Vector3d getVelocity() = 0;
  virtual Sophus::SO3d getRotation() = 0;
  virtual Sophus::SO3d getRotationCorrection() = 0;
  virtual ~ESEKFInterface() = default;
};

class ESEKF : public ESEKFInterface {
public:
  ESEKF();
  void propogate(ImuReading reading) override;
  void update(Odom update) override;
  Eigen::Vector3d getPosition() override;
  Eigen::Vector3d getVelocity() override;
  Sophus::SO3d getRotation() override;
  Sophus::SO3d getRotationCorrection() override;
  inline NoiseParams getNoise() { return _noise; }
  inline void setNoiseParams(NoiseParams noise_params) { _noise = noise_params; }

private:
  void initializeGravity();

  State _state;
  NoiseParams _noise;
  Eigen::Vector3d _gravity{0.0, 0.0, -9.81};

  std::vector<Eigen::Vector3d> _init_accels;
  bool _gravity_initialized = false;
  int _init_samples_required = 20;
};

class IterESEKF : public ESEKFInterface {
public:
  IterESEKF();
  void propogate(ImuReading reading) override;
  void update(Odom update) override;
  Eigen::Vector3d getPosition() override;
  Eigen::Vector3d getVelocity() override;
  Sophus::SO3d getRotation() override;
  Sophus::SO3d getRotationCorrection() override;
  inline NoiseParams getNoise() { return _noise; }
  inline void setNoiseParams(NoiseParams noise_params) { _noise = noise_params; }

private:
  void initializeGravity();
  void repropagateImu();

  State _state;
  State _reserve_state;
  NoiseParams _noise;
  Eigen::Vector3d _gravity{0.0, 0.0, -9.81};

  std::vector<ImuReading> _imu_buffer;
  std::vector<Eigen::Vector3d> _init_accels;
  bool _gravity_initialized = false;
  int _init_samples_required = 20;

  int _max_iterations = 5;
  double _convergence_threshold = 1e-4;
};

} // namespace esekf
