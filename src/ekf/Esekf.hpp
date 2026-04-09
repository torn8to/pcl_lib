#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Sophus/so3.hpp>
#include <vector>

#include "noise.hpp"
#include "state.hpp"

namespace esekf {

class ESEKFInterface {
public:
  virtual void propogate(ImuReading reading) = 0;
  virtual void update(Odom update) = 0;
  virtual Vector3d getPosition() = 0;
  virtual Vector3d getVelocity() = 0;
  virtual Sophus::SO3d getRotation() = 0;
  virtual Sophus::SO3d getRotationCorrection() = 0;
  virtual ~ESEKFInterface() = default;
};

class ESEKF : public ESEKFInterface {
public:
  ESEKF();
  void propogate(ImuReading reading) override;
  void update(Odom update) override;
  Vector3d getPosition() override;
  Vector3d getVelocity() override;
  Sophus::SO3d getRotation() override;
  Sophus::SO3d getRotationCorrection() override;

private:
  void initializeGravity();

  State _state;
  NoiseParams _noise;
  Vector3d _gravity{0.0, 0.0, -9.81};

  std::vector<Vector3d> _init_accels;
  bool _gravity_initialized = false;
  int _init_samples_required = 20;
};

class IterESEKF : public ESEKFInterface {
public:
  void propogate(ImuReading reading) override;
  void update(Odom update) override;
  Vector3d getPosition() override;
  Vector3d getVelocity() override;
  Sophus::SO3d getRotation() override;
  Sophus::SO3d getRotationCorrection() override;

private:
  State _state;
  State _reserve_state;
  NoiseParams _noise;
  Vector3d _gravity{0.0, 0.0, -9.81};
};

} // namespace esekf
