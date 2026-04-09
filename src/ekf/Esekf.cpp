#include "esekf.hpp"
#include <cmath>

namespace esekf {

ESEKF::ESEKF() {}

void ESEKF::initializeGravity() {
  Vector3d g_avg = Vector3d::Zero();
  for (const auto &a : _init_accels) {
    g_avg += a;
  }
  g_avg /= static_cast<double>(_init_accels.size());

  Vector3d g_measured = -g_avg.normalized() * 9.81;
  Vector3d g_canonical(0.0, 0.0, -9.81);

  Vector3d a_hat = g_canonical.normalized();
  Vector3d b_hat = g_measured.normalized();
  Vector3d v = a_hat.cross(b_hat);
  double s = v.norm();
  double c = a_hat.dot(b_hat);

  if (s < 1e-6) {
    if (c > 0) {
      _state.rotation_correction = Sophus::SO3d();
    } else {
      Vector3d perp = (std::abs(a_hat.x()) < 0.9) ? Vector3d::UnitX() : Vector3d::UnitY();
      Vector3d axis = a_hat.cross(perp).normalized();
      _state.rotation_correction = Sophus::SO3d::exp(axis * M_PI);
    }
  } else {
    Matrix3d vx = Sophus::SO3d::hat(v);
    Matrix3d R = Matrix3d::Identity() + vx + vx * vx * ((1.0 - c) / (s * s));
    _state.rotation_correction = Sophus::SO3d(Eigen::Quaterniond(R).normalized());
  }

  _gravity_initialized = true;
  _init_accels.clear();
}

void ESEKF::propogate(ImuReading update) {
  if (!_gravity_initialized) {
    _init_accels.push_back(update.acceleration);
    if (static_cast<int>(_init_accels.size()) >= _init_samples_required) {
      initializeGravity();
    }
    _state.last_time = update.time;
    return;
  }

  if (_state.last_time < 0.0) {
    _state.last_time = update.time;
    return;
  }

  double dt = update.time - _state.last_time;
  _state.last_time = update.time;

  Vector3d a = update.acceleration - _state.accelerometer_bias;
  Vector3d wv = update.angular_velocity - _state.gyro_bias;

  Vector3d g_odom = _state.rotation_correction * _gravity;

  Sophus::SO3d R = _state.rotation;
  _state.position =
      _state.position + R * (_state.velocity * dt) + ((R * a) + g_odom) * (dt * dt * 0.5);
  _state.velocity = _state.velocity + ((R * a) + g_odom) * dt;
  _state.rotation = R * Sophus::SO3d::exp(wv * dt);

  Matrix18d F = Matrix18d::Zero();
  F.block<3, 3>(0, 0) = -Sophus::SO3d::hat(wv).matrix();             // ∂θ̇/∂δθ
  F.block<3, 3>(0, 9) = -Matrix3d::Identity();                       // ∂θ̇/∂δbg
  F.block<3, 3>(3, 0) = -R.matrix() * Sophus::SO3d::hat(a).matrix(); // ∂v̇/∂δθ
  F.block<3, 3>(3, 12) = -R.matrix();                                // ∂v̇/∂δba
  F.block<3, 3>(3, 15) =
      -_state.rotation_correction.matrix() * Sophus::SO3d::hat(_gravity).matrix(); // ∂v̇/∂δθ_g
  F.block<3, 3>(6, 3) = Matrix3d::Identity();                                      // ∂ṗ/∂δv

  Matrix18d Qk = Matrix18d::Zero();
  Qk.block<3, 3>(0, 0) = _noise.gyro_noise * Matrix3d::Identity();
  Qk.block<3, 3>(3, 3) = _noise.accelerometer_noise * Matrix3d::Identity();
  Qk.block<3, 3>(9, 9) = _noise.gyro_bias_noise * Matrix3d::Identity();
  Qk.block<3, 3>(12, 12) = _noise.accelerometer_bias_noise * Matrix3d::Identity();
  Qk.block<3, 3>(15, 15) = _noise.gravity_correction_noise * Matrix3d::Identity();

  Matrix18d F_d = Matrix18d::Identity() + F * dt;
  Matrix18d Q_d = Qk * dt;

  _state.P = F_d * _state.P * F_d.transpose() + Q_d;
}

void ESEKF::update(Odom measure) {

  Vector3d P_diff = measure.position - _state.position;
  Vector3d R_diff = (measure.rotation * _state.rotation.inverse()).log();

  Eigen::Matrix<double, 6, 1> dz;
  dz << R_diff, P_diff;

  Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
  H.block<3, 3>(0, 0) = Matrix3d::Identity();
  H.block<3, 3>(3, 6) = Matrix3d::Identity();

  Matrix6d R_cov = Matrix6d::Zero();
  R_cov.block<3, 3>(0, 0) = _noise.lidar_rot_cov * Matrix3d::Identity();
  R_cov.block<3, 3>(3, 3) = _noise.lidar_trans_cov * Matrix3d::Identity();

  Eigen::Matrix<double, 18, 6> K =
      _state.P * H.transpose() * (H * _state.P * H.transpose() + R_cov).inverse();

  Eigen::Matrix<double, 18, 1> dx = K * dz;

  Vector3d dtheta = dx.segment<3>(0);
  Vector3d dv = dx.segment<3>(3);
  Vector3d dp = dx.segment<3>(6);
  Vector3d dbg = dx.segment<3>(9);
  Vector3d dba = dx.segment<3>(12);
  Vector3d dtheta_g = dx.segment<3>(15);

  _state.rotation = Sophus::SO3d::exp(dtheta) * _state.rotation;
  _state.velocity += dv;
  _state.position += dp;
  _state.gyro_bias += dbg;
  _state.accelerometer_bias += dba;
  _state.rotation_correction = Sophus::SO3d::exp(dtheta_g) * _state.rotation_correction;

  Matrix18d I_KH = Matrix18d::Identity() - K * H;
  _state.P = I_KH * _state.P * I_KH.transpose() + K * R_cov * K.transpose();
}

Vector3d ESEKF::getPosition() { return _state.position; }
Vector3d ESEKF::getVelocity() { return _state.velocity; }
Sophus::SO3d ESEKF::getRotation() { return _state.rotation; }
Sophus::SO3d ESEKF::getRotationCorrection() { return _state.rotation_correction; }

} // end namespace esekf
