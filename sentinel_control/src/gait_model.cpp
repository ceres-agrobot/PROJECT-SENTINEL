#include "sentinel_control/gait_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <string>
#include <stdexcept>

namespace sentinel::control
{
namespace
{
constexpr double kMinEpsilon = 1e-4;
constexpr std::array<double, kJointCount> kMinJointPositions = {
  -1.0472, -1.5708, -2.7227, -1.0472, -1.5708, -2.7227,
  -1.0472, -1.5708, -2.7227, -1.0472, -1.5708, -2.7227
};
constexpr std::array<double, kJointCount> kMaxJointPositions = {
  1.0472, 3.4907, -0.83776, 1.0472, 3.4907, -0.83776,
  1.0472, 3.4907, -0.83776, 1.0472, 3.4907, -0.83776
};

void requirePositive(const double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}
}  // namespace

void LocomotionParameters::validate() const
{
  requirePositive(control_rate_hz, "control_rate_hz");
  requirePositive(state_publish_rate_hz, "state_publish_rate_hz");
  requirePositive(stand_height_m, "stand_height_m");
  requirePositive(crouch_height_m, "crouch_height_m");
  requirePositive(thigh_length_m, "thigh_length_m");
  requirePositive(calf_length_m, "calf_length_m");
  requirePositive(gait_frequency_hz, "gait_frequency_hz");
  requirePositive(max_linear_x, "max_linear_x");
  requirePositive(max_linear_y, "max_linear_y");
  requirePositive(max_angular_z, "max_angular_z");
  requirePositive(command_timeout_s, "command_timeout_s");
  requirePositive(max_joint_rate_rad_s, "max_joint_rate_rad_s");
  for (const double gain : {step_height_m, step_length_m, lateral_step_gain, yaw_stride_gain}) {
    if (!std::isfinite(gain) || gain < 0.0) {
      throw std::invalid_argument("Gait distances and gains must be nonnegative");
    }
  }
  if (control_rate_hz > 1000.0 || state_publish_rate_hz > control_rate_hz ||
      stand_height_m >= thigh_length_m + calf_length_m ||
      crouch_height_m >= thigh_length_m + calf_length_m ||
      stand_height_m <= std::abs(thigh_length_m - calf_length_m) ||
      crouch_height_m <= std::abs(thigh_length_m - calf_length_m)) {
    throw std::invalid_argument("Invalid control rates or unreachable standing height");
  }
}

const std::array<const char *, kJointCount> & GaitModel::jointNames() noexcept
{
  static const std::array<const char *, kJointCount> names = {
    "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
    "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
    "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
    "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"
  };
  return names;
}

GaitModel::GaitModel(LocomotionParameters parameters)
  : parameters_(parameters)
{
  parameters_.validate();
}

void GaitModel::setMode(const LocomotionMode mode) noexcept
{
  mode_ = mode;
  if (mode != LocomotionMode::Walk) {
    velocity_ = {};
    phase_ = 0.0;
  }
}

void GaitModel::setVelocity(const VelocityCommand command) noexcept
{
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
      !std::isfinite(command.wz)) {
    return;
  }
  velocity_.vx = std::clamp(command.vx, -parameters_.max_linear_x, parameters_.max_linear_x);
  velocity_.vy = std::clamp(command.vy, -parameters_.max_linear_y, parameters_.max_linear_y);
  velocity_.wz = std::clamp(command.wz, -parameters_.max_angular_z, parameters_.max_angular_z);
  if (mode_ == LocomotionMode::Stand) {
    mode_ = LocomotionMode::Walk;
  }
}

void GaitModel::setMeasuredJoints(const JointArray & positions) noexcept
{
  if (initialized_) return;
  for (const double position : positions) {
    if (!std::isfinite(position)) return;
  }
  references_ = positions;
  initialized_ = true;
}

std::array<double, 2> GaitModel::sagittalIk(const double x, const double z) const noexcept
{
  const double thigh = parameters_.thigh_length_m;
  const double calf = parameters_.calf_length_m;
  const double distance = std::clamp(std::hypot(x, z),
    std::abs(thigh - calf) + kMinEpsilon, thigh + calf - kMinEpsilon);
  const double cosine = std::clamp(
    (distance * distance - thigh * thigh - calf * calf) / (2.0 * thigh * calf), -1.0, 1.0);
  const double knee = -std::acos(cosine);
  const double hip = std::atan2(-x, -z) -
    std::atan2(calf * std::sin(knee), thigh + calf * std::cos(knee));
  return {hip, knee};
}

void GaitModel::step(const double dt_s, const bool fresh_command)
{
  const double dt = std::clamp(dt_s, 0.0, 0.03);
  const auto & p = parameters_;
  const VelocityCommand cmd = (mode_ == LocomotionMode::Walk && fresh_command)
    ? velocity_ : VelocityCommand{};
  moving_ = std::max({std::abs(cmd.vx), std::abs(cmd.vy), std::abs(cmd.wz)}) > 1e-3;
  if (moving_) {
    phase_ = std::fmod(phase_ + dt * p.gait_frequency_hz, 1.0);
  } else {
    phase_ = 0.0;
  }

  const double height = mode_ == LocomotionMode::Crouch ? p.crouch_height_m : p.stand_height_m;
  JointArray target{};
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    const double side = (leg == 0 || leg == 2) ? -1.0 : 1.0;
    const double diagonal = (leg == 0 || leg == 3) ? 0.0 : 0.5;
    const double phase = std::fmod(phase_ + diagonal, 1.0);
    const double longitudinal = cmd.vx - side * cmd.wz * p.yaw_stride_gain;
    const double lateral = cmd.vy;
    const double speed_ratio = std::clamp(
      std::hypot(longitudinal, lateral) / p.max_linear_x, 0.0, 1.0);
    const double stride = p.step_length_m * speed_ratio;
    double x = 0.0;
    double lift = 0.0;
    double abduction = 0.0;
    if (moving_) {
      if (phase < 0.5) {
        x = stride * (0.5 - 2.0 * phase);
      } else {
        const double swing_phase = 2.0 * (phase - 0.5);
        x = stride * (swing_phase - 0.5);
        lift = p.step_height_m * std::sin(std::numbers::pi * swing_phase);
      }
      if (longitudinal < 0.0) x = -x;
      abduction = std::clamp(
        -side * lateral * p.lateral_step_gain / p.max_linear_y, -0.16, 0.16);
    }
    feet_[leg] = {x, -side * height * std::sin(abduction), -height + lift};
    const auto angles = sagittalIk(x, -height + lift);
    target[3 * leg] = abduction;
    target[3 * leg + 1] = angles[0];
    target[3 * leg + 2] = angles[1];
  }

  if (!initialized_) {
    references_ = target;
    initialized_ = true;
  }
  const double max_step = p.max_joint_rate_rad_s * dt;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double desired = std::clamp(target[i], kMinJointPositions[i], kMaxJointPositions[i]);
    references_[i] = std::clamp(
      references_[i] + std::clamp(desired - references_[i], -max_step, max_step),
      kMinJointPositions[i], kMaxJointPositions[i]);
  }
}

}  // namespace sentinel::control
