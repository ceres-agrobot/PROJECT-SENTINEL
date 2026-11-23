#pragma once

#include <array>
#include <chrono>
#include <cstddef>

namespace sentinel::control {

constexpr std::size_t kLegCount = 4;
constexpr std::size_t kJointCount = 12;
using JointArray = std::array<double, kJointCount>;
using FootArray = std::array<std::array<double, 3>, kLegCount>;

struct VelocityCommand {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

enum class LocomotionMode { Stand, Walk, Crouch, Stop };

struct LocomotionParameters {
  double control_rate_hz{100.0};
  double state_publish_rate_hz{10.0};
  double stand_height_m{0.34};
  double crouch_height_m{0.27};
  double thigh_length_m{0.213};
  double calf_length_m{0.213};
  double gait_frequency_hz{1.4};
  double step_height_m{0.045};
  double step_length_m{0.07};
  double lateral_step_gain{0.13};
  double yaw_stride_gain{0.12};
  double max_linear_x{0.22};
  double max_linear_y{0.12};
  double max_angular_z{0.40};
  double command_timeout_s{0.4};
  double max_joint_rate_rad_s{6.0};
  void validate() const;
};

class GaitModel final {
public:
  explicit GaitModel(LocomotionParameters parameters);
  void setMode(LocomotionMode mode) noexcept;
  void setVelocity(VelocityCommand command) noexcept;
  void setMeasuredJoints(const JointArray &positions) noexcept;
  void step(double dt_s, bool fresh_command);

  [[nodiscard]] const JointArray &references() const noexcept {
    return references_;
  }
  [[nodiscard]] const FootArray &footTargets() const noexcept { return feet_; }
  [[nodiscard]] LocomotionMode mode() const noexcept { return mode_; }
  [[nodiscard]] bool moving() const noexcept { return moving_; }
  [[nodiscard]] const LocomotionParameters &parameters() const noexcept {
    return parameters_;
  }
  [[nodiscard]] static const std::array<const char *, kJointCount> &
  jointNames() noexcept;

private:
  [[nodiscard]] std::array<double, 2> sagittalIk(double x,
                                                 double z) const noexcept;
  LocomotionParameters parameters_;
  VelocityCommand velocity_{};
  LocomotionMode mode_{LocomotionMode::Stand};
  JointArray references_{};
  FootArray feet_{};
  double phase_{0.0};
  bool initialized_{false};
  bool moving_{false};
};

} // namespace sentinel::control
