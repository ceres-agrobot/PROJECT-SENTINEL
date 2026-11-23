#include "sentinel_control/gait_model.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sentinel_interfaces/msg/joints.hpp>
#include <sentinel_interfaces/msg/point_array.hpp>
#include <sentinel_interfaces/msg/velocities.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace sentinel::control {

class LocomotionNode final : public rclcpp::Node {
public:
  LocomotionNode() : Node("sentinel_locomotion"), model_(loadParameters()) {
    const auto qos = rclcpp::QoS(10).reliable();
    pub_commands_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "joint_group_position_controller/commands", qos);
    pub_references_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_references", qos);
    pub_mode_ = create_publisher<std_msgs::msg::String>("mode_state", qos);
    pub_custom_joints_ = create_publisher<sentinel_interfaces::msg::Joints>(
        "joints_reference", qos);
    pub_feet_ = create_publisher<sentinel_interfaces::msg::PointArray>(
        "feet_reference", qos);

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          onVelocity(*msg);
        });
    sub_custom_velocity_ =
        create_subscription<sentinel_interfaces::msg::Velocities>(
            "velocity_command", qos,
            [this](sentinel_interfaces::msg::Velocities::ConstSharedPtr msg) {
              onVelocityComponents(msg->linear_x, msg->linear_y,
                                   msg->angular_z);
            });
    sub_mode_ = create_subscription<std_msgs::msg::String>(
        "mode", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          onMode(msg->data);
        });
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          onJointStates(*msg);
        });

    const auto &cfg = model_.parameters();
    using namespace std::chrono;
    last_tick_ = steady_clock::now();
    const auto control_period =
        duration_cast<nanoseconds>(duration<double>(1.0 / cfg.control_rate_hz));
    timer_ = create_wall_timer(control_period, [this] { update(); });
    telemetry_divider_ = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::round(cfg.control_rate_hz / cfg.state_publish_rate_hz)));
    RCLCPP_INFO(get_logger(),
                "Go2 simulated locomotion (C++20), 12 joints, %.0f Hz",
                cfg.control_rate_hz);
  }

private:
  [[nodiscard]] LocomotionParameters loadParameters() {
    LocomotionParameters p{};
    p.control_rate_hz = declare_parameter("control_rate_hz", p.control_rate_hz);
    p.state_publish_rate_hz =
        declare_parameter("state_publish_rate_hz", p.state_publish_rate_hz);
    p.stand_height_m = declare_parameter("stand_height_m", p.stand_height_m);
    p.crouch_height_m = declare_parameter("crouch_height_m", p.crouch_height_m);
    p.thigh_length_m = declare_parameter("thigh_length_m", p.thigh_length_m);
    p.calf_length_m = declare_parameter("calf_length_m", p.calf_length_m);
    p.gait_frequency_hz =
        declare_parameter("gait_frequency_hz", p.gait_frequency_hz);
    p.step_height_m = declare_parameter("step_height_m", p.step_height_m);
    p.step_length_m = declare_parameter("step_length_m", p.step_length_m);
    p.lateral_step_gain =
        declare_parameter("lateral_step_gain", p.lateral_step_gain);
    p.yaw_stride_gain = declare_parameter("yaw_stride_gain", p.yaw_stride_gain);
    p.max_linear_x = declare_parameter("max_linear_x", p.max_linear_x);
    p.max_linear_y = declare_parameter("max_linear_y", p.max_linear_y);
    p.max_angular_z = declare_parameter("max_angular_z", p.max_angular_z);
    p.command_timeout_s =
        declare_parameter("command_timeout_s", p.command_timeout_s);
    p.max_joint_rate_rad_s =
        declare_parameter("max_joint_rate_rad_s", p.max_joint_rate_rad_s);
    p.validate();
    return p;
  }

  void onVelocity(const geometry_msgs::msg::Twist &msg) {
    onVelocityComponents(msg.linear.x, msg.linear.y, msg.angular.z);
  }

  void onVelocityComponents(const double vx, const double vy, const double wz) {
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      RCLCPP_WARN(get_logger(), "Rejected nonfinite cmd_vel");
      return;
    }
    if (model_.mode() == LocomotionMode::Stop ||
        model_.mode() == LocomotionMode::Crouch) {
      return;
    }
    model_.setVelocity({vx, vy, wz});
    command_received_ = true;
    last_command_ = std::chrono::steady_clock::now();
  }

  void onMode(std::string mode) {
    mode.erase(std::remove_if(mode.begin(), mode.end(),
                              [](unsigned char c) { return std::isspace(c); }),
               mode.end());
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (mode == "stand")
      model_.setMode(LocomotionMode::Stand);
    else if (mode == "walk")
      model_.setMode(LocomotionMode::Walk);
    else if (mode == "crouch")
      model_.setMode(LocomotionMode::Crouch);
    else if (mode == "stop")
      model_.setMode(LocomotionMode::Stop);
    else {
      RCLCPP_WARN(get_logger(), "Unknown locomotion mode: %s", mode.c_str());
      return;
    }
    command_received_ = false;
    RCLCPP_INFO(get_logger(), "Locomotion mode: %s", mode.c_str());
  }

  void onJointStates(const sensor_msgs::msg::JointState &msg) {
    if (got_initial_state_ || msg.name.size() != msg.position.size())
      return;
    JointArray values{};
    std::array<bool, kJointCount> found{};
    const auto &names = GaitModel::jointNames();
    for (std::size_t idx = 0; idx < msg.name.size(); ++idx) {
      for (std::size_t j = 0; j < names.size(); ++j) {
        if (msg.name[idx] == names[j] && std::isfinite(msg.position[idx])) {
          values[j] = msg.position[idx];
          found[j] = true;
        }
      }
    }
    if (std::all_of(found.begin(), found.end(), [](bool x) { return x; })) {
      model_.setMeasuredJoints(values);
      got_initial_state_ = true;
    }
  }

  void update() {
    using Clock = std::chrono::steady_clock;
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    const bool fresh =
        command_received_ &&
        std::chrono::duration<double>(now - last_command_).count() <=
            model_.parameters().command_timeout_s;
    model_.step(dt, fresh);

    std_msgs::msg::Float64MultiArray command;
    const auto &references = model_.references();
    command.data.assign(references.begin(), references.end());
    pub_commands_->publish(command);

    if (++ticks_ % telemetry_divider_ == 0) {
      sensor_msgs::msg::JointState state;
      state.header.stamp = now_ros();
      const auto &names = GaitModel::jointNames();
      state.name.assign(names.begin(), names.end());
      state.position.assign(references.begin(), references.end());
      pub_references_->publish(state);

      sentinel_interfaces::msg::Joints custom_joints;
      custom_joints.position.reserve(references.size());
      for (const auto reference : references) {
        custom_joints.position.push_back(static_cast<float>(reference));
      }
      pub_custom_joints_->publish(custom_joints);

      const auto &feet = model_.footTargets();
      sentinel_interfaces::msg::PointArray target_feet;
      const auto to_point = [](const std::array<double, 3> &value) {
        sentinel_interfaces::msg::Point point;
        point.x = static_cast<float>(value[0]);
        point.y = static_cast<float>(value[1]);
        point.z = static_cast<float>(value[2]);
        return point;
      };
      target_feet.rf = to_point(feet[0]);
      target_feet.lf = to_point(feet[1]);
      target_feet.rh = to_point(feet[2]);
      target_feet.lh = to_point(feet[3]);
      pub_feet_->publish(target_feet);

      std_msgs::msg::String mode_msg;
      mode_msg.data = model_.moving()                           ? "walk"
                      : model_.mode() == LocomotionMode::Crouch ? "crouch"
                      : model_.mode() == LocomotionMode::Stop   ? "stop"
                                                                : "stand";
      pub_mode_->publish(mode_msg);
    }
  }

  [[nodiscard]] builtin_interfaces::msg::Time now_ros() const {
    return get_clock()->now();
  }

  GaitModel model_;
  bool command_received_{false};
  bool got_initial_state_{false};
  std::size_t ticks_{0};
  std::size_t telemetry_divider_{10};
  std::chrono::steady_clock::time_point last_command_{};
  std::chrono::steady_clock::time_point last_tick_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
  rclcpp::Subscription<sentinel_interfaces::msg::Velocities>::SharedPtr
      sub_custom_velocity_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      sub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_commands_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_references_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;
  rclcpp::Publisher<sentinel_interfaces::msg::Joints>::SharedPtr
      pub_custom_joints_;
  rclcpp::Publisher<sentinel_interfaces::msg::PointArray>::SharedPtr pub_feet_;
};

} // namespace sentinel::control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<sentinel::control::LocomotionNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("sentinel_locomotion"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
