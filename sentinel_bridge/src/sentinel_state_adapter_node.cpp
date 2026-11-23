#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sentinel_interfaces/msg/contacts.hpp>
#include <sentinel_interfaces/msg/contacts_stamped.hpp>
#include <sentinel_interfaces/msg/imu.hpp>
#include <sentinel_interfaces/msg/joints.hpp>
#include <sentinel_interfaces/msg/pose.hpp>
#include <sentinel_interfaces/msg/velocities.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace sentinel::bridge {
class StateAdapterNode final : public rclcpp::Node {
public:
  StateAdapterNode() : Node("sentinel_state_adapter") {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    pub_imu_ = create_publisher<sentinel_interfaces::msg::Imu>("imu", 10);
    pub_pose_ = create_publisher<sentinel_interfaces::msg::Pose>("pose", 10);
    pub_velocity_ = create_publisher<sentinel_interfaces::msg::Velocities>(
        "velocity_state", 10);
    pub_joints_ =
        create_publisher<sentinel_interfaces::msg::Joints>("joints_state", 10);
    pub_contacts_ =
        create_publisher<sentinel_interfaces::msg::Contacts>("contacts", 10);
    pub_contacts_stamped_ =
        create_publisher<sentinel_interfaces::msg::ContactsStamped>(
            "contacts_stamped", 10);

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", sensor_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          sentinel_interfaces::msg::Imu data;
          data.orientation = msg->orientation;
          data.linear_acceleration = msg->linear_acceleration;
          data.angular_velocity = msg->angular_velocity;
          // The original custom message includes magnetic_field; Gazebo IMU has
          // no magnetometer.
          pub_imu_->publish(data);
        });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom/ground_truth", sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          const auto &p = msg->pose.pose.position;
          const auto &q = msg->pose.pose.orientation;
          const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
          const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
          const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
          const double siny = 2.0 * (q.w * q.z + q.x * q.y);
          const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
          sentinel_interfaces::msg::Pose pose;
          pose.x = static_cast<float>(p.x);
          pose.y = static_cast<float>(p.y);
          pose.z = static_cast<float>(p.z);
          pose.roll = static_cast<float>(std::atan2(sinr, cosr));
          pose.pitch =
              static_cast<float>(std::asin(std::clamp(sinp, -1.0, 1.0)));
          pose.yaw = static_cast<float>(std::atan2(siny, cosy));
          pub_pose_->publish(pose);
          sentinel_interfaces::msg::Velocities velocity;
          velocity.linear_x = static_cast<float>(msg->twist.twist.linear.x);
          velocity.linear_y = static_cast<float>(msg->twist.twist.linear.y);
          velocity.angular_z = static_cast<float>(msg->twist.twist.angular.z);
          pub_velocity_->publish(velocity);
        });
    sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", sensor_qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          constexpr std::array<const char *, 12> joint_names = {
              "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
              "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
              "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
              "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"};
          if (msg->name.size() != msg->position.size())
            return;
          sentinel_interfaces::msg::Joints state;
          state.position.resize(joint_names.size());
          for (std::size_t j = 0; j < joint_names.size(); ++j) {
            bool found = false;
            for (std::size_t k = 0; k < msg->name.size(); ++k) {
              if (msg->name[k] == joint_names[j] &&
                  std::isfinite(msg->position[k])) {
                state.position[j] = static_cast<float>(msg->position[k]);
                found = true;
                break;
              }
            }
            if (!found)
              return;
          }
          pub_joints_->publish(state);
        });

    constexpr std::array<const char *, 4> feet = {
        "rf_foot_contacts", "lf_foot_contacts", "rh_foot_contacts",
        "lh_foot_contacts"};
    for (std::size_t i = 0; i < feet.size(); ++i) {
      subs_contact_[i] = create_subscription<ros_gz_interfaces::msg::Contacts>(
          feet[i], sensor_qos,
          [this, i](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
            touching_[i] = !msg->contacts.empty();
            last_contact_[i] = std::chrono::steady_clock::now();
            seen_contact_[i] = true;
          });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(40), [this] {
      const auto now = std::chrono::steady_clock::now();
      sentinel_interfaces::msg::Contacts contact_msg;
      for (std::size_t i = 0; i < touching_.size(); ++i) {
        const bool fresh = seen_contact_[i] && (now - last_contact_[i] <
                                                std::chrono::milliseconds(250));
        contact_msg.contacts.push_back(fresh && touching_[i]);
      }
      pub_contacts_->publish(contact_msg);
      sentinel_interfaces::msg::ContactsStamped stamped;
      stamped.header.stamp = get_clock()->now();
      stamped.header.frame_id = "base_link";
      stamped.contacts = contact_msg.contacts;
      pub_contacts_stamped_->publish(stamped);
    });
    RCLCPP_INFO(get_logger(), "SENTINEL custom-state adapter started");
  }

private:
  rclcpp::Publisher<sentinel_interfaces::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sentinel_interfaces::msg::Pose>::SharedPtr pub_pose_;
  rclcpp::Publisher<sentinel_interfaces::msg::Velocities>::SharedPtr
      pub_velocity_;
  rclcpp::Publisher<sentinel_interfaces::msg::Joints>::SharedPtr pub_joints_;
  rclcpp::Publisher<sentinel_interfaces::msg::Contacts>::SharedPtr
      pub_contacts_;
  rclcpp::Publisher<sentinel_interfaces::msg::ContactsStamped>::SharedPtr
      pub_contacts_stamped_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  std::array<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr,
             4>
      subs_contact_{};
  std::array<bool, 4> touching_{};
  std::array<bool, 4> seen_contact_{};
  std::array<std::chrono::steady_clock::time_point, 4> last_contact_{};
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace sentinel::bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentinel::bridge::StateAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
