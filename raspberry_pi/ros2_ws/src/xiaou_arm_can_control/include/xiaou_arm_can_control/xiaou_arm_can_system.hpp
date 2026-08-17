#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <linux/can.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"

namespace xiaou_arm_can_control
{

class XiaouArmCanSystem final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(XiaouArmCanSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kJointCount = 6;
  static constexpr int kCommandBaseId = 0x100;
  static constexpr int kFeedbackBaseId = 0x180;
  static constexpr std::uint8_t kCommandOpcode = 0x01;
  static constexpr std::uint8_t kFeedbackOpcode = 0x81;
  static constexpr std::uint8_t kFlagEnable = 1U << 0U;
  static constexpr std::uint8_t kFlagQuickStop = 1U << 2U;
  static constexpr std::uint8_t kStatusFault = 1U << 1U;
  static constexpr std::uint8_t kStatusEstop = 1U << 2U;

  bool get_bool_param(const std::string & name, bool fallback) const;
  std::string get_param(const std::string & name, const std::string & fallback) const;
  bool parse_joint_parameters();
  bool open_socket();
  void close_socket();
  bool send_position_command(std::size_t index);
  bool send_quick_stop_command(std::size_t index);
  bool decode_feedback(const ::can_frame & frame);

  std::string can_interface_ = "can0";
  int can_bitrate_{500000};
  int can_socket_{-1};
  bool motion_enabled_{false};
  bool protocol_confirmed_{false};
  bool estop_verified_{false};
  bool feedback_verified_{false};
  bool active_{false};
  std::uint8_t sequence_{0};
  std::array<int, kJointCount> node_ids_{};
  std::array<double, kJointCount> position_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> velocity_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> command_position_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> zero_offset_rad_{};
  std::array<int, kJointCount> direction_{};
  std::array<double, kJointCount> position_min_rad_{};
  std::array<double, kJointCount> position_max_rad_{};
  std::array<double, kJointCount> velocity_max_rad_s_{};
  std::array<bool, kJointCount> feedback_seen_{};
  std::array<std::chrono::steady_clock::time_point, kJointCount> last_feedback_time_{};
  std::chrono::milliseconds feedback_timeout_{200};
  std::chrono::steady_clock::time_point last_watchdog_log_time_{};
};

}  // namespace xiaou_arm_can_control
