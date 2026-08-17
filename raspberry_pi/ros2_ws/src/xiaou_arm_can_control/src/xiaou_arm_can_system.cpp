#include "xiaou_arm_can_control/xiaou_arm_can_system.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace xiaou_arm_can_control
{

namespace
{
constexpr int kNodeMin = 1;
constexpr int kNodeMax = 0x3F;
constexpr double kPositionScale = 1.0e6;
constexpr double kVelocityScale = 1.0e3;
std::int32_t to_position_units(double value)
{
  const double scaled = std::round(value * kPositionScale);
  if (!std::isfinite(scaled) || scaled < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
    scaled > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
  {
    throw std::out_of_range("position is outside CAN int32 micro-radian range");
  }
  return static_cast<std::int32_t>(scaled);
}

std::int16_t to_velocity_units(double value)
{
  const double scaled = std::round(value * kVelocityScale);
  if (!std::isfinite(scaled) || scaled < static_cast<double>(std::numeric_limits<std::int16_t>::min()) ||
    scaled > static_cast<double>(std::numeric_limits<std::int16_t>::max()))
  {
    throw std::out_of_range("velocity is outside CAN int16 milli-radian/s range");
  }
  return static_cast<std::int16_t>(scaled);
}

std::int32_t read_i32_le(const std::uint8_t * data)
{
  const std::uint32_t raw = static_cast<std::uint32_t>(data[0]) |
    (static_cast<std::uint32_t>(data[1]) << 8U) |
    (static_cast<std::uint32_t>(data[2]) << 16U) |
    (static_cast<std::uint32_t>(data[3]) << 24U);
  return static_cast<std::int32_t>(raw);
}

std::int16_t read_i16_le(const std::uint8_t * data)
{
  const std::uint16_t raw = static_cast<std::uint16_t>(data[0]) |
    (static_cast<std::uint16_t>(data[1]) << 8U);
  return static_cast<std::int16_t>(raw);
}

void write_i32_le(std::uint8_t * data, std::int32_t value)
{
  const auto raw = static_cast<std::uint32_t>(value);
  data[0] = static_cast<std::uint8_t>(raw & 0xFFU);
  data[1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  data[2] = static_cast<std::uint8_t>((raw >> 16U) & 0xFFU);
  data[3] = static_cast<std::uint8_t>((raw >> 24U) & 0xFFU);
}

void write_i16_le(std::uint8_t * data, std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  data[0] = static_cast<std::uint8_t>(raw & 0xFFU);
  data[1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
}
}  // namespace

hardware_interface::CallbackReturn XiaouArmCanSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "expected exactly six joints");
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_interface_ = get_param("can_interface", "can0");
  try {
    can_bitrate_ = std::stoi(get_param("can_bitrate", "500000"));
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "invalid CAN bitrate: %s", exc.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (can_bitrate_ != 500000) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "XiaoU CAN V1 requires 500000 bit/s");
    return hardware_interface::CallbackReturn::ERROR;
  }
  motion_enabled_ = get_bool_param("motion_enabled", false);
  protocol_confirmed_ = get_bool_param("protocol_confirmed", false);
  estop_verified_ = get_bool_param("estop_verified", false);
  feedback_verified_ = get_bool_param("feedback_verified", false);
  if (!parse_joint_parameters()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::string XiaouArmCanSystem::get_param(const std::string & name, const std::string & fallback) const
{
  const auto it = info_.hardware_parameters.find(name);
  return it == info_.hardware_parameters.end() ? fallback : it->second;
}

bool XiaouArmCanSystem::get_bool_param(const std::string & name, bool fallback) const
{
  const std::string value = get_param(name, fallback ? "true" : "false");
  return value == "true" || value == "1";
}

bool XiaouArmCanSystem::parse_joint_parameters()
{
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto & joint = info_.joints[i];
    const auto get_joint = [&joint](const std::string & name, const std::string & fallback) {
        const auto it = joint.parameters.find(name);
        return it == joint.parameters.end() ? fallback : it->second;
      };
    try {
      node_ids_[i] = std::stoi(get_joint("node_id", "0"));
      zero_offset_rad_[i] = std::stod(get_joint("zero_offset_rad", "0"));
      direction_[i] = std::stoi(get_joint("direction", "1"));
      position_min_rad_[i] = std::stod(get_joint("position_min_rad", "-6.283185307"));
      position_max_rad_[i] = std::stod(get_joint("position_max_rad", "6.283185307"));
      velocity_max_rad_s_[i] = std::stod(get_joint("velocity_max_rad_s", "1"));
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "invalid parameters for %s: %s", joint.name.c_str(), exc.what());
      return false;
    }
    if ((direction_[i] != -1 && direction_[i] != 1) || position_min_rad_[i] >= position_max_rad_[i] ||
      !std::isfinite(zero_offset_rad_[i]) || !std::isfinite(velocity_max_rad_s_[i]) || velocity_max_rad_s_[i] <= 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "invalid limits or direction for %s", joint.name.c_str());
      return false;
    }
    if (node_ids_[i] < kNodeMin || node_ids_[i] > kNodeMax) {
      RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "node_id for %s must be 1..63", joint.name.c_str());
      return false;
    }
    for (std::size_t previous = 0; previous < i; ++previous) {
      if (node_ids_[previous] == node_ids_[i]) {
        RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "duplicate node_id %d", node_ids_[i]);
        return false;
      }
    }
  }
  if (motion_enabled_ && (!protocol_confirmed_ || !estop_verified_ || !feedback_verified_)) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "motion requires protocol, estop, and feedback verification");
    return false;
  }
  return true;
}

std::vector<hardware_interface::StateInterface> XiaouArmCanSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> XiaouArmCanSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]);
  }
  return interfaces;
}

bool XiaouArmCanSystem::open_socket()
{
  can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "socket(PF_CAN) failed: %s", std::strerror(errno));
    return false;
  }
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "CAN interface lookup failed: %s", std::strerror(errno));
    close_socket();
    return false;
  }
  struct sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;
  if (::bind(can_socket_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "CAN bind failed: %s", std::strerror(errno));
    close_socket();
    return false;
  }
  const int flags = ::fcntl(can_socket_, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
  }
  return true;
}

void XiaouArmCanSystem::close_socket()
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
}

hardware_interface::CallbackReturn XiaouArmCanSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!motion_enabled_ || !protocol_confirmed_ || !estop_verified_ || !feedback_verified_) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "activation refused: hardware safety gate is closed");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!open_socket()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  feedback_seen_.fill(false);
  last_feedback_time_.fill(std::chrono::steady_clock::time_point{});
  last_watchdog_log_time_ = std::chrono::steady_clock::time_point{};
  sequence_ = 0;
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XiaouArmCanSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (can_socket_ >= 0) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (!send_quick_stop_command(i)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("xiaou_arm_can_system"),
          "failed to send quick-stop command for joint %zu", i + 1);
      }
    }
  }
  active_ = false;
  close_socket();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool XiaouArmCanSystem::send_position_command(std::size_t index)
{
  struct can_frame frame{};
  frame.can_id = static_cast<canid_t>(kCommandBaseId + node_ids_[index]);
  frame.can_dlc = CAN_MAX_DLEN;
  frame.data[0] = kCommandOpcode;
  frame.data[1] = static_cast<std::uint8_t>((sequence_ & 0x0FU) << 4U) | kFlagEnable;
  try {
    const double physical_position = static_cast<double>(direction_[index]) *
      (command_position_[index] - zero_offset_rad_[index]);
    const auto position_units = to_position_units(physical_position);
    const auto velocity_units = to_velocity_units(velocity_max_rad_s_[index]);
    write_i32_le(&frame.data[2], position_units);
    write_i16_le(&frame.data[6], velocity_units);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "cannot encode joint %zu command: %s", index + 1, exc.what());
    return false;
  }
  return ::write(can_socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool XiaouArmCanSystem::send_quick_stop_command(std::size_t index)
{
  struct can_frame frame{};
  frame.can_id = static_cast<canid_t>(kCommandBaseId + node_ids_[index]);
  frame.can_dlc = CAN_MAX_DLEN;
  frame.data[0] = kCommandOpcode;
  frame.data[1] = static_cast<std::uint8_t>((sequence_ & 0x0FU) << 4U) | kFlagQuickStop;
  double physical_position = 0.0;
  if (std::isfinite(position_[index])) {
    physical_position = static_cast<double>(direction_[index]) *
      (position_[index] - zero_offset_rad_[index]);
  }
  try {
    write_i32_le(&frame.data[2], to_position_units(physical_position));
    write_i16_le(&frame.data[6], 0);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      rclcpp::get_logger("xiaou_arm_can_system"),
      "cannot encode joint %zu quick-stop command: %s", index + 1, exc.what());
    return false;
  }
  return ::write(can_socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool XiaouArmCanSystem::decode_feedback(const ::can_frame & frame)
{
  if (frame.can_dlc != CAN_MAX_DLEN ||
    (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U) {
    return false;
  }
  const int node_id = static_cast<int>(frame.can_id) - kFeedbackBaseId;
  const auto it = std::find(node_ids_.begin(), node_ids_.end(), node_id);
  if (it == node_ids_.end() || frame.data[0] != kFeedbackOpcode) {
    return false;
  }
  const std::size_t index = static_cast<std::size_t>(std::distance(node_ids_.begin(), it));
  const auto status = frame.data[1];
  if ((status & kStatusFault) != 0U || (status & kStatusEstop) != 0U) {
    active_ = false;
  }
  position_[index] = static_cast<double>(direction_[index]) *
    (static_cast<double>(read_i32_le(&frame.data[2])) / kPositionScale) + zero_offset_rad_[index];
  velocity_[index] = static_cast<double>(direction_[index]) *
    (static_cast<double>(read_i16_le(&frame.data[6])) / kVelocityScale);
  if (!std::isfinite(position_[index]) || !std::isfinite(velocity_[index]) ||
    position_[index] < position_min_rad_[index] || position_[index] > position_max_rad_[index])
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("xiaou_arm_can_system"),
      "joint %zu feedback is outside configured finite position limits", index + 1);
    active_ = false;
    return false;
  }
  feedback_seen_[index] = true;
  last_feedback_time_[index] = std::chrono::steady_clock::now();
  return true;
}

hardware_interface::return_type XiaouArmCanSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }
  struct can_frame frame{};
  ssize_t received = 0;
  while ((received = ::read(can_socket_, &frame, sizeof(frame))) == static_cast<ssize_t>(sizeof(frame))) {
    decode_feedback(frame);
  }
  if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
    RCLCPP_ERROR(
      rclcpp::get_logger("xiaou_arm_can_system"), "CAN read failed: %s", std::strerror(errno));
    active_ = false;
    return hardware_interface::return_type::ERROR;
  }
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!feedback_seen_[i] || now - last_feedback_time_[i] > feedback_timeout_) {
      if (last_watchdog_log_time_ == std::chrono::steady_clock::time_point{} ||
        now - last_watchdog_log_time_ >= std::chrono::seconds(1))
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("xiaou_arm_can_system"),
          "feedback watchdog expired for joint %zu", i + 1);
        last_watchdog_log_time_ = now;
      }
      active_ = false;
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XiaouArmCanSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (command_position_[i] < position_min_rad_[i] || command_position_[i] > position_max_rad_[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("xiaou_arm_can_system"), "joint %zu command exceeds measured position limits", i + 1);
      return hardware_interface::return_type::ERROR;
    }
    if (!send_position_command(i)) {
      return hardware_interface::return_type::ERROR;
    }
  }
  sequence_ = static_cast<std::uint8_t>((sequence_ + 1U) & 0x0FU);
  return hardware_interface::return_type::OK;
}

}  // namespace xiaou_arm_can_control

PLUGINLIB_EXPORT_CLASS(
  xiaou_arm_can_control::XiaouArmCanSystem,
  hardware_interface::SystemInterface)
