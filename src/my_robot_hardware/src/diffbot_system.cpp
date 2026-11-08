#include "my_robot_hardware/diffbot_system.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_hardware
{
namespace
{
constexpr std::size_t kLeft = 0;
constexpr std::size_t kRight = 1;
constexpr double kTwoPi = 6.28318530717958647692;

int parse_int_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name, int fallback)
{
  const auto parameter = info.hardware_parameters.find(name);
  return parameter == info.hardware_parameters.end() ? fallback : std::stoi(parameter->second);
}
}  // namespace

hardware_interface::CallbackReturn DiffBotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto port = info.hardware_parameters.find("serial_port");
  if (port == info.hardware_parameters.end() || port->second.empty()) {
    RCLCPP_ERROR(logger_, "The serial_port hardware parameter is required");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    serial_port_ = port->second;
    baud_rate_ = parse_int_parameter(info, "baud_rate", 115200);
    encoder_cpr_ = parse_int_parameter(info, "encoder_counts_per_revolution", 450);
    serial_timeout_ms_ = parse_int_parameter(info, "serial_timeout_ms", 500);
    boot_wait_ms_ = parse_int_parameter(info, "boot_wait_ms", 2000);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "Invalid hardware parameter: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (encoder_cpr_ <= 0 || serial_timeout_ms_ <= 0 || boot_wait_ms_ < 0) {
    RCLCPP_ERROR(logger_, "CPR and timeout must be positive; boot wait cannot be negative");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "Expected exactly two wheel joints, got %zu", info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
      joint.state_interfaces.size() != 2)
    {
      RCLCPP_ERROR(
        logger_, "Joint %s has an unexpected interface layout", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::fill(hw_position_.begin(), hw_position_.end(), 0.0);
  std::fill(hw_velocity_.begin(), hw_velocity_.end(), 0.0);
  std::fill(hw_command_.begin(), hw_command_.end(), 0.0);
  rx_buffer_.clear();

  if (!open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger_, "Waiting %d ms for Arduino boot on %s", boot_wait_ms_, serial_port_.c_str());
  std::this_thread::sleep_for(std::chrono::milliseconds(boot_wait_ms_));
  tcflush(serial_fd_, TCIOFLUSH);
  last_state_time_ = std::chrono::steady_clock::now();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(hw_command_.begin(), hw_command_.end(), 0.0);
  active_ = true;
  last_state_time_ = std::chrono::steady_clock::now();
  send_payload("C,0,0,0");
  RCLCPP_INFO(logger_, "DiffBot hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  hw_command_ = {{0.0, 0.0}};
  send_payload("C,0,0,0");
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(
    "base_left_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_position_[kLeft]);
  interfaces.emplace_back(
    "base_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocity_[kLeft]);
  interfaces.emplace_back(
    "base_right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_position_[kRight]);
  interfaces.emplace_back(
    "base_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocity_[kRight]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(
    "base_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_command_[kLeft]);
  interfaces.emplace_back(
    "base_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_command_[kRight]);
  return interfaces;
}

hardware_interface::return_type DiffBotSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<char, 256> buffer{};
  while (true) {
    const ssize_t count = ::read(serial_fd_, buffer.data(), buffer.size());
    if (count > 0) {
      rx_buffer_.append(buffer.data(), static_cast<std::size_t>(count));
    } else if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    } else {
      RCLCPP_ERROR(logger_, "Serial read failed: %s", std::strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
  }

  std::size_t newline = 0;
  while ((newline = rx_buffer_.find('\n')) != std::string::npos) {
    std::string line = rx_buffer_.substr(0, newline);
    rx_buffer_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    process_line(line);
  }
  if (rx_buffer_.size() > 4096) {
    rx_buffer_.clear();
    RCLCPP_WARN(logger_, "Discarded an oversized serial frame");
  }

  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_state_time_).count();
  if (active_ && age > serial_timeout_ms_) {
    RCLCPP_ERROR(logger_, "Arduino telemetry timed out after %ld ms", age);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  constexpr double kMaximumWheelSpeed = 8.0;
  const long left_mrad_s = std::lround(
    std::clamp(hw_command_[kLeft], -kMaximumWheelSpeed, kMaximumWheelSpeed) * 1000.0);
  const long right_mrad_s = std::lround(
    std::clamp(hw_command_[kRight], -kMaximumWheelSpeed, kMaximumWheelSpeed) * 1000.0);
  std::ostringstream payload;
  payload << "C," << ++command_sequence_ << ',' << left_mrad_s << ',' << right_mrad_s;
  return send_payload(payload.str()) ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

bool DiffBotSystem::open_serial()
{
  if (baud_rate_ != 115200) {
    RCLCPP_ERROR(logger_, "Only 115200 baud is currently supported");
    return false;
  }

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      logger_, "Cannot open %s: %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  termios options{};
  if (tcgetattr(serial_fd_, &options) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr failed: %s", std::strerror(errno));
    close_serial();
    return false;
  }
  cfmakeraw(&options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CRTSCTS;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr failed: %s", std::strerror(errno));
    close_serial();
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void DiffBotSystem::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool DiffBotSystem::process_line(const std::string & line)
{
  const auto separator = line.rfind('*');
  if (separator == std::string::npos || separator + 5 != line.size()) {
    return false;
  }
  const std::string payload = line.substr(0, separator);
  unsigned int expected_crc = 0;
  if (std::sscanf(line.c_str() + separator + 1, "%4x", &expected_crc) != 1 ||
    crc16_ccitt(payload) != expected_crc)
  {
    return false;
  }
  if (payload.rfind("S,", 0) != 0) {
    return true;
  }

  unsigned int sequence = 0;
  long long left_ticks = 0;
  long long right_ticks = 0;
  long left_mrad_s = 0;
  long right_mrad_s = 0;
  unsigned int flags = 0;
  const int fields = std::sscanf(
    payload.c_str(), "S,%u,%lld,%lld,%ld,%ld,%u",
    &sequence, &left_ticks, &right_ticks, &left_mrad_s, &right_mrad_s, &flags);
  if (fields != 6) {
    return false;
  }

  (void)sequence;
  (void)flags;
  const double radians_per_count = kTwoPi / static_cast<double>(encoder_cpr_);
  hw_position_[kLeft] = static_cast<double>(left_ticks) * radians_per_count;
  hw_position_[kRight] = static_cast<double>(right_ticks) * radians_per_count;
  hw_velocity_[kLeft] = static_cast<double>(left_mrad_s) / 1000.0;
  hw_velocity_[kRight] = static_cast<double>(right_mrad_s) / 1000.0;
  last_state_time_ = std::chrono::steady_clock::now();
  return true;
}

bool DiffBotSystem::send_payload(const std::string & payload)
{
  std::ostringstream frame;
  frame << payload << '*' << std::uppercase << std::hex << std::setw(4)
        << std::setfill('0') << crc16_ccitt(payload) << '\n';
  const std::string bytes = frame.str();
  std::size_t written = 0;
  while (written < bytes.size()) {
    const ssize_t count = ::write(serial_fd_, bytes.data() + written, bytes.size() - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      RCLCPP_ERROR(logger_, "Serial write failed: %s", std::strerror(errno));
      return false;
    }
  }
  return true;
}

uint16_t DiffBotSystem::crc16_ccitt(const std::string & text)
{
  uint16_t crc = 0xFFFF;
  for (const unsigned char byte : text) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

}  // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::DiffBotSystem, hardware_interface::SystemInterface)
