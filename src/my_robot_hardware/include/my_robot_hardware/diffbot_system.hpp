#ifndef MY_ROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define MY_ROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot_hardware
{

class DiffBotSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool open_serial();
  void close_serial();
  bool process_line(const std::string & line);
  bool send_payload(const std::string & payload);
  static uint16_t crc16_ccitt(const std::string & text);

  rclcpp::Logger logger_{rclcpp::get_logger("DiffBotSystem")};
  std::string serial_port_;
  std::string rx_buffer_;
  int serial_fd_{-1};
  int baud_rate_{115200};
  int encoder_cpr_{450};
  int serial_timeout_ms_{500};
  int boot_wait_ms_{2000};
  uint32_t command_sequence_{0};
  bool active_{false};
  std::chrono::steady_clock::time_point last_state_time_{};
  std::array<double, 2> hw_position_{{0.0, 0.0}};
  std::array<double, 2> hw_velocity_{{0.0, 0.0}};
  std::array<double, 2> hw_command_{{0.0, 0.0}};
};

}  // namespace my_robot_hardware

#endif  // MY_ROBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
