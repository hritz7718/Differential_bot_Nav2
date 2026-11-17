#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "my_robot_hardware/diffbot_system.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
std::string frame(const std::string & payload)
{
  uint16_t crc = 0xFFFF;
  for (const unsigned char byte : payload) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  std::ostringstream result;
  result << payload << '*' << std::uppercase << std::hex << std::setw(4)
         << std::setfill('0') << crc << '\n';
  return result.str();
}

hardware_interface::ComponentInfo joint(const std::string & name)
{
  hardware_interface::ComponentInfo info;
  info.name = name;
  info.type = "joint";
  hardware_interface::InterfaceInfo position;
  position.name = "position";
  hardware_interface::InterfaceInfo velocity;
  velocity.name = "velocity";
  info.command_interfaces.push_back(velocity);
  info.state_interfaces = {position, velocity};
  return info;
}

class SerialTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    master_ = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    ASSERT_GE(master_, 0);
    ASSERT_EQ(grantpt(master_), 0);
    ASSERT_EQ(unlockpt(master_), 0);
    const char * slave = ptsname(master_);
    ASSERT_NE(slave, nullptr);
    hardware_interface::HardwareInfo info;
    info.name = "DiffBotSystem";
    info.type = "system";
    info.hardware_parameters = {{"serial_port", slave}, {"boot_wait_ms", "0"},
      {"encoder_counts_per_revolution", "450"}, {"serial_timeout_ms", "500"}};
    info.joints = {joint("base_left_wheel_joint"), joint("base_right_wheel_joint")};
    ASSERT_EQ(system_.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system_.on_configure(state_), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system_.on_activate(state_), hardware_interface::CallbackReturn::SUCCESS);
    drain();
  }

  void TearDown() override
  {
    system_.on_cleanup(state_);
    if (master_ >= 0) {
      close(master_);
    }
  }

  void send(const std::string & payload)
  {
    const auto message = frame(payload);
    ASSERT_EQ(write(master_, message.data(), message.size()),
      static_cast<ssize_t>(message.size()));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::string drain()
  {
    char buffer[256];
    const auto count = read(master_, buffer, sizeof(buffer));
    return count > 0 ? std::string(buffer, static_cast<std::size_t>(count)) : std::string();
  }

  hardware_interface::return_type read_state()
  {
    return system_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02));
  }

  int master_{-1};
  my_robot_hardware::DiffBotSystem system_;
  rclcpp_lifecycle::State state_;
};

TEST_F(SerialTest, DecodesWheelPositionsAndStopsOnBoardReset)
{
  send("S,42,450,900,1000,2000,0");
  ASSERT_EQ(read_state(), hardware_interface::return_type::OK);
  auto states = system_.export_state_interfaces();
  EXPECT_NEAR(states[0].get_value(), 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(states[2].get_value(), 4.0 * M_PI, 1e-6);

  send("S,1,0,0,0,0,0");
  EXPECT_EQ(read_state(), hardware_interface::return_type::ERROR);
  EXPECT_NE(drain().find(frame("C,0,0,0")), std::string::npos);
}

TEST_F(SerialTest, AcceptsSequenceWraparound)
{
  send("S,4294967295,0,0,0,0,0");
  ASSERT_EQ(read_state(), hardware_interface::return_type::OK);
  send("S,0,0,0,0,0,0");
  EXPECT_EQ(read_state(), hardware_interface::return_type::OK);
}

TEST_F(SerialTest, RejectsCorruptTelemetry)
{
  send("S,10,450,0,0,0,0");
  ASSERT_EQ(read_state(), hardware_interface::return_type::OK);
  const std::string corrupted = "S,11,900,0,0,0,0*0000\n";
  ASSERT_EQ(write(master_, corrupted.data(), corrupted.size()),
    static_cast<ssize_t>(corrupted.size()));
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_EQ(read_state(), hardware_interface::return_type::OK);
  auto states = system_.export_state_interfaces();
  EXPECT_NEAR(states[0].get_value(), 2.0 * M_PI, 1e-6);
}

TEST_F(SerialTest, ReportsMissingTelemetry)
{
  send("S,10,0,0,0,0,0");
  ASSERT_EQ(read_state(), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  EXPECT_EQ(read_state(), hardware_interface::return_type::ERROR);
  EXPECT_NE(drain().find(frame("C,0,0,0")), std::string::npos);
}

TEST_F(SerialTest, ClampsCommandsToFirmwareWheelSpeedLimit)
{
  auto commands = system_.export_command_interfaces();
  commands[0].set_value(8.0);
  commands[1].set_value(-9.0);
  EXPECT_EQ(system_.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::OK);
  EXPECT_NE(drain().find(frame("C,1,6000,-6000")), std::string::npos);
}

TEST_F(SerialTest, StopsForNonFiniteWheelCommand)
{
  auto commands = system_.export_command_interfaces();
  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(system_.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::ERROR);
  EXPECT_NE(drain().find(frame("C,0,0,0")), std::string::npos);
}
}  // namespace
