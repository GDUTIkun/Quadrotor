#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr uint8_t kTouchFrameHeader = 0x65;
constexpr std::array<uint8_t, 3> kFrameEnd{0xFF, 0xFF, 0xFF};
constexpr std::array<uint8_t, 7> kExpectedFrame{0x65, 0x00, 0x01, 0x00, 0xFF, 0xFF, 0xFF};

std::string to_hex(const std::vector<uint8_t> & bytes)
{
  std::ostringstream stream;
  stream << std::uppercase << std::hex << std::setfill('0');
  for (size_t i = 0; i < bytes.size(); ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<int>(bytes[i]);
  }
  return stream.str();
}

#ifdef __linux__
speed_t to_termios_baud(const int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B115200;
  }
}
#endif

}  // namespace

class HmiTouchVerifyNode : public rclcpp::Node
{
public:
  HmiTouchVerifyNode()
  : Node("hmi_touch_verify_node")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyS7");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);

    open_serial();
    timer_ = create_wall_timer(20ms, [this]() { read_serial_once(); });
  }

  ~HmiTouchVerifyNode() override
  {
#ifdef __linux__
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
#endif
  }

private:
  void open_serial()
  {
#ifdef __linux__
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(
        get_logger(), "failed to open %s: %s", serial_port_.c_str(), std::strerror(errno));
      return;
    }

    termios options {};
    if (tcgetattr(serial_fd_, &options) != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "failed to read serial attributes for %s: %s. If this is a 40pin UART, enable its "
        "device-tree overlay and reboot first.",
        serial_port_.c_str(), std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    cfmakeraw(&options);
    const speed_t baud = to_termios_baud(baud_rate_);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(get_logger(), "failed to configure serial port: %s", std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    RCLCPP_INFO(
      get_logger(), "listening on %s at %d baud, waiting for 65 00 01 00 FF FF FF",
      serial_port_.c_str(), baud_rate_);
#else
    RCLCPP_ERROR(get_logger(), "this verifier needs Linux termios. Run it on the Orange Pi.");
#endif
  }

  void read_serial_once()
  {
#ifdef __linux__
    if (serial_fd_ < 0) {
      return;
    }

    std::array<uint8_t, 128> buffer {};
    const ssize_t count = read(serial_fd_, buffer.data(), buffer.size());
    if (count <= 0) {
      return;
    }

    rx_buffer_.insert(rx_buffer_.end(), buffer.begin(), buffer.begin() + count);
    parse_frames();
#endif
  }

  void parse_frames()
  {
    while (rx_buffer_.size() >= 7) {
      auto header = std::find(rx_buffer_.begin(), rx_buffer_.end(), kTouchFrameHeader);
      rx_buffer_.erase(rx_buffer_.begin(), header);

      if (rx_buffer_.size() < 7) {
        return;
      }

      const bool has_frame_end =
        rx_buffer_[4] == kFrameEnd[0] && rx_buffer_[5] == kFrameEnd[1] &&
        rx_buffer_[6] == kFrameEnd[2];
      if (!has_frame_end) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      const std::vector<uint8_t> frame(rx_buffer_.begin(), rx_buffer_.begin() + 7);
      const uint8_t page_id = frame[1];
      const uint8_t component_id = frame[2];
      const uint8_t event = frame[3];

      RCLCPP_INFO(
        get_logger(), "touch frame: %s page=%u component=%u event=%u",
        to_hex(frame).c_str(), page_id, component_id, event);

      if (std::equal(kExpectedFrame.begin(), kExpectedFrame.end(), frame.begin())) {
        RCLCPP_INFO(get_logger(), "matched expected frame: 65 00 01 00 FF FF FF");
      }

      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 7);
    }
  }

  std::string serial_port_;
  int baud_rate_ = 115200;
  std::vector<uint8_t> rx_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;

#ifdef __linux__
  int serial_fd_ = -1;
#endif
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HmiTouchVerifyNode>());
  rclcpp::shutdown();
  return 0;
}
