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
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{

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

class SerialLoopbackTestNode : public rclcpp::Node
{
public:
  SerialLoopbackTestNode()
  : Node("serial_loopback_test_node")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyS7");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    timeout_ms_ = declare_parameter<int>("timeout_ms", 500);

    open_serial();
    timer_ = create_wall_timer(1s, [this]() { run_one_test(); });
  }

  ~SerialLoopbackTestNode() override
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
        "failed to read serial attributes for %s: %s. Enable the UART overlay and reboot first.",
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
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(
        get_logger(), "failed to configure %s: %s", serial_port_.c_str(), std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(
      get_logger(),
      "loopback test opened %s at %d baud. Short TX and RX together before running this test.",
      serial_port_.c_str(), baud_rate_);
#else
    RCLCPP_ERROR(get_logger(), "this loopback test needs Linux termios.");
#endif
  }

  void run_one_test()
  {
#ifdef __linux__
    if (serial_fd_ < 0) {
      return;
    }

    const std::string payload = "OPI_UART_LOOP_" + std::to_string(sequence_++) + "\n";
    const auto * data = reinterpret_cast<const uint8_t *>(payload.data());
    const ssize_t written = write(serial_fd_, data, payload.size());
    if (written != static_cast<ssize_t>(payload.size())) {
      RCLCPP_ERROR(
        get_logger(), "write failed on %s: wrote %zd/%zu bytes: %s", serial_port_.c_str(), written,
        payload.size(), std::strerror(errno));
      return;
    }

    tcdrain(serial_fd_);
    std::vector<uint8_t> received;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);

    while (std::chrono::steady_clock::now() < deadline && received.size() < payload.size()) {
      pollfd poll_fd {};
      poll_fd.fd = serial_fd_;
      poll_fd.events = POLLIN;
      const int poll_result = poll(&poll_fd, 1, 20);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "poll failed: %s", std::strerror(errno));
        return;
      }
      if (poll_result == 0 || (poll_fd.revents & POLLIN) == 0) {
        continue;
      }

      std::array<uint8_t, 128> buffer {};
      const ssize_t count = read(serial_fd_, buffer.data(), buffer.size());
      if (count > 0) {
        received.insert(received.end(), buffer.begin(), buffer.begin() + count);
      }
    }

    const std::string received_text(received.begin(), received.end());
    if (received_text == payload) {
      ++pass_count_;
      RCLCPP_INFO(
        get_logger(), "PASS #%u received echo: %s", pass_count_, received_text.c_str());
      return;
    }

    ++fail_count_;
    RCLCPP_WARN(
      get_logger(),
      "FAIL #%u sent %zu bytes but received %zu bytes within %d ms. sent='%s' received_hex=%s",
      fail_count_, payload.size(), received.size(), timeout_ms_, payload.c_str(),
      to_hex(received).c_str());
#endif
  }

  std::string serial_port_;
  int baud_rate_ = 115200;
  int timeout_ms_ = 500;
  uint32_t sequence_ = 1;
  uint32_t pass_count_ = 0;
  uint32_t fail_count_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;

#ifdef __linux__
  int serial_fd_ = -1;
#endif
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialLoopbackTestNode>());
  rclcpp::shutdown();
  return 0;
}
