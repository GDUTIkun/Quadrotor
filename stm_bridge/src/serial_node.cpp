#include "stm_bridge/protocol.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace
{

constexpr double kGravity = 9.80665;

bool as_bool(const rclcpp::Parameter & parameter)
{
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return parameter.as_bool();
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    const auto value = parameter.as_string();
    return value == "1" || value == "true" || value == "yes" || value == "on";
  }
  return false;
}

speed_t baud_to_constant(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
#ifdef B576000
    case 576000:
      return B576000;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
#ifdef B576000
      return B576000;
#else
      return B460800;
#endif
  }
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

std::string hex16(std::uint16_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex;
  stream.width(4);
  stream.fill('0');
  stream << value;
  return stream.str();
}

std::string hex_dump(const std::vector<std::uint8_t> & data, std::size_t max_len = 48)
{
  std::ostringstream stream;
  const auto count = std::min(data.size(), max_len);
  for (std::size_t i = 0; i < count; ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(data[i]);
  }
  if (data.size() > max_len) {
    stream << " ...";
  }
  return stream.str();
}

class SerialPort
{
public:
  ~SerialPort()
  {
    close();
  }

  bool open(const std::string & port, int baudrate, std::string & error)
  {
    close();
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      error = std::strerror(errno);
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      error = std::strerror(errno);
      close();
      return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    const auto baud = baud_to_constant(baudrate);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      error = std::strerror(errno);
      close();
      return false;
    }
    return true;
  }

  bool is_open() const
  {
    return fd_ >= 0;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool write_all(const std::vector<std::uint8_t> & data, std::string & error)
  {
    if (fd_ < 0) {
      error = "serial port is not open";
      return false;
    }

    std::size_t written = 0;
    while (written < data.size()) {
      const auto result = ::write(fd_, data.data() + written, data.size() - written);
      if (result < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        error = std::strerror(errno);
        return false;
      }
      written += static_cast<std::size_t>(result);
    }
    return true;
  }

  std::vector<std::uint8_t> read_available(std::string & error)
  {
    std::vector<std::uint8_t> data;
    if (fd_ < 0) {
      return data;
    }

    int waiting = 0;
    if (ioctl(fd_, FIONREAD, &waiting) != 0) {
      error = std::strerror(errno);
      return data;
    }
    if (waiting <= 0) {
      return data;
    }

    data.resize(static_cast<std::size_t>(std::min(waiting, 512)));
    const auto result = ::read(fd_, data.data(), data.size());
    if (result < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        error = std::strerror(errno);
      }
      data.clear();
      return data;
    }
    data.resize(static_cast<std::size_t>(result));
    return data;
  }

private:
  int fd_{-1};
};

}  // namespace

class StmBridgeNode : public rclcpp::Node
{
public:
  StmBridgeNode()
  : Node("stm_bridge_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyS0");
    baudrate_ = declare_parameter<int>("baudrate", 576000);
    cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 50.0);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.3);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);
    debug_rx_hex_ = declare_parameter<bool>("debug_rx_hex", false);
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "car_base_link");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "car_odom");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/car/imu/data_valid");
    const auto status_topic = declare_parameter<std::string>("status_topic", "/car/stm/status");
    const auto wheel_odom_topic =
      declare_parameter<std::string>("wheel_odom_topic", "/car/odom/wheel");
    declare_parameter<bool>("publish_wheel_odom", false);
    declare_parameter<bool>("publish_odom_tf", false);
    publish_wheel_odom_enabled_ = as_bool(get_parameter("publish_wheel_odom"));
    publish_odom_tf_ = as_bool(get_parameter("publish_odom_tf"));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10, std::bind(&StmBridgeNode::on_cmd_vel, this, std::placeholders::_1));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, 20);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic, 10);
    if (publish_wheel_odom_enabled_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic, 20);
    }
    if (publish_wheel_odom_enabled_ && publish_odom_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    last_cmd_time_ = now();
    open_serial();

    cmd_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.001, 1.0 / cmd_rate_hz_))),
      std::bind(&StmBridgeNode::send_cmd_vel, this));
    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&StmBridgeNode::read_serial, this));
    reconnect_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&StmBridgeNode::reconnect_if_needed, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.1, 1.0 / diagnostics_rate_hz_))),
      std::bind(&StmBridgeNode::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(),
      "STM protocol v1.1: 0x01 CMD_VEL, 0x81 IMU, 0x82 WHEEL_ODOM, 0x83 STATUS");
  }

private:
  void open_serial()
  {
    if (serial_port_.is_open()) {
      return;
    }

    std::string error;
    if (serial_port_.open(port_, baudrate_, error)) {
      RCLCPP_INFO(get_logger(), "Opened STM serial port %s at %d", port_.c_str(), baudrate_);
    } else {
      RCLCPP_WARN(
        get_logger(), "Cannot open STM serial port %s: %s",
        port_.c_str(), error.c_str());
    }
  }

  void reconnect_if_needed()
  {
    if (!serial_port_.is_open()) {
      open_serial();
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    has_cmd_ = true;
    last_cmd_time_ = now();
  }

  void send_cmd_vel()
  {
    if (!serial_port_.is_open()) {
      return;
    }

    const double elapsed = (now() - last_cmd_time_).seconds();
    const bool enabled = has_cmd_ && elapsed <= cmd_timeout_s_;
    const double v = enabled ? last_cmd_.linear.x : 0.0;
    const double w = enabled ? last_cmd_.angular.z : 0.0;
    const auto payload = stm_bridge::encode_cmd_vel(v, w, enabled);
    const auto packet = stm_bridge::build_frame(stm_bridge::kMsgCmdVel, seq_, payload);
    seq_ = static_cast<std::uint8_t>(seq_ + 1U);

    std::string error;
    if (serial_port_.write_all(packet, error)) {
      ++tx_frames_;
    } else {
      RCLCPP_WARN(get_logger(), "STM serial write failed: %s", error.c_str());
      serial_port_.close();
    }
  }

  void read_serial()
  {
    if (!serial_port_.is_open()) {
      return;
    }

    std::string error;
    const auto data = serial_port_.read_available(error);
    if (!error.empty()) {
      RCLCPP_WARN(get_logger(), "STM serial read failed: %s", error.c_str());
      serial_port_.close();
      return;
    }
    if (data.empty()) {
      return;
    }

    raw_rx_bytes_ += data.size();
    has_raw_rx_ = true;
    last_raw_rx_time_ = now();
    if (debug_rx_hex_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STM RX raw %zu bytes: %s", data.size(), hex_dump(data).c_str());
    }

    for (const auto & frame : parser_.feed(data)) {
      ++rx_frames_;
      handle_frame(frame);
    }
  }

  void handle_frame(const stm_bridge::Frame & frame)
  {
    try {
      if (frame.msg_id == stm_bridge::kMsgImu) {
        publish_imu(frame.payload);
      } else if (frame.msg_id == stm_bridge::kMsgWheelOdom) {
        publish_wheel_odom(frame.payload);
      } else if (frame.msg_id == stm_bridge::kMsgStatus) {
        publish_status(frame.payload);
      } else {
        ++unknown_frames_;
        RCLCPP_DEBUG(get_logger(), "Ignored unknown STM msg_id=0x%02X", frame.msg_id);
      }
    } catch (const std::runtime_error & exc) {
      ++malformed_frames_;
      RCLCPP_WARN(
        get_logger(), "Dropped malformed STM frame 0x%02X: %s",
        frame.msg_id, exc.what());
    }
  }

  void publish_imu(const std::vector<std::uint8_t> & payload)
  {
    const auto data = stm_bridge::decode_imu(payload);
    ++imu_frames_;
    has_imu_ = true;
    last_imu_time_ = now();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_id_;

    const double roll = data.roll_cdeg / 100.0 * M_PI / 180.0;
    const double pitch = data.pitch_cdeg / 100.0 * M_PI / 180.0;
    const double yaw = data.yaw_cdeg / 100.0 * M_PI / 180.0;
    msg.orientation = euler_to_quaternion(roll, pitch, yaw);

    msg.angular_velocity.x = data.gx_cdeg_s * M_PI / (180.0 * 100.0);
    msg.angular_velocity.y = data.gy_cdeg_s * M_PI / (180.0 * 100.0);
    msg.angular_velocity.z = data.gz_cdeg_s * M_PI / (180.0 * 100.0);
    msg.linear_acceleration.x = data.ax_mg * kGravity / 1000.0;
    msg.linear_acceleration.y = data.ay_mg * kGravity / 1000.0;
    msg.linear_acceleration.z = data.az_mg * kGravity / 1000.0;

    msg.orientation_covariance = {0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.01};
    msg.angular_velocity_covariance =
      {0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004};
    msg.linear_acceleration_covariance =
      {0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04};
    imu_pub_->publish(msg);
  }

  void publish_wheel_odom(const std::vector<std::uint8_t> & payload)
  {
    if (!publish_wheel_odom_enabled_ || !odom_pub_) {
      return;
    }

    const auto data = stm_bridge::decode_wheel_odom(payload);
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = odom_frame_id_;
    msg.child_frame_id = base_frame_id_;
    msg.pose.pose.position.x = data.x_mm / 1000.0;
    msg.pose.pose.position.y = data.y_mm / 1000.0;
    msg.pose.pose.orientation = yaw_to_quaternion(data.yaw_mrad / 1000.0);
    msg.twist.twist.linear.x = data.vx_mm_s / 1000.0;
    msg.twist.twist.angular.z = data.wz_mrad_s / 1000.0;
    msg.pose.covariance[0] = 0.01;
    msg.pose.covariance[7] = 0.01;
    msg.pose.covariance[35] = 0.05;
    msg.twist.covariance[0] = 0.02;
    msg.twist.covariance[35] = 0.05;
    odom_pub_->publish(msg);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = msg.header;
      tf.child_frame_id = msg.child_frame_id;
      tf.transform.translation.x = msg.pose.pose.position.x;
      tf.transform.translation.y = msg.pose.pose.position.y;
      tf.transform.rotation = msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void publish_status(const std::vector<std::uint8_t> & payload)
  {
    const auto data = stm_bridge::decode_status(payload);
    ++status_frames_;
    has_status_ = true;
    last_status_time_ = now();
    last_status_ = data;

    publish_diagnostics();
  }

  void publish_diagnostics()
  {
    const auto stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "stm_bridge";
    status.hardware_id = port_;
    const auto error_flags = last_status_ ? last_status_->error_flags : 0U;
    if (!serial_port_.is_open()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "serial port is not open";
    } else if (!has_raw_rx_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "serial open, but no STM RX bytes";
    } else if (rx_frames_ == 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "STM RX bytes received, but no valid protocol frames";
    } else if (!has_imu_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "valid STM frames received, but no IMU frames";
    } else if (error_flags != 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "STM error flags set";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "ok";
    }

    add_value(status, "port", port_);
    add_value(status, "baudrate", std::to_string(baudrate_));
    add_value(status, "serial_open", serial_port_.is_open() ? "true" : "false");
    add_value(status, "raw_rx_bytes", std::to_string(raw_rx_bytes_));
    add_value(status, "rx_frames", std::to_string(rx_frames_));
    add_value(status, "tx_frames", std::to_string(tx_frames_));
    add_value(status, "imu_frames", std::to_string(imu_frames_));
    add_value(status, "status_frames", std::to_string(status_frames_));
    add_value(status, "unknown_frames", std::to_string(unknown_frames_));
    add_value(status, "malformed_frames", std::to_string(malformed_frames_));
    add_value(status, "crc_errors", std::to_string(parser_.crc_errors()));
    add_value(status, "dropped_bytes", std::to_string(parser_.dropped_bytes()));
    add_value(status, "parser_frames_received", std::to_string(parser_.frames_received()));
    add_value(status, "seconds_since_raw_rx", age_string(stamp, has_raw_rx_, last_raw_rx_time_));
    add_value(status, "seconds_since_imu", age_string(stamp, has_imu_, last_imu_time_));
    add_value(status, "seconds_since_status", age_string(stamp, has_status_, last_status_time_));
    if (last_status_) {
      add_value(status, "voltage_mv", std::to_string(last_status_->voltage_mv));
      add_value(status, "current_ma", std::to_string(last_status_->current_ma));
      add_value(status, "state", std::to_string(last_status_->state));
      add_value(status, "error_flags", hex16(last_status_->error_flags));
      add_value(status, "stamp_ms", std::to_string(last_status_->stamp_ms));
    } else {
      add_value(status, "error_flags", hex16(0));
      add_value(status, "stm_status", "not received");
    }

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = stamp;
    msg.status.push_back(status);
    status_pub_->publish(msg);
  }

  static std::string age_string(
    const rclcpp::Time & now_time, bool valid, const rclcpp::Time & then_time)
  {
    if (!valid) {
      return "never";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << (now_time - then_time).seconds();
    return stream.str();
  }

  static void add_value(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  }

  std::string port_;
  int baudrate_{};
  double cmd_rate_hz_{};
  double cmd_timeout_s_{};
  double diagnostics_rate_hz_{};
  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool debug_rx_hex_{};
  bool publish_wheel_odom_enabled_{};
  bool publish_odom_tf_{};

  SerialPort serial_port_;
  stm_bridge::FrameParser parser_;
  std::uint8_t seq_{};
  std::uint64_t raw_rx_bytes_{};
  std::uint64_t tx_frames_{};
  std::uint64_t rx_frames_{};
  std::uint64_t imu_frames_{};
  std::uint64_t status_frames_{};
  std::uint64_t unknown_frames_{};
  std::uint64_t malformed_frames_{};
  bool has_raw_rx_{};
  bool has_imu_{};
  bool has_status_{};
  rclcpp::Time last_raw_rx_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_status_time_;
  std::optional<stm_bridge::StatusPayload> last_status_;
  geometry_msgs::msg::Twist last_cmd_;
  bool has_cmd_{};
  rclcpp::Time last_cmd_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StmBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
