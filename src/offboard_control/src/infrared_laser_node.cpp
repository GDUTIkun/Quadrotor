#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace
{
std::string errno_message(const std::string & action)
{
  return action + ": " + std::strerror(errno);
}

std::string resolve_gpiochip_path(const std::string & requested_path, unsigned int line_offset)
{
  if (requested_path != "auto") {
    return requested_path;
  }

  const std::string expected_line_name = "GPIO" + std::to_string(line_offset);
  std::string first_candidate;

  for (int chip_index = 0; chip_index < 32; ++chip_index) {
    const std::string chip_path = "/dev/gpiochip" + std::to_string(chip_index);
    if (!std::filesystem::exists(chip_path)) {
      continue;
    }

    const int fd = ::open(chip_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
      continue;
    }

    gpiochip_info chip_info{};
    if (::ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &chip_info) == 0 &&
      line_offset < chip_info.lines)
    {
      gpio_v2_line_info line_info{};
      line_info.offset = line_offset;
      if (::ioctl(fd, GPIO_V2_GET_LINEINFO_IOCTL, &line_info) == 0) {
        if (expected_line_name == line_info.name) {
          ::close(fd);
          return chip_path;
        }
        if (first_candidate.empty()) {
          first_candidate = chip_path;
        }
      }
    }

    ::close(fd);
  }

  if (!first_candidate.empty()) {
    return first_candidate;
  }

  throw std::runtime_error(
    "could not auto-detect a GPIO chip containing line " + std::to_string(line_offset) +
    "; set the gpiochip parameter explicitly, for example /dev/gpiochip4");
}

class GpioOutputLine final
{
public:
  GpioOutputLine(
    const std::string & chip_path,
    unsigned int line_offset,
    bool initial_value,
    const std::string & consumer)
  {
    chip_fd_ = ::open(chip_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_fd_ < 0) {
      throw std::runtime_error(errno_message("open " + chip_path));
    }

    gpiochip_info chip_info{};
    if (::ioctl(chip_fd_, GPIO_GET_CHIPINFO_IOCTL, &chip_info) < 0) {
      throw std::runtime_error(errno_message("read GPIO chip info"));
    }
    if (line_offset >= chip_info.lines) {
      throw std::runtime_error(
        "GPIO line " + std::to_string(line_offset) +
        " is outside chip range 0.." + std::to_string(chip_info.lines - 1));
    }

    gpio_v2_line_request request{};
    request.offsets[0] = line_offset;
    request.num_lines = 1;
    std::strncpy(request.consumer, consumer.c_str(), sizeof(request.consumer) - 1);
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    request.config.num_attrs = 1;
    request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    request.config.attrs[0].attr.values = initial_value ? 1 : 0;
    request.config.attrs[0].mask = 1;

    if (::ioctl(chip_fd_, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
      throw std::runtime_error(errno_message("request GPIO line"));
    }

    line_fd_ = request.fd;
  }

  GpioOutputLine(const GpioOutputLine &) = delete;
  GpioOutputLine & operator=(const GpioOutputLine &) = delete;

  ~GpioOutputLine()
  {
    if (line_fd_ >= 0) {
      ::close(line_fd_);
    }
    if (chip_fd_ >= 0) {
      ::close(chip_fd_);
    }
  }

  void set_value(bool value)
  {
    gpio_v2_line_values values{};
    values.mask = 1;
    values.bits = value ? 1 : 0;

    if (::ioctl(line_fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0) {
      throw std::runtime_error(errno_message("set GPIO line value"));
    }
  }

private:
  int chip_fd_{-1};
  int line_fd_{-1};
};
}  // namespace

class InfraredLaserNode final : public rclcpp::Node
{
public:
  InfraredLaserNode()
  : Node("infrared_laser_node")
  {
    gpiochip_path_ = declare_parameter<std::string>("gpiochip", "auto");
    gpio_line_ = declare_parameter<int>("gpio_line", 18);
    active_high_ = declare_parameter<bool>("active_high", true);
    enabled_ = declare_parameter<bool>("initially_on", false);
    shutdown_off_ = declare_parameter<bool>("shutdown_off", true);
    dry_run_ = declare_parameter<bool>("dry_run", false);
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "infrared_laser/enable");
    const auto state_topic =
      declare_parameter<std::string>("state_topic", "infrared_laser/state");
    const auto service_name =
      declare_parameter<std::string>("service_name", "infrared_laser/set_enabled");

    if (gpio_line_ < 0) {
      throw std::invalid_argument("gpio_line must be >= 0");
    }

    state_pub_ = create_publisher<std_msgs::msg::Bool>(
      state_topic, rclcpp::QoS(1).reliable().transient_local());
    command_sub_ = create_subscription<std_msgs::msg::Bool>(
      command_topic, 10,
      std::bind(&InfraredLaserNode::command_callback, this, std::placeholders::_1));
    service_ = create_service<std_srvs::srv::SetBool>(
      service_name,
      std::bind(
        &InfraredLaserNode::service_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    if (!dry_run_) {
      gpiochip_path_ = resolve_gpiochip_path(
        gpiochip_path_, static_cast<unsigned int>(gpio_line_));
      gpio_line_handle_ = std::make_unique<GpioOutputLine>(
        gpiochip_path_, static_cast<unsigned int>(gpio_line_),
        logical_to_physical(enabled_), get_name());
    }

    publish_state();
    RCLCPP_INFO(
      get_logger(),
      "Infrared laser ready: gpiochip=%s line=%d active_high=%s initially_on=%s dry_run=%s",
      gpiochip_path_.c_str(), gpio_line_, active_high_ ? "true" : "false",
      enabled_ ? "true" : "false", dry_run_ ? "true" : "false");
  }

  ~InfraredLaserNode() override
  {
    if (!shutdown_off_) {
      return;
    }

    try {
      set_laser(false);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to turn laser off during shutdown: %s", error.what());
    }
  }

private:
  bool logical_to_physical(bool enabled) const
  {
    return active_high_ ? enabled : !enabled;
  }

  void set_laser(bool enabled)
  {
    if (gpio_line_handle_) {
      gpio_line_handle_->set_value(logical_to_physical(enabled));
    }
    enabled_ = enabled;
    publish_state();
  }

  void command_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    try {
      set_laser(message->data);
      RCLCPP_INFO(get_logger(), "Infrared laser %s", enabled_ ? "ON" : "OFF");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to set infrared laser: %s", error.what());
    }
  }

  void service_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    try {
      set_laser(request->data);
      response->success = true;
      response->message = enabled_ ? "infrared laser on" : "infrared laser off";
      RCLCPP_INFO(get_logger(), "Infrared laser %s", enabled_ ? "ON" : "OFF");
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Failed to set infrared laser: %s", error.what());
    }
  }

  void publish_state()
  {
    std_msgs::msg::Bool state;
    state.data = enabled_;
    state_pub_->publish(state);
  }

  std::string gpiochip_path_;
  int gpio_line_{18};
  bool active_high_{true};
  bool enabled_{false};
  bool shutdown_off_{true};
  bool dry_run_{false};

  std::unique_ptr<GpioOutputLine> gpio_line_handle_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<InfraredLaserNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("infrared_laser_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}



/*
ros2 topic pub --once /infrared_laser/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /infrared_laser/enable std_msgs/msg/Bool "{data: false}"

*/