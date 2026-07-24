#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace offboard_control
{

class YoloSnapshotRequestNode final : public rclcpp::Node
{
public:
  YoloSnapshotRequestNode()
  : Node("yolo_snapshot_request_node")
  {
    service_name_ =
      declare_parameter<std::string>("service_name", "/yolo/capture_and_detect");
    service_timeout_s_ = declare_parameter<double>("service_timeout_s", 3.0);
    wait_service_timeout_s_ = declare_parameter<double>("wait_service_timeout_s", 1.0);
    repeat_interval_s_ = declare_parameter<double>("repeat_interval_s", 1.0);

    client_ = create_client<std_srvs::srv::Trigger>(service_name_);
    RCLCPP_INFO(
      get_logger(),
      "YOLO snapshot request client ready: service=%s",
      service_name_.c_str());
  }

  bool wait_for_service_once()
  {
    const auto timeout = duration_from_seconds(wait_service_timeout_s_);
    if (client_->wait_for_service(timeout)) {
      return true;
    }
    RCLCPP_WARN(
      get_logger(),
      "Waiting for service %s ...",
      service_name_.c_str());
    return false;
  }

  bool send_request()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client_->async_send_request(request);
    const auto timeout = duration_from_seconds(service_timeout_s_);
    const auto result = rclcpp::spin_until_future_complete(
      shared_from_this(), future, timeout);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "Request to %s timed out after %.1f seconds",
        service_name_.c_str(), service_timeout_s_);
      return false;
    }

    const auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(get_logger(), "Capture request accepted: %s", response->message.c_str());
      return true;
    }

    RCLCPP_WARN(get_logger(), "Capture request rejected: %s", response->message.c_str());
    return false;
  }

  void run_terminal_loop()
  {
    print_help();

    while (rclcpp::ok()) {
      while (rclcpp::ok() && !wait_for_service_once()) {
      }
      if (!rclcpp::ok()) {
        break;
      }

      std::cout << "\nyolo> " << std::flush;
      std::string line;
      if (!std::getline(std::cin, line)) {
        break;
      }

      const std::string command = trim(line);
      if (command == "q" || command == "quit" || command == "exit") {
        break;
      }
      if (command == "h" || command == "help" || command == "?") {
        print_help();
        continue;
      }

      const int request_count = parse_request_count(command);
      for (int index = 0; rclcpp::ok() && index < request_count; ++index) {
        send_request();
        if (index + 1 < request_count) {
          rclcpp::sleep_for(duration_from_seconds(repeat_interval_s_));
        }
      }
    }
  }

private:
  static std::chrono::nanoseconds duration_from_seconds(const double seconds)
  {
    const double safe_seconds = std::max(0.0, seconds);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(safe_seconds));
  }

  static std::string trim(const std::string & text)
  {
    const auto begin = std::find_if_not(
      text.begin(), text.end(),
      [](const unsigned char ch) { return std::isspace(ch) != 0; });
    const auto end = std::find_if_not(
      text.rbegin(), text.rend(),
      [](const unsigned char ch) { return std::isspace(ch) != 0; }).base();
    if (begin >= end) {
      return "";
    }
    return std::string(begin, end);
  }

  int parse_request_count(const std::string & command) const
  {
    if (command.empty()) {
      return 1;
    }

    std::istringstream stream(command);
    int count = 0;
    stream >> count;
    if (!stream.fail() && count > 0) {
      return count;
    }

    RCLCPP_WARN(
      get_logger(),
      "Unknown command '%s'; sending one capture request. Type 'h' for help.",
      command.c_str());
    return 1;
  }

  static void print_help()
  {
    std::cout
      << "Press Enter to request one YOLO snapshot.\n"
      << "Type a positive number to request that many times.\n"
      << "Type h for help, q to quit.\n";
  }

  std::string service_name_;
  double service_timeout_s_{3.0};
  double wait_service_timeout_s_{1.0};
  double repeat_interval_s_{1.0};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

}  // namespace offboard_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<offboard_control::YoloSnapshotRequestNode>();
  node->run_terminal_loop();
  rclcpp::shutdown();
  return 0;
}
