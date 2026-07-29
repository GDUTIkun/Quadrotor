#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <sstream>
#include <string>

namespace
{

std::string lower_copy(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

double sanitize_finite(double value, double fallback)
{
  return std::isfinite(value) ? value : fallback;
}

}  // namespace

class AngularRateTunerNode : public rclcpp::Node
{
public:
  AngularRateTunerNode()
  : Node("angular_rate_tuner_node")
  {
    linear_speed_m_s_ = declare_parameter<double>("linear_speed_m_s", 0.02);
    target_w_rad_s_ = declare_parameter<double>("target_w_rad_s", 0.2);
    k_w_rate_ = declare_parameter<double>("k_w_rate", 1.0);
    w_max_rad_s_ = declare_parameter<double>("w_max_rad_s", 0.3);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.35);
    publish_when_idle_ = declare_parameter<bool>("publish_when_idle", true);
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 50.0);

    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/car/odom/carto");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/car/angular_rate_tuner/command");
    const auto speed_topic =
      declare_parameter<std::string>("speed_topic", "/car/angular_rate_tuner/speed");
    const auto target_w_topic =
      declare_parameter<std::string>("target_w_topic", "/car/angular_rate_tuner/target_w");
    const auto k_w_rate_topic =
      declare_parameter<std::string>("k_w_rate_topic", "/car/angular_rate_tuner/k_w_rate");
    const auto status_topic =
      declare_parameter<std::string>("status_topic", "/car/angular_rate_tuner/status");

    linear_speed_m_s_ = sanitize_finite(linear_speed_m_s_, 0.02);
    target_w_rad_s_ = sanitize_finite(target_w_rad_s_, 0.2);
    k_w_rate_ = std::max(0.0, sanitize_finite(k_w_rate_, 1.0));
    w_max_rad_s_ = std::max(0.0, sanitize_finite(w_max_rad_s_, 0.3));
    odom_timeout_s_ = std::max(0.02, sanitize_finite(odom_timeout_s_, 0.35));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20, std::bind(&AngularRateTunerNode::on_odom, this, std::placeholders::_1));
    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic, 10,
      std::bind(&AngularRateTunerNode::on_command, this, std::placeholders::_1));
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      speed_topic, 10,
      std::bind(&AngularRateTunerNode::on_speed, this, std::placeholders::_1));
    target_w_sub_ = create_subscription<std_msgs::msg::Float64>(
      target_w_topic, 10,
      std::bind(&AngularRateTunerNode::on_target_w, this, std::placeholders::_1));
    k_w_rate_sub_ = create_subscription<std_msgs::msg::Float64>(
      k_w_rate_topic, 10,
      std::bind(&AngularRateTunerNode::on_k_w_rate, this, std::placeholders::_1));

    const double period_s = std::max(0.001, 1.0 / std::max(1.0, control_rate_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&AngularRateTunerNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Angular rate tuner ready: v=%.3f m/s target_w=%.3f rad/s k_w_rate=%.3f w_max=%.3f",
      linear_speed_m_s_, target_w_rad_s_, k_w_rate_, w_max_rad_s_);
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    measured_w_rad_s_ = msg->twist.twist.angular.z;
    has_odom_ = true;
    last_odom_time_ = now();
  }

  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto command = lower_copy(msg->data);
    if (command == "start") {
      running_ = true;
      RCLCPP_INFO(get_logger(), "Angular rate tuner started");
    } else if (command == "stop" || command == "pause") {
      running_ = false;
      publish_stop();
      RCLCPP_INFO(get_logger(), "Angular rate tuner stopped");
    } else if (command == "reverse") {
      target_w_rad_s_ = -target_w_rad_s_;
      RCLCPP_INFO(get_logger(), "Angular rate target reversed to %.3f rad/s", target_w_rad_s_);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown angular rate tuner command: %s", msg->data.c_str());
    }
    publish_status();
  }

  void on_speed(const std_msgs::msg::Float64::SharedPtr msg)
  {
    linear_speed_m_s_ = sanitize_finite(msg->data, linear_speed_m_s_);
    RCLCPP_INFO(get_logger(), "Linear speed set to %.3f m/s", linear_speed_m_s_);
    publish_status();
  }

  void on_target_w(const std_msgs::msg::Float64::SharedPtr msg)
  {
    target_w_rad_s_ = sanitize_finite(msg->data, target_w_rad_s_);
    RCLCPP_INFO(get_logger(), "Target angular velocity set to %.3f rad/s", target_w_rad_s_);
    publish_status();
  }

  void on_k_w_rate(const std_msgs::msg::Float64::SharedPtr msg)
  {
    k_w_rate_ = std::max(0.0, sanitize_finite(msg->data, k_w_rate_));
    RCLCPP_INFO(get_logger(), "k_w_rate set to %.3f", k_w_rate_);
    publish_status();
  }

  void tick()
  {
    if (!running_) {
      if (publish_when_idle_) {
        publish_stop();
      }
      publish_status_throttled();
      return;
    }

    if (!odom_is_fresh()) {
      publish_stop();
      status_reason_ = "odom_unavailable";
      publish_status_throttled();
      return;
    }

    last_w_error_ = target_w_rad_s_ - measured_w_rad_s_;
    last_cmd_w_ = std::clamp(k_w_rate_ * last_w_error_, -w_max_rad_s_, w_max_rad_s_);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_speed_m_s_;
    twist.angular.z = last_cmd_w_;
    cmd_pub_->publish(twist);

    status_reason_ = "running";
    publish_status_throttled();
  }

  bool odom_is_fresh() const
  {
    return has_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_s_;
  }

  void publish_stop()
  {
    last_cmd_w_ = 0.0;
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void publish_status_throttled()
  {
    const auto stamp = now();
    if ((stamp - last_status_time_).seconds() >= 0.2) {
      publish_status();
      last_status_time_ = stamp;
    }
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "running=" << (running_ ? "true" : "false")
           << " k_w_rate=" << k_w_rate_
           << " speed=" << linear_speed_m_s_
           << " target_w=" << target_w_rad_s_
           << " measured_w=" << measured_w_rad_s_
           << " w_error=" << last_w_error_
           << " cmd_w=" << last_cmd_w_
           << " w_max=" << w_max_rad_s_
           << " odom_fresh=" << (odom_is_fresh() ? "true" : "false")
           << " reason=" << status_reason_;
    msg.data = stream.str();
    status_pub_->publish(msg);
  }

  double linear_speed_m_s_{0.02};
  double target_w_rad_s_{0.2};
  double k_w_rate_{1.0};
  double w_max_rad_s_{0.3};
  double odom_timeout_s_{0.35};
  bool publish_when_idle_{true};

  bool running_{false};
  bool has_odom_{false};
  double measured_w_rad_s_{0.0};
  double last_w_error_{0.0};
  double last_cmd_w_{0.0};
  std::string status_reason_{"idle"};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_w_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr k_w_rate_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngularRateTunerNode>());
  rclcpp::shutdown();
  return 0;
}
