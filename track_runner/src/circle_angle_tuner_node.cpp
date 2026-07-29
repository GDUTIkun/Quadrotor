#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
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

struct Point
{
  double x{};
  double y{};
};

struct Pose2D
{
  double x{};
  double y{};
  double yaw{};
};

std::string lower_copy(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double finite_or(double value, double fallback)
{
  return std::isfinite(value) ? value : fallback;
}

}  // namespace

class CircleAngleTunerNode : public rclcpp::Node
{
public:
  CircleAngleTunerNode()
  : Node("circle_angle_tuner_node")
  {
    radius_m_ = declare_parameter<double>("radius_m", 0.75);
    lookahead_distance_m_ = declare_parameter<double>("lookahead_distance_m", 0.25);
    linear_speed_m_s_ = declare_parameter<double>("linear_speed_m_s", 0.03);
    k_w_ = declare_parameter<double>("k_w", 0.6);
    k_w_rate_ = declare_parameter<double>("k_w_rate", 0.32);
    k_i_rate_ = declare_parameter<double>("k_i_rate", 0.9);
    k_d_rate_ = declare_parameter<double>("k_d_rate", 0.0);
    w_error_integral_max_ = declare_parameter<double>("w_error_integral_max", 0.5);
    w_error_derivative_filter_tau_s_ =
      declare_parameter<double>("w_error_derivative_filter_tau_s", 0.05);
    w_max_rad_s_ = declare_parameter<double>("w_max_rad_s", 0.8);
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 0.5);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.35);
    clockwise_ = declare_parameter<bool>("clockwise", true);
    publish_when_idle_ = declare_parameter<bool>("publish_when_idle", true);
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 50.0);

    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/car/pose");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/car/odom/carto");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/car/circle_angle_tuner/command");
    const auto speed_topic =
      declare_parameter<std::string>("speed_topic", "/car/circle_angle_tuner/speed");
    const auto k_w_topic =
      declare_parameter<std::string>("k_w_topic", "/car/circle_angle_tuner/k_w");
    const auto lookahead_topic =
      declare_parameter<std::string>("lookahead_topic", "/car/circle_angle_tuner/lookahead");
    const auto status_topic =
      declare_parameter<std::string>("status_topic", "/car/circle_angle_tuner/status");

    radius_m_ = std::max(0.05, finite_or(radius_m_, 0.75));
    lookahead_distance_m_ = std::max(0.01, finite_or(lookahead_distance_m_, 0.25));
    linear_speed_m_s_ = std::max(0.0, finite_or(linear_speed_m_s_, 0.03));
    k_w_ = std::max(0.0, finite_or(k_w_, 0.6));
    k_w_rate_ = std::max(0.0, finite_or(k_w_rate_, 0.32));
    k_i_rate_ = std::max(0.0, finite_or(k_i_rate_, 0.9));
    k_d_rate_ = std::max(0.0, finite_or(k_d_rate_, 0.0));
    w_error_integral_max_ = std::max(0.0, finite_or(w_error_integral_max_, 0.5));
    w_error_derivative_filter_tau_s_ =
      std::max(0.0, finite_or(w_error_derivative_filter_tau_s_, 0.05));
    w_max_rad_s_ = std::max(0.0, finite_or(w_max_rad_s_, 0.8));
    pose_timeout_s_ = std::max(0.02, finite_or(pose_timeout_s_, 0.5));
    odom_timeout_s_ = std::max(0.02, finite_or(odom_timeout_s_, 0.35));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 20, std::bind(&CircleAngleTunerNode::on_pose, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20, std::bind(&CircleAngleTunerNode::on_odom, this, std::placeholders::_1));
    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic, 10,
      std::bind(&CircleAngleTunerNode::on_command, this, std::placeholders::_1));
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      speed_topic, 10,
      std::bind(&CircleAngleTunerNode::on_speed, this, std::placeholders::_1));
    k_w_sub_ = create_subscription<std_msgs::msg::Float64>(
      k_w_topic, 10,
      std::bind(&CircleAngleTunerNode::on_k_w, this, std::placeholders::_1));
    lookahead_sub_ = create_subscription<std_msgs::msg::Float64>(
      lookahead_topic, 10,
      std::bind(&CircleAngleTunerNode::on_lookahead, this, std::placeholders::_1));

    const double period_s = std::max(0.001, 1.0 / std::max(1.0, control_rate_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&CircleAngleTunerNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Circle angle tuner ready: radius=%.3f m speed=%.3f m/s k_w=%.3f k_rate=(%.3f, %.3f, %.3f)",
      radius_m_, linear_speed_m_s_, k_w_, k_w_rate_, k_i_rate_, k_d_rate_);
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_ = Pose2D{
      msg->pose.position.x,
      msg->pose.position.y,
      quaternion_to_yaw(msg->pose.orientation)};
    has_pose_ = true;
    last_pose_time_ = now();
  }

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
      if (!pose_is_fresh()) {
        running_ = false;
        status_reason_ = "pose_unavailable";
        publish_stop();
        publish_status();
        return;
      }
      configure_circle_from_pose();
      reset_pid_state();
      last_control_time_ = now();
      running_ = true;
      status_reason_ = odom_is_fresh() ? "running" : "waiting_odom";
      RCLCPP_INFO(
        get_logger(), "Circle angle tuner started: center=(%.3f, %.3f), clockwise=%s",
        center_.x, center_.y, clockwise_ ? "true" : "false");
    } else if (command == "stop" || command == "pause") {
      running_ = false;
      last_yaw_error_ = 0.0;
      last_w_error_ = 0.0;
      reset_pid_state();
      status_reason_ = "idle";
      publish_stop();
    } else if (command == "reverse") {
      clockwise_ = !clockwise_;
      if (pose_is_fresh()) {
        configure_circle_from_pose();
      }
      reset_pid_state();
      RCLCPP_INFO(get_logger(), "Circle direction reversed: clockwise=%s", clockwise_ ? "true" : "false");
    } else if (command == "reset_pid") {
      reset_pid_state();
    } else {
      RCLCPP_WARN(get_logger(), "Unknown circle angle tuner command: %s", msg->data.c_str());
    }
    publish_status();
  }

  void on_speed(const std_msgs::msg::Float64::SharedPtr msg)
  {
    linear_speed_m_s_ = std::max(0.0, finite_or(msg->data, linear_speed_m_s_));
    publish_status();
  }

  void on_k_w(const std_msgs::msg::Float64::SharedPtr msg)
  {
    k_w_ = std::max(0.0, finite_or(msg->data, k_w_));
    publish_status();
  }

  void on_lookahead(const std_msgs::msg::Float64::SharedPtr msg)
  {
    lookahead_distance_m_ = std::max(0.01, finite_or(msg->data, lookahead_distance_m_));
    publish_status();
  }

  void tick()
  {
    if (!running_) {
      if (publish_when_idle_) {
        publish_stop();
      }
      status_reason_ = "idle";
      last_control_time_ = now();
      publish_status_throttled();
      return;
    }

    if (!pose_is_fresh()) {
      publish_stop();
      reset_pid_state();
      status_reason_ = "pose_unavailable";
      publish_status_throttled();
      return;
    }

    if (!odom_is_fresh()) {
      publish_stop();
      reset_pid_state();
      status_reason_ = "odom_unavailable";
      publish_status_throttled();
      return;
    }

    publish_control();
    publish_status_throttled();
  }

  bool pose_is_fresh() const
  {
    return has_pose_ && (now() - last_pose_time_).seconds() <= pose_timeout_s_;
  }

  bool odom_is_fresh() const
  {
    return has_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_s_;
  }

  void configure_circle_from_pose()
  {
    const double right_x = std::sin(pose_.yaw);
    const double right_y = -std::cos(pose_.yaw);
    const double side = clockwise_ ? 1.0 : -1.0;
    center_ = Point{
      pose_.x + side * radius_m_ * right_x,
      pose_.y + side * radius_m_ * right_y};
    reference_theta_ = std::atan2(pose_.y - center_.y, pose_.x - center_.x);
    has_circle_ = true;
  }

  void publish_control()
  {
    if (!has_circle_) {
      configure_circle_from_pose();
    }

    const auto stamp = now();
    const double dt = control_dt(stamp);
    const double direction = clockwise_ ? -1.0 : 1.0;
    reference_theta_ += direction * linear_speed_m_s_ * dt / radius_m_;
    const double target_theta = reference_theta_ + direction * lookahead_distance_m_ / radius_m_;
    last_target_ = Point{
      center_.x + radius_m_ * std::cos(target_theta),
      center_.y + radius_m_ * std::sin(target_theta)};

    const double dx = last_target_.x - pose_.x;
    const double dy = last_target_.y - pose_.y;
    last_target_yaw_ = std::atan2(dy, dx);
    last_yaw_error_ = normalize_angle(last_target_yaw_ - pose_.yaw);
    last_target_w_ = std::clamp(k_w_ * last_yaw_error_, -w_max_rad_s_, w_max_rad_s_);

    last_w_error_ = last_target_w_ - measured_w_rad_s_;
    w_error_integral_ = std::clamp(
      w_error_integral_ + last_w_error_ * dt,
      -w_error_integral_max_, w_error_integral_max_);
    update_derivative(dt);
    last_cmd_w_ = std::clamp(
      k_w_rate_ * last_w_error_ + k_i_rate_ * w_error_integral_ +
        k_d_rate_ * w_error_derivative_,
      -w_max_rad_s_, w_max_rad_s_);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_speed_m_s_;
    twist.angular.z = last_cmd_w_;
    cmd_pub_->publish(twist);
    status_reason_ = "running";
  }

  double control_dt(const rclcpp::Time & stamp)
  {
    double dt = (stamp - last_control_time_).seconds();
    if (dt <= 0.0 || dt > 0.2) {
      dt = 0.0;
    }
    last_control_time_ = stamp;
    return dt;
  }

  void update_derivative(double dt)
  {
    double raw_derivative = 0.0;
    if (has_previous_w_error_ && dt > 1e-6) {
      raw_derivative = (last_w_error_ - previous_w_error_) / dt;
    }
    const double alpha = w_error_derivative_filter_tau_s_ <= 0.0 || dt <= 0.0 ?
      1.0 : dt / (w_error_derivative_filter_tau_s_ + dt);
    w_error_derivative_ += alpha * (raw_derivative - w_error_derivative_);
    previous_w_error_ = last_w_error_;
    has_previous_w_error_ = true;
  }

  void reset_pid_state()
  {
    w_error_integral_ = 0.0;
    w_error_derivative_ = 0.0;
    previous_w_error_ = 0.0;
    has_previous_w_error_ = false;
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
           << " radius=" << radius_m_
           << " clockwise=" << (clockwise_ ? "true" : "false")
           << " speed=" << linear_speed_m_s_
           << " lookahead=" << lookahead_distance_m_
           << " k_w=" << k_w_
           << " k_w_rate=" << k_w_rate_
           << " k_i_rate=" << k_i_rate_
           << " k_d_rate=" << k_d_rate_
           << " pose=(" << pose_.x << "," << pose_.y << "," << pose_.yaw << ")"
           << " center=(" << center_.x << "," << center_.y << ")"
           << " reference_theta=" << reference_theta_
           << " target=(" << last_target_.x << "," << last_target_.y << ")"
           << " target_yaw=" << last_target_yaw_
           << " yaw_error=" << last_yaw_error_
           << " target_w=" << last_target_w_
           << " measured_w=" << measured_w_rad_s_
           << " w_error=" << last_w_error_
           << " w_error_integral=" << w_error_integral_
           << " w_error_derivative=" << w_error_derivative_
           << " cmd_w=" << last_cmd_w_
           << " w_max=" << w_max_rad_s_
           << " pose_fresh=" << (pose_is_fresh() ? "true" : "false")
           << " odom_fresh=" << (odom_is_fresh() ? "true" : "false")
           << " reason=" << status_reason_;
    msg.data = stream.str();
    status_pub_->publish(msg);
  }

  double radius_m_{0.75};
  double lookahead_distance_m_{0.25};
  double linear_speed_m_s_{0.03};
  double k_w_{0.6};
  double k_w_rate_{0.32};
  double k_i_rate_{0.9};
  double k_d_rate_{0.0};
  double w_error_integral_max_{0.5};
  double w_error_derivative_filter_tau_s_{0.05};
  double w_max_rad_s_{0.8};
  double pose_timeout_s_{0.5};
  double odom_timeout_s_{0.35};
  bool clockwise_{true};
  bool publish_when_idle_{true};

  bool running_{false};
  bool has_pose_{false};
  bool has_odom_{false};
  bool has_circle_{false};
  Pose2D pose_{};
  Point center_{};
  Point last_target_{};
  double reference_theta_{0.0};
  double measured_w_rad_s_{0.0};
  double last_target_yaw_{0.0};
  double last_yaw_error_{0.0};
  double last_target_w_{0.0};
  double last_w_error_{0.0};
  double w_error_integral_{0.0};
  double w_error_derivative_{0.0};
  double previous_w_error_{0.0};
  bool has_previous_w_error_{false};
  double last_cmd_w_{0.0};
  std::string status_reason_{"idle"};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr k_w_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lookahead_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleAngleTunerNode>());
  rclcpp::shutdown();
  return 0;
}
