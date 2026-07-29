#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

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

enum class RunnerState
{
  Idle,
  Running,
  Paused,
  Done,
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

double distance(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

const char * state_name(RunnerState state)
{
  switch (state) {
    case RunnerState::Idle:
      return "idle";
    case RunnerState::Running:
      return "running";
    case RunnerState::Paused:
      return "paused";
    case RunnerState::Done:
      return "done";
  }
  return "unknown";
}

}  // namespace

class TrackRunnerNode : public rclcpp::Node
{
public:
  TrackRunnerNode()
  : Node("track_runner_node")
  {
    straight_length_m_ = declare_parameter<double>("straight_length_m", 1.5);
    radius_m_ = declare_parameter<double>("radius_m", 0.75);
    path_spacing_m_ = declare_parameter<double>("path_spacing_m", 0.03);
    speed_m_s_ = declare_parameter<double>("default_speed_m_s", 0.02);
    target_laps_ = declare_parameter<int>("default_laps", 1);
    lookahead_distance_m_ = declare_parameter<double>("lookahead_distance_m", 0.25);
    waypoint_tolerance_m_ = declare_parameter<double>("waypoint_tolerance_m", 0.08);
    goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.10);
    k_w_ = declare_parameter<double>("k_w", 1.8);
    k_w_rate_ = declare_parameter<double>("k_w_rate", 1.0);
    w_max_rad_s_ = declare_parameter<double>("w_max_rad_s", 0.6);
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 0.5);
    angular_velocity_timeout_s_ = declare_parameter<double>("angular_velocity_timeout_s", 0.35);
    use_angular_velocity_feedback_ =
      declare_parameter<bool>("use_angular_velocity_feedback", true);
    use_start_pose_as_origin_ = declare_parameter<bool>("use_start_pose_as_origin", true);
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 50.0);

    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/car/pose");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/car/odom/carto");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/car/track_runner/command");
    const auto speed_topic =
      declare_parameter<std::string>("speed_topic", "/car/track_runner/speed");
    const auto laps_topic =
      declare_parameter<std::string>("laps_topic", "/car/track_runner/laps");
    const auto status_topic =
      declare_parameter<std::string>("status_topic", "/car/track_runner/status");

    straight_length_m_ = std::max(0.01, straight_length_m_);
    radius_m_ = std::max(0.01, radius_m_);
    path_spacing_m_ = std::max(0.005, path_spacing_m_);
    speed_m_s_ = sanitize_speed(speed_m_s_);
    target_laps_ = std::max(1, target_laps_);
    lookahead_distance_m_ = std::max(0.01, lookahead_distance_m_);
    waypoint_tolerance_m_ = std::max(0.01, waypoint_tolerance_m_);
    goal_tolerance_m_ = std::max(0.01, goal_tolerance_m_);
    k_w_rate_ = std::max(0.0, k_w_rate_);
    lap_length_m_ = 2.0 * straight_length_m_ + 2.0 * M_PI * radius_m_;
    build_path();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 20, std::bind(&TrackRunnerNode::on_pose, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20, std::bind(&TrackRunnerNode::on_odom, this, std::placeholders::_1));
    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic, 10,
      std::bind(&TrackRunnerNode::on_command, this, std::placeholders::_1));
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      speed_topic, 10,
      std::bind(&TrackRunnerNode::on_speed, this, std::placeholders::_1));
    laps_sub_ = create_subscription<std_msgs::msg::Int32>(
      laps_topic, 10,
      std::bind(&TrackRunnerNode::on_laps, this, std::placeholders::_1));

    const double period_s = std::max(0.001, 1.0 / std::max(1.0, control_rate_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&TrackRunnerNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Closed-loop track runner ready: points=%zu lap=%.3f m speed=%.3f m/s laps=%d rate_feedback=%s",
      path_.size(), lap_length_m_, speed_m_s_, target_laps_,
      use_angular_velocity_feedback_ ? "on" : "off");
  }

private:
  double sanitize_speed(double speed) const
  {
    if (!std::isfinite(speed)) {
      return 0.02;
    }
    return std::max(0.0, speed);
  }

  void build_path()
  {
    path_.clear();
    cumulative_s_.clear();
    append_line(
      Point{origin_x_, origin_y_},
      Point{origin_x_, origin_y_ + straight_length_m_});
    append_arc(Point{origin_x_ + radius_m_, origin_y_ + straight_length_m_}, M_PI, -M_PI);
    append_line(
      Point{origin_x_ + 2.0 * radius_m_, origin_y_ + straight_length_m_},
      Point{origin_x_ + 2.0 * radius_m_, origin_y_});
    append_arc(Point{origin_x_ + radius_m_, origin_y_}, 0.0, -M_PI);

    cumulative_s_.resize(path_.size(), 0.0);
    for (std::size_t i = 1; i < path_.size(); ++i) {
      cumulative_s_[i] = cumulative_s_[i - 1] + distance(path_[i - 1], path_[i]);
    }
    if (!cumulative_s_.empty()) {
      lap_length_m_ = cumulative_s_.back();
    }
  }

  void push_point(const Point & point)
  {
    if (path_.empty() || distance(path_.back(), point) > 1e-6) {
      path_.push_back(point);
    }
  }

  void append_line(const Point & start, const Point & end)
  {
    const double length = distance(start, end);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / path_spacing_m_)));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      push_point(Point{
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t});
    }
  }

  void append_arc(const Point & center, double start_angle, double delta_angle)
  {
    const double length = std::abs(delta_angle) * radius_m_;
    const int steps = std::max(1, static_cast<int>(std::ceil(length / path_spacing_m_)));
    for (int i = 1; i <= steps; ++i) {
      const double theta = start_angle + delta_angle * static_cast<double>(i) /
        static_cast<double>(steps);
      push_point(Point{
        center.x + radius_m_ * std::cos(theta),
        center.y + radius_m_ * std::sin(theta)});
    }
  }

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
    has_angular_velocity_ = true;
    last_angular_velocity_time_ = now();
  }

  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto command = lower_copy(msg->data);
    if (command == "start") {
      if (!pose_is_fresh()) {
        RCLCPP_WARN(get_logger(), "Cannot start track runner: /car/pose is not available");
        state_ = RunnerState::Idle;
        publish_stop();
        status_reason_ = "pose_unavailable";
        publish_status();
        return;
      }
      if (use_start_pose_as_origin_) {
        origin_x_ = pose_.x;
        origin_y_ = pose_.y;
        build_path();
      }
      reset_progress();
      state_ = RunnerState::Running;
      RCLCPP_INFO(
        get_logger(), "Track runner started with A origin=(%.3f, %.3f)",
        origin_x_, origin_y_);
    } else if (command == "pause") {
      if (state_ == RunnerState::Running) {
        state_ = RunnerState::Paused;
        RCLCPP_INFO(get_logger(), "Track runner paused");
      }
    } else if (command == "resume") {
      if (state_ == RunnerState::Paused) {
        state_ = RunnerState::Running;
        RCLCPP_INFO(get_logger(), "Track runner resumed");
      }
    } else if (command == "stop") {
      state_ = RunnerState::Idle;
      reset_progress();
      publish_stop();
      RCLCPP_INFO(get_logger(), "Track runner stopped");
    } else if (command == "reset") {
      state_ = RunnerState::Idle;
      reset_progress();
      publish_stop();
      RCLCPP_INFO(get_logger(), "Track runner reset");
    } else {
      RCLCPP_WARN(get_logger(), "Unknown track runner command: %s", msg->data.c_str());
    }
    publish_status();
  }

  void on_speed(const std_msgs::msg::Float64::SharedPtr msg)
  {
    speed_m_s_ = sanitize_speed(msg->data);
    RCLCPP_INFO(get_logger(), "Track runner speed set to %.3f m/s", speed_m_s_);
    publish_status();
  }

  void on_laps(const std_msgs::msg::Int32::SharedPtr msg)
  {
    target_laps_ = std::max(1, msg->data);
    RCLCPP_INFO(get_logger(), "Track runner target laps set to %d", target_laps_);
    publish_status();
  }

  void tick()
  {
    if (state_ == RunnerState::Running) {
      if (!pose_is_fresh()) {
        publish_stop();
        status_reason_ = "pose_unavailable";
      } else {
        publish_tracking_command();
      }
    } else {
      publish_stop();
    }

    const auto stamp = now();
    if ((stamp - last_status_time_).seconds() >= 0.2) {
      publish_status();
      last_status_time_ = stamp;
    }
  }

  bool pose_is_fresh() const
  {
    return has_pose_ && (now() - last_pose_time_).seconds() <= pose_timeout_s_;
  }

  bool angular_velocity_is_fresh() const
  {
    return has_angular_velocity_ &&
      (now() - last_angular_velocity_time_).seconds() <= angular_velocity_timeout_s_;
  }

  void publish_tracking_command()
  {
    const auto robot = Point{pose_.x, pose_.y};
    update_progress(robot);

    const double target_distance = static_cast<double>(target_laps_) * lap_length_m_;
    const double remaining = std::max(0.0, target_distance - total_progress_m_);
    if (remaining <= goal_tolerance_m_) {
      state_ = RunnerState::Done;
      publish_stop();
      status_reason_ = "goal_reached";
      return;
    }

    const Point target = point_at_progress(total_progress_m_ + lookahead_distance_m_);
    const double dx = target.x - pose_.x;
    const double dy = target.y - pose_.y;
    const double target_yaw = std::atan2(dy, dx);
    const double yaw_error = normalize_angle(target_yaw - pose_.yaw);
    const double target_w = std::clamp(k_w_ * yaw_error, -w_max_rad_s_, w_max_rad_s_);

    double w = target_w;
    last_w_error_ = 0.0;
    const bool use_rate_loop = use_angular_velocity_feedback_ && angular_velocity_is_fresh();
    if (use_rate_loop) {
      last_w_error_ = target_w - measured_w_rad_s_;
      w = std::clamp(
        k_w_rate_ * last_w_error_,
        -w_max_rad_s_, w_max_rad_s_);
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = speed_m_s_;
    twist.angular.z = w;
    cmd_pub_->publish(twist);

    last_target_ = target;
    last_yaw_error_ = yaw_error;
    last_target_w_ = target_w;
    last_cmd_w_ = w;
    status_reason_ = use_rate_loop ? "tracking_rate_feedback" : "tracking_angle_only";
  }

  void update_progress(const Point & robot)
  {
    const std::size_t nearest = nearest_path_index(robot);
    double measured_progress = static_cast<double>(completed_laps_) * lap_length_m_ +
      cumulative_s_[nearest];

    const double lap_progress = measured_progress -
      static_cast<double>(completed_laps_) * lap_length_m_;
    if (lap_progress < 0.25 * lap_length_m_ && previous_lap_progress_m_ > 0.75 * lap_length_m_) {
      ++completed_laps_;
      measured_progress = static_cast<double>(completed_laps_) * lap_length_m_ + lap_progress;
      RCLCPP_INFO(get_logger(), "Completed lap %d/%d", completed_laps_, target_laps_);
    }

    if (measured_progress + waypoint_tolerance_m_ >= total_progress_m_) {
      total_progress_m_ = std::max(total_progress_m_, measured_progress);
      current_index_ = nearest;
    }
    previous_lap_progress_m_ = cumulative_s_[nearest];
  }

  std::size_t nearest_path_index(const Point & robot) const
  {
    std::size_t begin = current_index_;
    std::size_t end = path_.size();
    if (path_.size() > 1) {
      end = std::min(path_.size(), current_index_ + 80);
    }

    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = begin;
    for (std::size_t i = begin; i < end; ++i) {
      const double d = distance(robot, path_[i]);
      if (d < best_distance) {
        best_distance = d;
        best_index = i;
      }
    }

    if (end == path_.size()) {
      for (std::size_t i = 0; i < std::min<std::size_t>(80, path_.size()); ++i) {
        const double d = distance(robot, path_[i]);
        if (d < best_distance) {
          best_distance = d;
          best_index = i;
        }
      }
    }
    return best_index;
  }

  Point point_at_progress(double progress) const
  {
    if (path_.empty()) {
      return Point{};
    }
    const double lap_progress = std::fmod(std::max(0.0, progress), lap_length_m_);
    auto upper = std::lower_bound(cumulative_s_.begin(), cumulative_s_.end(), lap_progress);
    if (upper == cumulative_s_.end()) {
      return path_.back();
    }
    const auto index = static_cast<std::size_t>(upper - cumulative_s_.begin());
    return path_[index];
  }

  void publish_stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void reset_progress()
  {
    current_index_ = 0;
    total_progress_m_ = 0.0;
    completed_laps_ = 0;
    previous_lap_progress_m_ = 0.0;
    last_yaw_error_ = 0.0;
    last_target_w_ = 0.0;
    last_cmd_w_ = 0.0;
    last_w_error_ = 0.0;
    last_target_ = path_.empty() ? Point{} : path_.front();
    status_reason_ = "reset";
  }

  double remaining_distance_m() const
  {
    const double target_distance = static_cast<double>(target_laps_) * lap_length_m_;
    return std::max(0.0, target_distance - total_progress_m_);
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "state=" << state_name(state_)
           << " lap=" << completed_laps_ << "/" << target_laps_
           << " index=" << current_index_ << "/" << path_.size()
           << " progress=" << total_progress_m_
           << " speed=" << speed_m_s_
           << " target_w=" << last_target_w_
           << " measured_w=" << measured_w_rad_s_
           << " w_error=" << last_w_error_
           << " cmd_w=" << last_cmd_w_
           << " yaw_error=" << last_yaw_error_
           << " target=(" << last_target_.x << "," << last_target_.y << ")"
           << " pose=(" << pose_.x << "," << pose_.y << "," << pose_.yaw << ")"
           << " remaining=" << remaining_distance_m()
           << " reason=" << status_reason_;
    msg.data = stream.str();
    status_pub_->publish(msg);
  }

  double straight_length_m_{1.5};
  double radius_m_{0.75};
  double path_spacing_m_{0.03};
  double speed_m_s_{0.02};
  int target_laps_{1};
  double lookahead_distance_m_{0.25};
  double waypoint_tolerance_m_{0.08};
  double goal_tolerance_m_{0.10};
  double k_w_{1.8};
  double k_w_rate_{1.0};
  double w_max_rad_s_{0.6};
  double pose_timeout_s_{0.5};
  double angular_velocity_timeout_s_{0.35};
  bool use_angular_velocity_feedback_{true};
  bool use_start_pose_as_origin_{true};
  double lap_length_m_{};
  double origin_x_{0.0};
  double origin_y_{0.0};

  RunnerState state_{RunnerState::Idle};
  Pose2D pose_{};
  bool has_pose_{false};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  double measured_w_rad_s_{0.0};
  bool has_angular_velocity_{false};
  rclcpp::Time last_angular_velocity_time_{0, 0, RCL_ROS_TIME};
  std::vector<Point> path_;
  std::vector<double> cumulative_s_;
  std::size_t current_index_{0};
  double total_progress_m_{0.0};
  int completed_laps_{0};
  double previous_lap_progress_m_{0.0};
  Point last_target_{};
  double last_yaw_error_{0.0};
  double last_target_w_{0.0};
  double last_cmd_w_{0.0};
  double last_w_error_{0.0};
  std::string status_reason_{"idle"};

  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr laps_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
