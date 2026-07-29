#include "path_controller/control.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class PathControllerNode : public rclcpp::Node
{
public:
  PathControllerNode()
  : Node("path_controller_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_id_ = declare_parameter<std::string>("global_frame_id", "car_carto_map");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "car_base_link");
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 20.0);
    config_.xy_tolerance_m = declare_parameter<double>("xy_tolerance_m", 0.05);
    config_.yaw_tolerance_rad = declare_parameter<double>("yaw_tolerance_rad", 0.0872665);
    config_.heading_tolerance_rad =
      declare_parameter<double>("heading_tolerance_rad", 0.15);
    config_.lookahead_distance_m =
      declare_parameter<double>("lookahead_distance_m", 0.35);
    config_.goal_slowdown_radius_m =
      declare_parameter<double>("goal_slowdown_radius_m", 0.40);
    config_.k_w = declare_parameter<double>("k_w", 1.8);
    config_.target_speed_m_s = declare_parameter<double>("target_speed_m_s", 0.05);
    config_.v_max_m_s = declare_parameter<double>("v_max_m_s", 0.05);
    config_.w_max_rad_s = declare_parameter<double>("w_max_rad_s", 0.5);
    config_.v_min_m_s = declare_parameter<double>("v_min_m_s", 0.0);
    config_.w_min_rad_s = declare_parameter<double>("w_min_rad_s", 0.08);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&PathControllerNode::on_path, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/path_controller/status", 10);

    const double period_s = std::max(0.01, 1.0 / control_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&PathControllerNode::control_step, this));
  }

  ~PathControllerNode() override
  {
    publish_stop();
  }

private:
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    const auto frame_id =
      msg->header.frame_id.empty() ? global_frame_id_ : msg->header.frame_id;
    if (frame_id != global_frame_id_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring path in frame \"%s\"; expected \"%s\"",
        frame_id.c_str(), global_frame_id_.c_str());
      clear_path();
      return;
    }

    if (msg->poses.empty()) {
      clear_path();
      publish_status("idle reason=empty_path");
      return;
    }

    path_.clear();
    path_.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      path_.push_back(path_controller::PathPoint{
        pose.pose.position.x,
        pose.pose.position.y,
        quaternion_to_yaw(pose.pose.orientation)});
    }
    closed_path_ =
      path_.size() > 2 &&
      std::hypot(path_.front().x - path_.back().x, path_.front().y - path_.back().y) <=
      config_.xy_tolerance_m;
    closed_path_departed_ = !closed_path_;
    start_x_ = path_.front().x;
    start_y_ = path_.front().y;
    last_phase_.clear();
    RCLCPP_INFO(
      get_logger(), "Accepted path with %zu points, closed=%s",
      path_.size(), closed_path_ ? "true" : "false");
  }

  void control_step()
  {
    if (path_.empty()) {
      publish_stop();
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_id_, base_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Cannot lookup %s->%s: %s",
        global_frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
      publish_stop();
      return;
    }

    const path_controller::RobotPose robot{
      transform.transform.translation.x,
      transform.transform.translation.y,
      quaternion_to_yaw(transform.transform.rotation)};
    if (closed_path_ && !closed_path_departed_) {
      const double departure_distance = std::hypot(robot.x - start_x_, robot.y - start_y_);
      closed_path_departed_ = departure_distance > 2.0 * config_.xy_tolerance_m;
    }
    const auto result = path_controller::compute_command(
      robot, path_, config_, !closed_path_ || closed_path_departed_);

    if (result.phase != last_phase_) {
      last_phase_ = result.phase;
      RCLCPP_INFO(
        get_logger(), "Path controller phase=%s, distance=%.3f, yaw_error=%.3f",
        result.phase.c_str(), result.distance_error, result.yaw_error);
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = result.v;
    twist.angular.z = result.w;
    cmd_pub_->publish(twist);

    std::ostringstream status;
    status << "phase=" << result.phase
           << " distance=" << result.distance_error
           << " yaw_error=" << result.yaw_error
           << " v=" << result.v
           << " w=" << result.w
           << " lookahead=(" << result.lookahead_x << "," << result.lookahead_y << ")";
    publish_status(status.str());

    if (result.reached) {
      clear_path();
    }
  }

  void clear_path()
  {
    path_.clear();
    closed_path_ = false;
    closed_path_departed_ = false;
    last_phase_.clear();
    publish_stop();
  }

  void publish_stop()
  {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  std::string global_frame_id_;
  std::string base_frame_id_;
  path_controller::ControllerConfig config_;
  std::vector<path_controller::PathPoint> path_;
  std::string last_phase_;
  bool closed_path_{};
  bool closed_path_departed_{};
  double start_x_{};
  double start_y_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathControllerNode>());
  rclcpp::shutdown();
  return 0;
}
