#include "path_planner/geometry.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
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

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode()
  : Node("path_planner_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_id_ = declare_parameter<std::string>("global_frame_id", "car_carto_map");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "car_base_link");
    plan_mode_ = declare_parameter<std::string>("plan_mode", "goal");
    path_spacing_m_ = declare_parameter<double>("path_spacing_m", 0.02);
    max_plan_length_m_ = declare_parameter<double>("max_plan_length_m", 20.0);
    test_line_length_m_ = declare_parameter<double>("test_line_length_m", 0.5);
    test_arc_radius_m_ = declare_parameter<double>("test_arc_radius_m", 0.3);
    test_arc_angle_rad_ = declare_parameter<double>("test_arc_angle_rad", M_PI_2);
    racetrack_straight_length_m_ =
      declare_parameter<double>("racetrack_straight_length_m", 1.5);
    racetrack_radius_m_ = declare_parameter<double>("racetrack_radius_m", 0.75);
    racetrack_turn_right_ = declare_parameter<bool>("racetrack_turn_right", true);
    test_publish_rate_hz_ = declare_parameter<double>("test_publish_rate_hz", 2.0);
    test_publish_repeats_ = declare_parameter<int>("test_publish_repeats", 20);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PathPlannerNode::on_goal_pose, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/path_planner/status", 10);

    RCLCPP_INFO(
      get_logger(),
      "Path planner mode=%s, spacing=%.3f m, no obstacle avoidance",
      plan_mode_.c_str(), path_spacing_m_);

    if (plan_mode_ != "goal") {
      const double period_s = std::max(0.1, 1.0 / std::max(0.1, test_publish_rate_hz_));
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(period_s)),
        std::bind(&PathPlannerNode::publish_test_path, this));
    }
  }

private:
  void publish_test_path()
  {
    if (!test_path_ready_ && !build_test_path()) {
      return;
    }

    path_pub_->publish(test_path_msg_);
    std::ostringstream status;
    status << "planned mode=" << plan_mode_
           << " points=" << test_path_msg_.poses.size()
           << " length=" << test_path_length_
           << " publish=" << (test_publish_count_ + 1)
           << "/" << test_publish_repeats_;
    publish_status(status.str());
    RCLCPP_INFO(
      get_logger(), "Published %s test path with %zu points, length=%.3f m (%d/%d)",
      plan_mode_.c_str(), test_path_msg_.poses.size(), test_path_length_,
      test_publish_count_ + 1, test_publish_repeats_);

    ++test_publish_count_;
    if (timer_ && test_publish_count_ >= test_publish_repeats_) {
      timer_->cancel();
    }
  }

  bool build_test_path()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_id_, base_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exc) {
      publish_status("failed reason=tf_unavailable");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Cannot lookup %s->%s: %s",
        global_frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
      publish_empty_path();
      return false;
    }

    const path_planner::Point start{
      transform.transform.translation.x,
      transform.transform.translation.y};
    const double start_yaw = quaternion_to_yaw(transform.transform.rotation);

    std::vector<path_planner::Point> points;
    double goal_yaw = start_yaw;
    try {
      if (plan_mode_ == "line" || plan_mode_ == "straight") {
        points = path_planner::make_straight_path(
          start, start_yaw, test_line_length_m_, path_spacing_m_);
      } else if (plan_mode_ == "arc") {
        points = path_planner::make_arc_path(
          start, start_yaw, test_arc_radius_m_, test_arc_angle_rad_, path_spacing_m_);
        goal_yaw = start_yaw + test_arc_angle_rad_;
      } else if (plan_mode_ == "racetrack") {
        points = path_planner::make_racetrack_path(
          start, start_yaw, racetrack_straight_length_m_,
          racetrack_radius_m_, path_spacing_m_, racetrack_turn_right_);
        goal_yaw = start_yaw;
      } else {
        publish_status("failed reason=unsupported_plan_mode mode=" + plan_mode_);
        RCLCPP_ERROR(get_logger(), "Unsupported plan_mode: %s", plan_mode_.c_str());
        publish_empty_path();
        if (timer_) {
          timer_->cancel();
        }
        return false;
      }
      check_plan_length(points);
    } catch (const path_planner::PlanningError & exc) {
      publish_status(std::string("failed reason=\"") + exc.what() + "\"");
      RCLCPP_WARN(get_logger(), "Planning failed: %s", exc.what());
      publish_empty_path();
      return false;
    }

    test_path_msg_ = build_path_msg(points, goal_yaw);
    test_path_length_ = path_planner::path_length(points);
    test_path_ready_ = true;
    return true;
  }

  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto frame_id =
      msg->header.frame_id.empty() ? global_frame_id_ : msg->header.frame_id;
    if (frame_id != global_frame_id_) {
      publish_status("failed frame=" + frame_id + " reason=unexpected_goal_frame");
      RCLCPP_WARN(
        get_logger(), "Ignoring goal in frame \"%s\"; expected \"%s\"",
        frame_id.c_str(), global_frame_id_.c_str());
      publish_empty_path();
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_id_, base_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exc) {
      publish_status("failed reason=tf_unavailable");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Cannot lookup %s->%s: %s",
        global_frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
      publish_empty_path();
      return;
    }

    const path_planner::Point start{
      transform.transform.translation.x,
      transform.transform.translation.y};
    const path_planner::Point goal{
      msg->pose.position.x,
      msg->pose.position.y};
    const double goal_yaw = quaternion_to_yaw(msg->pose.orientation);

    std::vector<path_planner::Point> points;
    try {
      points = path_planner::plan_path(
        start, goal, {}, path_spacing_m_, max_plan_length_m_);
    } catch (const path_planner::PlanningError & exc) {
      publish_status(std::string("failed reason=\"") + exc.what() + "\"");
      RCLCPP_WARN(get_logger(), "Planning failed: %s", exc.what());
      publish_empty_path();
      return;
    }

    path_pub_->publish(build_path_msg(points, goal_yaw));
    const double length = path_planner::path_length(points);
    std::ostringstream status;
    status << "planned mode=goal points=" << points.size() << " length=" << length;
    publish_status(status.str());
    RCLCPP_INFO(
      get_logger(), "Planned goal path with %zu points, length=%.3f m",
      points.size(), length);
  }

  nav_msgs::msg::Path build_path_msg(
    const std::vector<path_planner::Point> & points, double goal_yaw) const
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = global_frame_id_;

    for (std::size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = points[i].x;
      pose.pose.position.y = points[i].y;
      pose.pose.position.z = 0.0;
      if (i + 1 == points.size()) {
        pose.pose.orientation = yaw_to_quaternion(goal_yaw);
      } else {
        const auto & next = points[i + 1];
        pose.pose.orientation =
          yaw_to_quaternion(std::atan2(next.y - points[i].y, next.x - points[i].x));
      }
      msg.poses.push_back(pose);
    }
    return msg;
  }

  void publish_empty_path()
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = global_frame_id_;
    path_pub_->publish(msg);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  void check_plan_length(const std::vector<path_planner::Point> & points) const
  {
    const double length = path_planner::path_length(points);
    if (length > max_plan_length_m_) {
      throw path_planner::PlanningError(
        "Planned path is too long: " + std::to_string(length) + " m");
    }
  }

  std::string global_frame_id_;
  std::string base_frame_id_;
  std::string plan_mode_;
  double path_spacing_m_{};
  double max_plan_length_m_{};
  double test_line_length_m_{};
  double test_arc_radius_m_{};
  double test_arc_angle_rad_{};
  double racetrack_straight_length_m_{};
  double racetrack_radius_m_{};
  bool racetrack_turn_right_{};
  double test_publish_rate_hz_{};
  int test_publish_repeats_{};
  bool test_path_ready_{false};
  int test_publish_count_{0};
  double test_path_length_{0.0};
  nav_msgs::msg::Path test_path_msg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
