#include "path_planner/geometry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
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

std::vector<double> yaml_double_vector(const YAML::Node & node)
{
  std::vector<double> values;
  for (const auto & item : node) {
    values.push_back(item.as<double>());
  }
  return values;
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
    const auto obstacle_file = declare_parameter<std::string>("obstacle_file", "");
    robot_radius_m_ = declare_parameter<double>("robot_radius_m", 0.18);
    safety_margin_m_ = declare_parameter<double>("safety_margin_m", 0.08);
    min_waypoint_spacing_m_ = declare_parameter<double>("min_waypoint_spacing_m", 0.10);
    max_plan_length_m_ = declare_parameter<double>("max_plan_length_m", 20.0);

    keepout_zones_ = load_keepout_zones(obstacle_file);
    const double inflate_radius = robot_radius_m_ + safety_margin_m_;
    inflated_zones_ = path_planner::inflate_zones(keepout_zones_, inflate_radius);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PathPlannerNode::on_goal_pose, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/path_planner/status", 10);

    RCLCPP_INFO(
      get_logger(), "Loaded %zu keepout zones; inflate_radius=%.3f m",
      keepout_zones_.size(), inflate_radius);
  }

private:
  std::vector<path_planner::KeepoutZone> load_keepout_zones(
    const std::string & obstacle_file) const
  {
    const auto config_path = resolve_obstacle_file(obstacle_file);
    if (!std::filesystem::exists(config_path)) {
      RCLCPP_WARN(
        get_logger(), "Keepout zone file not found: %s; using no zones",
        config_path.c_str());
      return {};
    }

    const YAML::Node data = YAML::LoadFile(config_path);
    const auto frame_id = data["frame_id"] ?
      data["frame_id"].as<std::string>() : global_frame_id_;
    if (frame_id != global_frame_id_) {
      throw path_planner::PlanningError(
        "Keepout zone frame \"" + frame_id + "\" does not match \"" +
        global_frame_id_ + "\"");
    }

    std::vector<path_planner::KeepoutZone> zones;
    const YAML::Node raw_zones = data["keepout_zones"];
    if (!raw_zones) {
      return zones;
    }

    for (const auto & raw_zone : raw_zones) {
      const auto name = raw_zone["name"] ?
        raw_zone["name"].as<std::string>() :
        "zone_" + std::to_string(zones.size());
      const auto type = raw_zone["type"].as<std::string>();
      if (type == "rectangle") {
        zones.push_back(path_planner::rectangle_zone(
          name,
          yaml_double_vector(raw_zone["center"]),
          yaml_double_vector(raw_zone["size"])));
      } else if (type == "polygon") {
        std::vector<std::vector<double>> points;
        for (const auto & point : raw_zone["points"]) {
          points.push_back(yaml_double_vector(point));
        }
        zones.push_back(path_planner::polygon_zone(name, points));
      } else {
        throw path_planner::PlanningError(
          "Unsupported keepout zone type \"" + type + "\" for \"" + name + "\"");
      }
    }
    return zones;
  }

  std::string resolve_obstacle_file(const std::string & obstacle_file) const
  {
    std::filesystem::path path = obstacle_file.empty() ?
      std::filesystem::path("keepout_zones.yaml") :
      std::filesystem::path(obstacle_file);
    if (path.is_absolute()) {
      return path.string();
    }

    const auto share_dir = ament_index_cpp::get_package_share_directory("path_planner");
    return (std::filesystem::path(share_dir) / "config" / path.filename()).string();
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
        start, goal, inflated_zones_,
        min_waypoint_spacing_m_, max_plan_length_m_);
    } catch (const path_planner::PlanningError & exc) {
      publish_status(std::string("failed reason=\"") + exc.what() + "\"");
      RCLCPP_WARN(get_logger(), "Planning failed: %s", exc.what());
      publish_empty_path();
      return;
    }

    path_pub_->publish(build_path_msg(points, goal_yaw));
    const double length = path_planner::path_length(points);
    std::ostringstream status;
    status << "planned points=" << points.size() << " length=" << length;
    publish_status(status.str());
    RCLCPP_INFO(
      get_logger(), "Planned path with %zu points, length=%.3f m",
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

  std::string global_frame_id_;
  std::string base_frame_id_;
  double robot_radius_m_{};
  double safety_margin_m_{};
  double min_waypoint_spacing_m_{};
  double max_plan_length_m_{};

  std::vector<path_planner::KeepoutZone> keepout_zones_;
  std::vector<path_planner::KeepoutZone> inflated_zones_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
