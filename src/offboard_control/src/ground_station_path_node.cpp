#include <algorithm>
#include <cctype>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
constexpr int kGridWidth = 9;
constexpr int kGridHeight = 7;

struct GridPoint {
  int a{};
  int b{};
};

std::string normalizeToken(std::string token)
{
  token.erase(
    std::remove_if(token.begin(), token.end(), [](unsigned char ch) {return std::isspace(ch);}),
    token.end());
  std::transform(token.begin(), token.end(), token.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return token;
}

GridPoint parseGridPoint(const std::string & raw_token)
{
  const std::string token = normalizeToken(raw_token);
  if (token.size() != 4U || token[0] != 'A' || token[2] != 'B' ||
    !std::isdigit(static_cast<unsigned char>(token[1])) ||
    !std::isdigit(static_cast<unsigned char>(token[3])))
  {
    throw std::invalid_argument("非法点位: " + raw_token + "，正确格式如 A4B3");
  }

  const int a = token[1] - '0';
  const int b = token[3] - '0';
  if (a < 1 || a > kGridWidth || b < 1 || b > kGridHeight) {
    throw std::out_of_range("点位超出 9x7 范围: " + token);
  }
  return GridPoint{a, b};
}

std::string gridCode(const GridPoint & point)
{
  return "A" + std::to_string(point.a) + "B" + std::to_string(point.b);
}

std::vector<GridPoint> parsePathText(const std::string & text)
{
  std::vector<GridPoint> points;
  const std::regex token_regex(R"([Aa][1-9][Bb][1-7])");
  for (std::sregex_iterator it(text.begin(), text.end(), token_regex), end; it != end; ++it) {
    points.push_back(parseGridPoint(it->str()));
  }
  if (points.empty()) {
    throw std::invalid_argument("消息中没有找到路径点，示例: A9B1 -> A8B1 -> A7B1");
  }
  return points;
}

std::string routeText(const std::vector<GridPoint> & points)
{
  std::ostringstream text;
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (index != 0U) {
      text << " -> ";
    }
    text << gridCode(points[index]);
  }
  return text.str();
}
}  // namespace

class GroundStationPathNode final : public rclcpp::Node {
public:
  GroundStationPathNode()
  : Node("ground_station_path_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "ground_station_path_grid");
    output_topic_ = declare_parameter<std::string>("output_topic", "ground_station_flight_path");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    flight_z_ = declare_parameter<double>("flight_z", 0.9);
    cell_size_ = declare_parameter<double>("cell_size", 0.4);
    qos_depth_ = declare_parameter<int>("qos_depth", 10);

    if (input_topic_.empty()) {
      throw std::invalid_argument("参数 input_topic 不能为空");
    }
    if (output_topic_.empty()) {
      throw std::invalid_argument("参数 output_topic 不能为空");
    }
    if (cell_size_ <= 0.0) {
      throw std::invalid_argument("参数 cell_size 必须大于 0");
    }
    if (qos_depth_ <= 0) {
      throw std::invalid_argument("参数 qos_depth 必须大于 0");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(qos_depth_))).reliable();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(output_topic_, qos);
    path_subscription_ = create_subscription<std_msgs::msg::String>(
      input_topic_, qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        onPathMessage(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "等待地面站路径话题 [%s]，格式示例: A9B1 -> A8B1；cell_size=%.3f m，输出 [%s]",
      input_topic_.c_str(), cell_size_, output_topic_.c_str());
  }

private:
  geometry_msgs::msg::PoseStamped toPose(
    const GridPoint & point, const std_msgs::msg::Header & header) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = origin_x_ + static_cast<double>(point.a - 1) * cell_size_;
    pose.pose.position.y = origin_y_ + static_cast<double>(point.b - 1) * cell_size_;
    pose.pose.position.z = flight_z_;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  void onPathMessage(const std_msgs::msg::String & msg)
  {
    std::vector<GridPoint> grid_points;
    try {
      grid_points = parsePathText(msg.data);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "地面站路径解析失败: %s；原始消息: [%s]", error.what(), msg.data.c_str());
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    path.poses.reserve(grid_points.size());
    for (const GridPoint & point : grid_points) {
      path.poses.push_back(toPose(point, path.header));
    }
    path_publisher_->publish(path);

    RCLCPP_INFO(
      get_logger(), "收到 %zu 个路径点: %s",
      grid_points.size(), routeText(grid_points).c_str());

    std::ostringstream converted;
    converted << std::fixed << std::setprecision(3);
    for (std::size_t index = 0; index < grid_points.size(); ++index) {
      const auto & pose = path.poses[index].pose.position;
      converted << '\n'
                << std::setw(2) << index + 1U << ". "
                << gridCode(grid_points[index]) << " -> "
                << "x=" << pose.x << " m, "
                << "y=" << pose.y << " m, "
                << "z=" << pose.z << " m";
    }
    RCLCPP_INFO(get_logger(), "真实飞行路径点:%s", converted.str().c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int qos_depth_{};
  double origin_x_{};
  double origin_y_{};
  double flight_z_{};
  double cell_size_{};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GroundStationPathNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ground_station_path_node"), "节点启动失败: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
