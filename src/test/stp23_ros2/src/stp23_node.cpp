#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "stp23_ros2/cmd_interface_linux.h"
#include "stp23_ros2/lipkg.h"
#include "stp23_ros2/msg/stp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace {
constexpr double kPi = 3.14159265358979323846;

std::vector<std::string> SerialCandidatesFromDir(const fs::path &dir) {
  std::vector<std::string> candidates;
  std::error_code ec;
  if (!fs::exists(dir, ec)) {
    return candidates;
  }

  for (const auto &entry : fs::directory_iterator(dir, ec)) {
    if (ec) {
      break;
    }
    candidates.push_back(entry.path().string());
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates;
}

std::vector<std::string> SerialCandidatesWithPrefix(const fs::path &dir,
                                                    const std::string &prefix) {
  std::vector<std::string> candidates;
  std::error_code ec;
  if (!fs::exists(dir, ec)) {
    return candidates;
  }

  for (const auto &entry : fs::directory_iterator(dir, ec)) {
    if (ec) {
      break;
    }
    const auto name = entry.path().filename().string();
    if (name.rfind(prefix, 0) == 0) {
      candidates.push_back(entry.path().string());
    }
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates;
}
}  // namespace

class Stp23DriverNode : public rclcpp::Node {
 public:
  Stp23DriverNode() : Node("stp23_node") {
    product_name_ = declare_parameter<std::string>("product_name", "LDLiDAR_STP23");
    port_name_ = declare_parameter<std::string>("port_name", "auto");
    baudrate_ = declare_parameter<int>("baudrate", 921600);
    hardware_flow_control_ = declare_parameter<bool>("hardware_flow_control", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar_frame");
    point_topic_ = declare_parameter<std::string>("point_topic", "stp23");
    distance_topic_ = declare_parameter<std::string>("distance_topic", "/ldlidar_distance");
    range_topic_ = declare_parameter<std::string>("range_topic", "/stp23/range");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "scan");
    publish_scan_ = declare_parameter<bool>("publish_scan", true);
    publish_point_ = declare_parameter<bool>("publish_point", true);
    publish_range_ = declare_parameter<bool>("publish_range", true);
    scan_angle_increment_deg_ =
        declare_parameter<double>("scan_angle_increment_deg", 1.0);
    range_min_ = declare_parameter<double>("range_min", 0.02);
    range_max_ = declare_parameter<double>("range_max", 25.0);
    log_points_ = declare_parameter<bool>("log_points", false);

    if (product_name_ != "LDLiDAR_STP23") {
      throw std::runtime_error("unsupported product_name: " + product_name_);
    }
    if (scan_angle_increment_deg_ <= 0.0 || scan_angle_increment_deg_ > 90.0) {
      throw std::runtime_error("scan_angle_increment_deg must be in (0, 90]");
    }

    lidar_ = std::make_unique<LiPkg>(frame_id_, LDVersion::STP23);
    cmd_port_.SetReadCallback([this](const char *bytes, size_t len) {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      if (lidar_->Parse(reinterpret_cast<const uint8_t *>(bytes),
                        static_cast<long>(len))) {
        lidar_->AssemblePacket();
      }
    });

    const auto resolved_port = ResolvePort(port_name_);
    if (resolved_port.empty()) {
      throw std::runtime_error("no serial device found for port_name=" + port_name_);
    }

    if (!cmd_port_.Open(resolved_port, baudrate_, hardware_flow_control_)) {
      throw std::runtime_error("failed to open serial device: " + resolved_port);
    }

    point_pub_ =
        create_publisher<stp23_ros2::msg::STP>(point_topic_, rclcpp::SensorDataQoS());
    distance_pub_ =
        create_publisher<std_msgs::msg::Float32>(distance_topic_, rclcpp::SensorDataQoS());
    range_pub_ =
        create_publisher<sensor_msgs::msg::Range>(range_topic_, rclcpp::SensorDataQoS());
    scan_pub_ =
        create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(5ms, std::bind(&Stp23DriverNode::PublishTimer, this));

    RCLCPP_INFO(get_logger(),
                "STP23 driver started: port=%s baudrate=%d frame_id=%s",
                resolved_port.c_str(), baudrate_, frame_id_.c_str());
  }

 private:
  std::string ResolvePort(const std::string &requested_port) {
    if (!requested_port.empty() && requested_port != "auto") {
      return requested_port;
    }

    std::vector<std::string> candidates;
    const auto by_id = SerialCandidatesFromDir("/dev/serial/by-id");
    candidates.insert(candidates.end(), by_id.begin(), by_id.end());

    const auto tty_usb = SerialCandidatesWithPrefix("/dev", "ttyUSB");
    candidates.insert(candidates.end(), tty_usb.begin(), tty_usb.end());

    const auto tty_acm = SerialCandidatesWithPrefix("/dev", "ttyACM");
    candidates.insert(candidates.end(), tty_acm.begin(), tty_acm.end());

    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    if (candidates.empty()) {
      return "";
    }

    RCLCPP_INFO(get_logger(), "auto selected serial port candidate: %s",
                candidates.front().c_str());
    return candidates.front();
  }

  void PublishTimer() {
    std::array<PointData, POINT_PER_PACK> package_points;
    Points2D frame_points;
    bool has_package = false;
    bool has_frame = false;

    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      if (lidar_->IsPkgReady()) {
        package_points = lidar_->GetPkgData();
        has_package = true;
      }
      if (lidar_->IsFrameReady()) {
        frame_points = lidar_->GetData();
        lidar_->ResetFrameReady();
        has_frame = true;
      }
    }

    const auto stamp = now();
    if (has_package && publish_point_) {
      for (const auto &point : package_points) {
        PublishPoint(point, stamp);
        PublishRange(point, stamp);
      }
    }
    if (has_frame && publish_scan_) {
      PublishScan(frame_points, stamp);
    }
  }

  void PublishPoint(const PointData &point, const rclcpp::Time &stamp) {
    stp23_ros2::msg::STP point_msg;
    point_msg.header.stamp = stamp;
    point_msg.header.frame_id = frame_id_;
    point_msg.distance = static_cast<float>(point.distance) / 1000.0f;
    point_msg.intensity = static_cast<int32_t>(point.intensity);
    point_pub_->publish(point_msg);

    std_msgs::msg::Float32 distance_msg;
    distance_msg.data = point_msg.distance;
    distance_pub_->publish(distance_msg);

    if (log_points_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "distance=%.4f m intensity=%d angle=%.2f deg",
                           point_msg.distance, point_msg.intensity, point.angle);
    }
  }

  void PublishRange(const PointData &point, const rclcpp::Time &stamp) {
    if (!publish_range_) {
      return;
    }

    sensor_msgs::msg::Range range_msg;
    const auto range_m = static_cast<float>(point.distance) / 1000.0f;
    if (range_m < range_min_ || range_m > range_max_) {
      return;
    }

    range_msg.header.stamp = stamp;
    range_msg.header.frame_id = frame_id_;
    range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range_msg.field_of_view = 0.05f;
    range_msg.min_range = static_cast<float>(range_min_);
    range_msg.max_range = static_cast<float>(range_max_);
    range_msg.range = range_m;
    range_pub_->publish(range_msg);
  }

  void PublishScan(const Points2D &points, const rclcpp::Time &stamp) {
    if (points.empty()) {
      return;
    }

    const double increment_rad = scan_angle_increment_deg_ * kPi / 180.0;
    const size_t bin_count =
        static_cast<size_t>(std::ceil(2.0 * kPi / increment_rad));

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = frame_id_;
    scan.angle_min = 0.0f;
    scan.angle_increment = static_cast<float>(increment_rad);
    scan.angle_max = static_cast<float>((bin_count - 1) * increment_rad);
    scan.time_increment = 0.0f;
    scan.scan_time = 0.0f;
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges.assign(bin_count, std::numeric_limits<float>::infinity());
    scan.intensities.assign(bin_count, 0.0f);

    for (const auto &point : points) {
      const float range = static_cast<float>(point.distance) / 1000.0f;
      if (range < scan.range_min || range > scan.range_max) {
        continue;
      }

      const double angle = point.angle < 0.0 ? point.angle + 360.0 : point.angle;
      size_t index =
          static_cast<size_t>(std::llround(angle / scan_angle_increment_deg_));
      if (index >= bin_count) {
        index = 0;
      }

      if (!std::isfinite(scan.ranges[index]) || range < scan.ranges[index]) {
        scan.ranges[index] = range;
        scan.intensities[index] = static_cast<float>(point.intensity);
      }
    }

    scan_pub_->publish(scan);
  }

  std::string product_name_;
  std::string port_name_;
  int baudrate_;
  bool hardware_flow_control_;
  std::string frame_id_;
  std::string point_topic_;
  std::string distance_topic_;
  std::string range_topic_;
  std::string scan_topic_;
  bool publish_scan_;
  bool publish_point_;
  bool publish_range_;
  double scan_angle_increment_deg_;
  double range_min_;
  double range_max_;
  bool log_points_;

  std::unique_ptr<LiPkg> lidar_;
  CmdInterfaceLinux cmd_port_;
  std::mutex lidar_mutex_;

  rclcpp::Publisher<stp23_ros2::msg::STP>::SharedPtr point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Stp23DriverNode>());
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("stp23_node"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
