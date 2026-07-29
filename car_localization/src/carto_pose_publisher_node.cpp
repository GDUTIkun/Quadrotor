#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

namespace
{

constexpr double kHalfPi = 1.57079632679489661923;

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
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

class CartoPosePublisherNode : public rclcpp::Node
{
public:
  CartoPosePublisherNode()
  : Node("carto_pose_publisher_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_id_ = declare_parameter<std::string>("global_frame_id", "map");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "car_map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/car/pose");
    yaw_offset_rad_ = declare_parameter<double>("yaw_offset_rad", 0.0);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);

    const double period_s = 1.0 / std::clamp(publish_rate_hz, 1.0, 200.0);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&CartoPosePublisherNode::publish_pose, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing %s->%s as %s on %s with axes: +x right, +y forward, +z up",
      global_frame_id_.c_str(), base_frame_id_.c_str(),
      output_frame_id_.c_str(), pose_topic_.c_str());
  }

private:
  void publish_pose()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_id_, base_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Cannot lookup %s->%s: %s",
        global_frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
      return;
    }

    const auto & t = transform.transform.translation;
    const double carto_yaw = quaternion_to_yaw(transform.transform.rotation);
    const double output_yaw = normalize_angle(carto_yaw + kHalfPi + yaw_offset_rad_);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = output_frame_id_;
    pose.pose.position.x = -t.y;
    pose.pose.position.y = t.x;
    pose.pose.position.z = t.z;
    pose.pose.orientation = yaw_to_quaternion(output_yaw);

    pose_pub_->publish(pose);
  }

  std::string global_frame_id_;
  std::string base_frame_id_;
  std::string output_frame_id_;
  std::string pose_topic_;
  double yaw_offset_rad_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartoPosePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
