#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/car/odom/carto");
    odom_child_frame_id_ = declare_parameter<std::string>("odom_child_frame_id", base_frame_id_);
    yaw_offset_rad_ = declare_parameter<double>("yaw_offset_rad", 0.0);
    publish_odom_ = declare_parameter<bool>("publish_odom", true);
    velocity_filter_tau_s_ = declare_parameter<double>("velocity_filter_tau_s", 0.25);
    max_velocity_dt_s_ = declare_parameter<double>("max_velocity_dt_s", 0.5);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);

    velocity_filter_tau_s_ = std::max(0.0, velocity_filter_tau_s_);
    max_velocity_dt_s_ = std::max(0.02, max_velocity_dt_s_);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    }

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
    if (publish_odom_) {
      RCLCPP_INFO(
        get_logger(),
        "Publishing filtered differential velocity on %s, child_frame_id=%s, tau=%.3f s",
        odom_topic_.c_str(), odom_child_frame_id_.c_str(), velocity_filter_tau_s_);
    }
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
    publish_odom(pose);
  }

  void publish_odom(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!publish_odom_) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.child_frame_id = odom_child_frame_id_;
    odom.pose.pose = pose.pose;

    const rclcpp::Time stamp(pose.header.stamp);
    if (has_previous_pose_) {
      const double dt = (stamp - previous_stamp_).seconds();
      if (dt > 1e-4 && dt <= max_velocity_dt_s_) {
        const double vx_map = (pose.pose.position.x - previous_pose_.pose.position.x) / dt;
        const double vy_map = (pose.pose.position.y - previous_pose_.pose.position.y) / dt;
        const double yaw = quaternion_to_yaw(pose.pose.orientation);
        const double previous_yaw = quaternion_to_yaw(previous_pose_.pose.orientation);
        const double wz = normalize_angle(yaw - previous_yaw) / dt;

        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const double vx_body = c * vx_map + s * vy_map;
        const double vy_body = -s * vx_map + c * vy_map;
        const double alpha = velocity_filter_tau_s_ <= 0.0 ?
          1.0 : dt / (velocity_filter_tau_s_ + dt);

        filtered_vx_m_s_ += alpha * (vx_body - filtered_vx_m_s_);
        filtered_vy_m_s_ += alpha * (vy_body - filtered_vy_m_s_);
        filtered_wz_rad_s_ += alpha * (wz - filtered_wz_rad_s_);
        velocity_filter_initialized_ = true;
      } else {
        reset_velocity_filter();
      }
    }

    odom.twist.twist.linear.x = velocity_filter_initialized_ ? filtered_vx_m_s_ : 0.0;
    odom.twist.twist.linear.y = velocity_filter_initialized_ ? filtered_vy_m_s_ : 0.0;
    odom.twist.twist.angular.z = velocity_filter_initialized_ ? filtered_wz_rad_s_ : 0.0;
    odom.pose.covariance[0] = 0.02;
    odom.pose.covariance[7] = 0.02;
    odom.pose.covariance[35] = 0.05;
    odom.twist.covariance[0] = 0.05;
    odom.twist.covariance[7] = 0.05;
    odom.twist.covariance[35] = 0.10;

    odom_pub_->publish(odom);

    previous_pose_ = pose;
    previous_stamp_ = stamp;
    has_previous_pose_ = true;
  }

  void reset_velocity_filter()
  {
    filtered_vx_m_s_ = 0.0;
    filtered_vy_m_s_ = 0.0;
    filtered_wz_rad_s_ = 0.0;
    velocity_filter_initialized_ = false;
  }

  std::string global_frame_id_;
  std::string base_frame_id_;
  std::string output_frame_id_;
  std::string pose_topic_;
  std::string odom_topic_;
  std::string odom_child_frame_id_;
  double yaw_offset_rad_{};
  bool publish_odom_{true};
  double velocity_filter_tau_s_{0.25};
  double max_velocity_dt_s_{0.5};
  double filtered_vx_m_s_{0.0};
  double filtered_vy_m_s_{0.0};
  double filtered_wz_rad_s_{0.0};
  bool velocity_filter_initialized_{false};
  bool has_previous_pose_{false};
  geometry_msgs::msg::PoseStamped previous_pose_{};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartoPosePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
