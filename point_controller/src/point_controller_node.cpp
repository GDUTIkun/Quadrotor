#include "point_controller/control.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

namespace
{

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class PointControllerNode : public rclcpp::Node
{
public:
  PointControllerNode()
  : Node("point_controller_node"),
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
    config_.k_v = declare_parameter<double>("k_v", 0.8);
    config_.k_w = declare_parameter<double>("k_w", 1.8);
    config_.v_max_m_s = declare_parameter<double>("v_max_m_s", 0.25);
    config_.w_max_rad_s = declare_parameter<double>("w_max_rad_s", 0.8);
    config_.v_min_m_s = declare_parameter<double>("v_min_m_s", 0.03);
    config_.w_min_rad_s = declare_parameter<double>("w_min_rad_s", 0.08);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PointControllerNode::on_goal_pose, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/point_controller/status", 10);

    const double period_s = std::max(0.01, 1.0 / control_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s)),
      std::bind(&PointControllerNode::control_step, this));
  }

  ~PointControllerNode() override
  {
    publish_stop();
  }

private:
  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto frame_id =
      msg->header.frame_id.empty() ? global_frame_id_ : msg->header.frame_id;
    if (frame_id != global_frame_id_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring goal in frame \"%s\"; expected \"%s\"",
        frame_id.c_str(), global_frame_id_.c_str());
      return;
    }

    goal_ = point_controller::GoalPose{
      msg->pose.position.x,
      msg->pose.position.y,
      quaternion_to_yaw(msg->pose.orientation)};
    last_phase_.clear();
    RCLCPP_INFO(
      get_logger(), "Accepted goal x=%.3f, y=%.3f, yaw=%.3f",
      goal_->x, goal_->y, goal_->yaw);
  }

  void control_step()
  {
    if (!goal_) {
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

    const point_controller::RobotPose robot{
      transform.transform.translation.x,
      transform.transform.translation.y,
      quaternion_to_yaw(transform.transform.rotation)};
    const auto result = point_controller::compute_command(robot, *goal_, config_);

    if (result.phase != last_phase_) {
      last_phase_ = result.phase;
      RCLCPP_INFO(
        get_logger(), "Point controller phase=%s, distance=%.3f, yaw_error=%.3f",
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
           << " w=" << result.w;
    std_msgs::msg::String status_msg;
    status_msg.data = status.str();
    status_pub_->publish(status_msg);

    if (result.reached) {
      goal_.reset();
    }
  }

  void publish_stop()
  {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }
  }

  std::string global_frame_id_;
  std::string base_frame_id_;
  point_controller::ControllerConfig config_;
  std::optional<point_controller::GoalPose> goal_;
  std::string last_phase_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointControllerNode>());
  rclcpp::shutdown();
  return 0;
}
