#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class OffboardForwardLandNode : public rclcpp::Node
{
public:
  OffboardForwardLandNode() : Node("offboard_forward_land_node")
  {
    declare_parameter<double>("takeoff_height", 1.2);
    declare_parameter<double>("hover_seconds", 3.0);
    declare_parameter<double>("forward_speed", 0.15);
    declare_parameter<double>("descent_speed", 0.15);
    declare_parameter<double>("land_switch_height", 0.13);
    declare_parameter<double>("arrival_tolerance", 0.10);
    declare_parameter<bool>("auto_set_mode", true);
    declare_parameter<bool>("auto_arm", true);
    declare_parameter<std::string>("land_mode", "AUTO.LAND");

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      [this](mavros_msgs::msg::State::SharedPtr msg) { state_ = *msg; });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pose_ = *msg;
        pose_received_ = true;
        if (!reference_captured_) {
          start_x_ = msg->pose.position.x;
          start_y_ = msg->pose.position.y;
          initial_yaw_ = yaw_from_pose(*msg);
          reference_captured_ = true;
          RCLCPP_INFO(get_logger(), "Takeoff reference (%.2f, %.2f), holding yaw %.1f deg",
            start_x_, start_y_, initial_yaw_ * 180.0 / M_PI);
        }
      });
    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "mavros/setpoint_raw/local", 10);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    last_request_time_ = now();
    timer_ = create_wall_timer(20ms, std::bind(&OffboardForwardLandNode::timer_callback, this));
  }

  bool connected() const { return state_.connected; }

private:
  enum class Phase {
    WAIT_FOR_POSE, STREAM_SETPOINTS, ARM_AND_OFFBOARD, TAKEOFF,
    HOVER, FORWARD_DESCEND, LAND, FINISHED
  };

  void timer_callback()
  {
    if (!state_.connected) return;
    if (!pose_received_ || !reference_captured_) {
      phase_ = Phase::WAIT_FOR_POSE;
      return;
    }
    const double takeoff_z = std::max(0.2, get_parameter("takeoff_height").as_double());
    switch (phase_) {
      case Phase::WAIT_FOR_POSE:
        phase_ = Phase::STREAM_SETPOINTS;
        stream_count_ = 0;
        break;
      case Phase::STREAM_SETPOINTS:
        publish_position(start_x_, start_y_, takeoff_z);
        if (++stream_count_ >= 100) phase_ = Phase::ARM_AND_OFFBOARD;
        break;
      case Phase::ARM_AND_OFFBOARD:
        publish_position(start_x_, start_y_, takeoff_z);
        ensure_offboard_and_armed();
        if (state_.mode == "OFFBOARD" && state_.armed) {
          phase_ = Phase::TAKEOFF;
          RCLCPP_INFO(get_logger(), "Taking off to %.2f m", takeoff_z);
        }
        break;
      case Phase::TAKEOFF:
        publish_position(start_x_, start_y_, takeoff_z);
        if (position_error(start_x_, start_y_, takeoff_z) <= arrival_tolerance()) {
          hover_start_time_ = now();
          phase_ = Phase::HOVER;
          RCLCPP_INFO(get_logger(), "Takeoff complete; hovering for %.1f s", hover_seconds());
        }
        break;
      case Phase::HOVER:
        publish_position(start_x_, start_y_, takeoff_z);
        if ((now() - hover_start_time_).seconds() >= hover_seconds()) {
          descent_start_time_ = now();
          phase_ = Phase::FORWARD_DESCEND;
          RCLCPP_INFO(get_logger(), "Moving forward while descending");
        }
        break;
      case Phase::FORWARD_DESCEND:
        run_forward_descent(takeoff_z);
        break;
      case Phase::LAND:
        request_land();
        if (!state_.armed) {
          phase_ = Phase::FINISHED;
          RCLCPP_INFO(get_logger(), "Mission finished");
        }
        break;
      case Phase::FINISHED:
        break;
    }
  }

  void run_forward_descent(double takeoff_z)
  {
    const double elapsed = std::max(0.0, (now() - descent_start_time_).seconds());
    const double forward_speed = std::max(0.0, get_parameter("forward_speed").as_double());
    const double descent_speed = std::max(0.02, get_parameter("descent_speed").as_double());
    const double switch_z = std::clamp(
      get_parameter("land_switch_height").as_double(), 0.08, takeoff_z);
    // The local coordinate convention used by this project is:
    // +Y forward, +X right and +Z up. Therefore forward flight only changes Y.
    const double distance = forward_speed * elapsed;
    const double target_x = start_x_;
    const double target_y = start_y_ + distance;
    const double target_z = std::max(switch_z, takeoff_z - descent_speed * elapsed);
    publish_position(target_x, target_y, target_z);
    if (target_z <= switch_z + 1e-6 &&
      pose_.pose.position.z <= switch_z + arrival_tolerance()) {
      phase_ = Phase::LAND;
      RCLCPP_INFO(get_logger(), "Low altitude reached; requesting land mode");
    }
  }

  void publish_position(double x, double y, double z)
  {
    mavros_msgs::msg::PositionTarget msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
      mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    msg.yaw = initial_yaw_;
    setpoint_pub_->publish(msg);
  }

  void ensure_offboard_and_armed()
  {
    if (state_.mode != "OFFBOARD") {
      if ((now() - last_request_time_).seconds() < 2.0) return;
      if (get_parameter("auto_set_mode").as_bool() && set_mode_client_->service_is_ready()) {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "OFFBOARD";
        set_mode_client_->async_send_request(req);
      }
      last_request_time_ = now();
      return;
    }
    if (!state_.armed && (now() - last_request_time_).seconds() >= 1.0) {
      if (get_parameter("auto_arm").as_bool() && arm_client_->service_is_ready()) {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        arm_client_->async_send_request(req);
      }
      last_request_time_ = now();
    }
  }

  void request_land()
  {
    if (land_requested_ || !set_mode_client_->service_is_ready()) return;
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = get_parameter("land_mode").as_string();
    set_mode_client_->async_send_request(req);
    land_requested_ = true;
  }

  double position_error(double x, double y, double z) const
  {
    const double dx = pose_.pose.position.x - x;
    const double dy = pose_.pose.position.y - y;
    const double dz = pose_.pose.position.z - z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  double arrival_tolerance() const
  { return std::max(0.02, get_parameter("arrival_tolerance").as_double()); }
  double hover_seconds() const
  { return std::max(0.0, get_parameter("hover_seconds").as_double()); }
  static double yaw_from_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & q = pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  Phase phase_{Phase::WAIT_FOR_POSE};
  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped pose_;
  bool pose_received_{false}, reference_captured_{false}, land_requested_{false};
  int stream_count_{0};
  double start_x_{0.0}, start_y_{0.0}, initial_yaw_{0.0};
  rclcpp::Time last_request_time_, hover_start_time_, descent_start_time_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardForwardLandNode>();
  while (rclcpp::ok() && !node->connected()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
