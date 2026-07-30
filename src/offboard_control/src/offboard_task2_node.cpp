#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class OffboardTask2Node : public rclcpp::Node
{
public:
  OffboardTask2Node() : Node("offboard_task2_node")
  {
    declare_parameter<std::string>("vehicle_topic", "/car/target_pose");
    declare_parameter<std::string>("track_command_topic", "/car/track_runner/command");
    declare_parameter<std::string>("track_start_command", "start");
    declare_parameter<double>("flight_height", 1.5);
    declare_parameter<double>("offset_x", 0.375);
    declare_parameter<double>("offset_y", 0.875);
    declare_parameter<double>("follow_tolerance", 0.10);
    declare_parameter<double>("descent_speed", 0.08);
    declare_parameter<double>("land_switch_height", 0.18);
    declare_parameter<double>("arrival_tolerance", 0.10);
    declare_parameter<double>("vehicle_timeout", 1.0);
    declare_parameter<double>("max_vehicle_jump", 0.5);
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
          hold_x_ = start_x_;
          hold_y_ = start_y_;
          initial_yaw_ = yaw_from_pose(*msg);
          reference_captured_ = true;
          RCLCPP_INFO(
            get_logger(), "Takeoff reference: x=%.3f, y=%.3f, yaw=%.1f deg",
            start_x_, start_y_, initial_yaw_ * 180.0 / M_PI);
        }
      });

    const auto vehicle_topic = get_parameter("vehicle_topic").as_string();
    vehicle_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      vehicle_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const double x = msg->pose.position.x;
        const double y = msg->pose.position.y;
        if (!std::isfinite(x) || !std::isfinite(y)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Ignoring invalid vehicle position");
          return;
        }

        if (vehicle_pose_received_ && !vehicle_follow_locked_) {
          const double dx = x - vehicle_x_;
          const double dy = y - vehicle_y_;
          const double jump_distance = std::hypot(dx, dy);
          if (jump_distance > max_vehicle_jump()) {
            vehicle_follow_locked_ = true;
            RCLCPP_ERROR(
              get_logger(),
              "Vehicle position jumped %.3f m (limit %.3f m); horizontal follow is "
              "permanently locked, but the descent mission will continue",
              jump_distance, max_vehicle_jump());
            return;
          }
        }

        if (vehicle_follow_locked_) {
          return;
        }
        vehicle_x_ = x;
        vehicle_y_ = y;
        vehicle_pose_received_ = true;
        last_vehicle_time_ = now();
      });

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "mavros/setpoint_raw/local", 10);
    track_command_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("track_command_topic").as_string(), 10);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

    last_request_time_ = now();
    timer_ = create_wall_timer(20ms, std::bind(&OffboardTask2Node::timer_callback, this));

    RCLCPP_INFO(
      get_logger(), "Vehicle follow-and-land mission ready: topic=%s, offset=(%.3f, %.3f), height=%.2f m",
      vehicle_topic.c_str(), offset_x(), offset_y(), flight_height());
  }

  bool connected() const { return state_.connected; }

private:
  enum class Phase {
    WAIT_FOR_POSE, STREAM_SETPOINTS, ARM_AND_OFFBOARD, TAKEOFF,
    FOLLOW_APPROACH, FOLLOW_DESCEND, LAND, FINISHED
  };

  void timer_callback()
  {
    if (!state_.connected) {
      return;
    }
    if (!pose_received_ || !reference_captured_) {
      phase_ = Phase::WAIT_FOR_POSE;
      return;
    }

    const double target_z = flight_height();
    switch (phase_) {
      case Phase::WAIT_FOR_POSE:
        stream_count_ = 0;
        phase_ = Phase::STREAM_SETPOINTS;
        break;

      case Phase::STREAM_SETPOINTS:
        publish_position(start_x_, start_y_, target_z);
        if (++stream_count_ >= 100) {
          phase_ = Phase::ARM_AND_OFFBOARD;
        }
        break;

      case Phase::ARM_AND_OFFBOARD:
        publish_position(start_x_, start_y_, target_z);
        ensure_offboard_and_armed();
        if (state_.mode == "OFFBOARD" && state_.armed) {
          publish_track_start_once();
          phase_ = Phase::TAKEOFF;
          RCLCPP_INFO(get_logger(), "Taking off to %.2f m", target_z);
        }
        break;

      case Phase::TAKEOFF:
        // Keep X/Y fixed; following starts only after reaching flight height.
        publish_position(start_x_, start_y_, target_z);
        if (std::abs(pose_.pose.position.z - target_z) <= arrival_tolerance()) {
          phase_ = Phase::FOLLOW_APPROACH;
          RCLCPP_INFO(get_logger(),
            "Takeoff complete; approaching vehicle target at fixed height");
        }
        break;

      case Phase::FOLLOW_APPROACH:
        update_horizontal_follow_target();
        publish_position(hold_x_, hold_y_, target_z);
        if (!vehicle_follow_locked_ && vehicle_pose_fresh() &&
          horizontal_error(hold_x_, hold_y_) <= follow_tolerance())
        {
          descent_start_time_ = now();
          phase_ = Phase::FOLLOW_DESCEND;
          RCLCPP_INFO(get_logger(),
            "Vehicle target reached; starting follow while descending");
        }
        break;

      case Phase::FOLLOW_DESCEND:
        run_follow_descent(target_z);
        break;

      case Phase::LAND:
        request_land();
        if (!state_.armed) {
          phase_ = Phase::FINISHED;
          RCLCPP_INFO(get_logger(), "Follow-and-land mission finished");
        }
        break;

      case Phase::FINISHED:
        break;
    }
  }

  void run_follow_descent(double takeoff_z)
  {
    update_horizontal_follow_target();
    const double elapsed = std::max(0.0, (now() - descent_start_time_).seconds());
    const double switch_z = std::clamp(land_switch_height(), 0.08, takeoff_z);
    const double target_z = std::max(switch_z, takeoff_z - descent_speed() * elapsed);
    publish_position(hold_x_, hold_y_, target_z);
    if (target_z <= switch_z + 1e-6 &&
      pose_.pose.position.z <= switch_z + arrival_tolerance())
    {
      phase_ = Phase::LAND;
      RCLCPP_INFO(get_logger(), "Low altitude reached; requesting land mode");
    }
  }

  void update_horizontal_follow_target()
  {
    if (vehicle_follow_locked_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Vehicle follow is locked due to a coordinate jump; holding target (%.3f, %.3f)",
        hold_x_, hold_y_);
    } else if (vehicle_pose_fresh()) {
      // Both vehicle and UAV coordinates must be expressed in the same local map frame.
      hold_x_ = vehicle_x_ + offset_x();
      hold_y_ = vehicle_y_ + offset_y();
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "No fresh vehicle pose; holding the last safe target (%.3f, %.3f)",
        hold_x_, hold_y_);
    }
  }

  double horizontal_error(double x, double y) const
  {
    return std::hypot(pose_.pose.position.x - x, pose_.pose.position.y - y);
  }

  bool vehicle_pose_fresh() const
  {
    if (!vehicle_pose_received_) {
      return false;
    }
    return (now() - last_vehicle_time_).seconds() <= vehicle_timeout();
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

  void publish_track_start_once()
  {
    if (track_start_sent_) return;
    std_msgs::msg::String message;
    message.data = get_parameter("track_start_command").as_string();
    track_command_pub_->publish(message);
    track_start_sent_ = true;
    RCLCPP_INFO(get_logger(), "Vehicle track command published: %s", message.data.c_str());
  }

  void ensure_offboard_and_armed()
  {
    if (state_.mode != "OFFBOARD") {
      if ((now() - last_request_time_).seconds() < 2.0) {
        return;
      }
      if (get_parameter("auto_set_mode").as_bool() && set_mode_client_->service_is_ready()) {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        set_mode_client_->async_send_request(request);
      }
      last_request_time_ = now();
      return;
    }

    if (!state_.armed && (now() - last_request_time_).seconds() >= 1.0) {
      if (get_parameter("auto_arm").as_bool() && arm_client_->service_is_ready()) {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        arm_client_->async_send_request(request);
      }
      last_request_time_ = now();
    }
  }

  void request_land()
  {
    if (land_requested_ || !set_mode_client_->service_is_ready()) return;
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = get_parameter("land_mode").as_string();
    set_mode_client_->async_send_request(request);
    land_requested_ = true;
  }

  double flight_height() const
  {
    return std::max(0.2, get_parameter("flight_height").as_double());
  }

  double offset_x() const { return get_parameter("offset_x").as_double(); }
  double offset_y() const { return get_parameter("offset_y").as_double(); }

  double follow_tolerance() const
  { return std::max(0.02, get_parameter("follow_tolerance").as_double()); }
  double descent_speed() const
  { return std::max(0.02, get_parameter("descent_speed").as_double()); }
  double land_switch_height() const
  { return get_parameter("land_switch_height").as_double(); }
  double arrival_tolerance() const
  { return std::max(0.02, get_parameter("arrival_tolerance").as_double()); }

  double vehicle_timeout() const
  {
    return std::max(0.1, get_parameter("vehicle_timeout").as_double());
  }

  double max_vehicle_jump() const
  {
    return std::max(0.01, get_parameter("max_vehicle_jump").as_double());
  }

  static double yaw_from_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & q = pose.pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  Phase phase_{Phase::WAIT_FOR_POSE};
  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped pose_;
  bool pose_received_{false};
  bool reference_captured_{false};
  bool vehicle_pose_received_{false};
  bool vehicle_follow_locked_{false};
  bool track_start_sent_{false};
  bool land_requested_{false};
  int stream_count_{0};
  double start_x_{0.0};
  double start_y_{0.0};
  double initial_yaw_{0.0};
  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double hold_x_{0.0};
  double hold_y_{0.0};
  rclcpp::Time last_request_time_;
  rclcpp::Time last_vehicle_time_;
  rclcpp::Time descent_start_time_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr track_command_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardTask2Node>();
  while (rclcpp::ok() && !node->connected()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
