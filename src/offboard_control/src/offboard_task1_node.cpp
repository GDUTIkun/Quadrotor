#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class OffboardTask1Node : public rclcpp::Node
{
public:
  OffboardTask1Node() : Node("offboard_task1_node")
  {
    declare_parameter<std::string>("vehicle_topic", "/car/target_pose");
    declare_parameter<std::string>("track_command_topic", "/car/track_runner/command");
    declare_parameter<std::string>("track_start_command", "start");
    declare_parameter<double>("flight_height", 1.5);
    declare_parameter<double>("offset_x", 0.375);
    declare_parameter<double>("offset_y", 0.875);
    declare_parameter<double>("takeoff_tolerance", 0.10);
    declare_parameter<double>("vehicle_timeout", 1.0);
    declare_parameter<double>("max_vehicle_jump", 0.5);
    declare_parameter<double>("follow_tolerance", 0.1);
    declare_parameter<double>("follow_stable_time", 2.0);
    declare_parameter<double>("drop_height", 0.5);
    declare_parameter<double>("mission_z_speed", 0.05);
    declare_parameter<double>("release_wait_time", 1.0);
    declare_parameter<double>("return_tolerance", 0.1);
    declare_parameter<std::string>("servo_topic", "/servo/angle_deg");
    declare_parameter<double>("release_angle", 0.0);
    declare_parameter<std::string>("land_mode", "AUTO.LAND");
    declare_parameter<bool>("auto_set_mode", true);
    declare_parameter<bool>("auto_arm", true);

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
              "Vehicle position jumped %.3f m (limit %.3f m); follow is permanently locked "
              "and UAV will hold position until this node is restarted",
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
    servo_angle_pub_ = create_publisher<std_msgs::msg::Float64>(
      get_parameter("servo_topic").as_string(), 10);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

    last_request_time_ = now();
    timer_ = create_wall_timer(20ms, std::bind(&OffboardTask1Node::timer_callback, this));

    RCLCPP_INFO(
      get_logger(), "Vehicle follow mission ready: topic=%s, offset=(%.3f, %.3f), height=%.2f m",
      vehicle_topic.c_str(), offset_x(), offset_y(), flight_height());
  }

  bool connected() const { return state_.connected; }

private:
  enum class Phase {
    WAIT_FOR_POSE,
    STREAM_SETPOINTS,
    ARM_AND_OFFBOARD,
    TAKEOFF,
    FOLLOW,
    DESCEND_FOR_DROP,
    RELEASE_PAYLOAD,
    ASCEND_AFTER_DROP,
    RETURN_HOME,
    LAND,
    COMPLETE
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
        publish_position(start_x_, start_y_, target_z);
        if (std::abs(pose_.pose.position.z - target_z) <= takeoff_tolerance()) {
          hold_x_ = start_x_;
          hold_y_ = start_y_;
          phase_ = Phase::FOLLOW;
          RCLCPP_INFO(get_logger(), "Takeoff complete; starting vehicle follow");
        }
        break;

      case Phase::FOLLOW:
        follow_vehicle(target_z);
        check_follow_complete();
        break;

      case Phase::DESCEND_FOR_DROP:
        follow_vehicle(slow_z_setpoint(drop_height()));
        if (height_reached(drop_height())) {
          publish_release_once();
          release_started_time_ = now();
          phase_ = Phase::RELEASE_PAYLOAD;
          RCLCPP_INFO(
            get_logger(), "Drop height reached at %.2f m; releasing payload",
            drop_height());
        }
        break;

      case Phase::RELEASE_PAYLOAD:
        follow_vehicle(drop_height());
        if ((now() - release_started_time_).seconds() >= release_wait_time()) {
          reset_slow_z_control();
          phase_ = Phase::ASCEND_AFTER_DROP;
          RCLCPP_INFO(
            get_logger(), "Payload release complete; ascending to %.2f m", target_z);
        }
        break;

      case Phase::ASCEND_AFTER_DROP:
        follow_vehicle(slow_z_setpoint(target_z));
        if (height_reached(target_z)) {
          phase_ = Phase::RETURN_HOME;
          RCLCPP_INFO(get_logger(), "Original height restored; returning to takeoff point");
        }
        break;

      case Phase::RETURN_HOME:
        publish_position(start_x_, start_y_, target_z);
        if (home_reached(target_z)) {
          phase_ = Phase::LAND;
          last_request_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          RCLCPP_INFO(get_logger(), "Takeoff point reached; requesting landing");
        }
        break;

      case Phase::LAND:
        handle_landing(target_z);
        break;

      case Phase::COMPLETE:
        break;
    }
  }

  void check_follow_complete()
  {
    if (vehicle_follow_locked_ || !vehicle_pose_fresh()) {
      follow_stable_ = false;
      return;
    }

    const double error = std::hypot(
      pose_.pose.position.x - hold_x_, pose_.pose.position.y - hold_y_);
    if (error > follow_tolerance()) {
      follow_stable_ = false;
      return;
    }

    if (!follow_stable_) {
      follow_stable_ = true;
      follow_stable_since_ = now();
      RCLCPP_INFO(
        get_logger(), "Follow target reached (error %.3f m); checking stability", error);
      return;
    }

    if ((now() - follow_stable_since_).seconds() >= follow_stable_time()) {
      follow_stable_ = false;
      reset_slow_z_control();
      phase_ = Phase::DESCEND_FOR_DROP;
      RCLCPP_INFO(
        get_logger(), "Vehicle follow stable for %.1f s; descending to %.2f m",
        follow_stable_time(), drop_height());
    }
  }

  void reset_slow_z_control()
  {
    slow_z_active_ = false;
  }

  double slow_z_setpoint(double target_z)
  {
    const auto current_time = now();
    if (!slow_z_active_) {
      slow_z_command_ = pose_.pose.position.z;
      slow_z_last_update_ = current_time;
      slow_z_active_ = true;
      return slow_z_command_;
    }

    const double dt = std::clamp(
      (current_time - slow_z_last_update_).seconds(), 0.0, 0.1);
    slow_z_last_update_ = current_time;
    const double max_step = mission_z_speed() * dt;
    const double error = target_z - slow_z_command_;
    if (std::abs(error) <= max_step) {
      slow_z_command_ = target_z;
    } else {
      slow_z_command_ += std::copysign(max_step, error);
    }
    return slow_z_command_;
  }

  bool height_reached(double target_z) const
  {
    return std::abs(pose_.pose.position.z - target_z) <= takeoff_tolerance();
  }

  bool home_reached(double target_z) const
  {
    const double horizontal_error = std::hypot(
      pose_.pose.position.x - start_x_, pose_.pose.position.y - start_y_);
    return horizontal_error <= return_tolerance() && height_reached(target_z);
  }

  void publish_release_once()
  {
    if (payload_released_) {
      return;
    }
    std_msgs::msg::Float64 message;
    message.data = release_angle();
    servo_angle_pub_->publish(message);
    payload_released_ = true;
    RCLCPP_INFO(get_logger(), "Servo release command published: %.1f deg", message.data);
  }

  void handle_landing(double target_z)
  {
    if (state_.mode != land_mode()) {
      publish_position(start_x_, start_y_, target_z);
      if ((now() - last_request_time_).seconds() >= 2.0 &&
        set_mode_client_->service_is_ready())
      {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = land_mode();
        set_mode_client_->async_send_request(request);
        last_request_time_ = now();
        RCLCPP_INFO(get_logger(), "Landing mode requested: %s", land_mode().c_str());
      }
      return;
    }

    if (!state_.armed) {
      phase_ = Phase::COMPLETE;
      RCLCPP_INFO(get_logger(), "Landing complete and vehicle disarmed");
    }
  }

  void follow_vehicle(double target_z)
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
    publish_position(hold_x_, hold_y_, target_z);
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

  double flight_height() const
  {
    return std::max(0.2, get_parameter("flight_height").as_double());
  }

  double offset_x() const { return get_parameter("offset_x").as_double(); }
  double offset_y() const { return get_parameter("offset_y").as_double(); }

  double takeoff_tolerance() const
  {
    return std::max(0.02, get_parameter("takeoff_tolerance").as_double());
  }

  double vehicle_timeout() const
  {
    return std::max(0.1, get_parameter("vehicle_timeout").as_double());
  }

  double max_vehicle_jump() const
  {
    return std::max(0.01, get_parameter("max_vehicle_jump").as_double());
  }

  double follow_tolerance() const
  {
    return std::max(0.02, get_parameter("follow_tolerance").as_double());
  }

  double follow_stable_time() const
  {
    return std::max(0.1, get_parameter("follow_stable_time").as_double());
  }

  double drop_height() const
  {
    return std::clamp(
      get_parameter("drop_height").as_double(), 0.2, flight_height());
  }

  double mission_z_speed() const
  {
    return std::clamp(
      get_parameter("mission_z_speed").as_double(), 0.05, 1.0);
  }

  double release_wait_time() const
  {
    return std::max(0.0, get_parameter("release_wait_time").as_double());
  }

  double return_tolerance() const
  {
    return std::max(0.02, get_parameter("return_tolerance").as_double());
  }

  double release_angle() const
  {
    return std::clamp(get_parameter("release_angle").as_double(), 0.0, 180.0);
  }

  std::string land_mode() const
  {
    const auto mode = get_parameter("land_mode").as_string();
    return mode.empty() ? "AUTO.LAND" : mode;
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
  bool follow_stable_{false};
  bool payload_released_{false};
  bool slow_z_active_{false};
  int stream_count_{0};
  double start_x_{0.0};
  double start_y_{0.0};
  double initial_yaw_{0.0};
  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double hold_x_{0.0};
  double hold_y_{0.0};
  double slow_z_command_{0.0};
  rclcpp::Time last_request_time_;
  rclcpp::Time last_vehicle_time_;
  rclcpp::Time follow_stable_since_;
  rclcpp::Time release_started_time_;
  rclcpp::Time slow_z_last_update_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr track_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_angle_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardTask1Node>();
  while (rclcpp::ok() && !node->connected()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
