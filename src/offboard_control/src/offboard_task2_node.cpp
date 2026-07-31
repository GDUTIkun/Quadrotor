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
    declare_parameter<double>("track_command_publish_period", 0.2);
    declare_parameter<std::string>("status_topic", "/offboard_task2/status");
    declare_parameter<double>("status_publish_period", 1.0);
    declare_parameter<double>("flight_height", 1.5);
    declare_parameter<double>("offset_x", 0.345);
    declare_parameter<double>("offset_y", 0.855);
    declare_parameter<double>("follow_tolerance", 0.1);
    declare_parameter<double>("approach_tolerance", 0.15);
    declare_parameter<double>("approach_stable_time", 0.3);
    declare_parameter<double>("landing_follow_tolerance", 0.15);
    declare_parameter<double>("fast_descent_height", 0.6);
    declare_parameter<double>("fast_descent_speed", 0.6);
    declare_parameter<double>("descent_speed", 0.05);
    declare_parameter<double>("land_switch_height", 0.38);
    declare_parameter<double>("arrival_tolerance", 0.05);
    declare_parameter<double>("takeoff_tolerance", 0.20);
    declare_parameter<double>("idle_after_land_seconds", 5.0);
    declare_parameter<double>("follow_stable_time", 1.5);
    declare_parameter<double>("return_tolerance", 0.07);
    declare_parameter<double>("land_request_height", 0.65);
    declare_parameter<double>("vehicle_timeout", 1.0);
    declare_parameter<double>("max_vehicle_jump", 0.5);
    declare_parameter<double>("first_land_lowpass_deadband", 0.180);
    declare_parameter<double>("first_land_lowpass_near_distance", 0.180);
    declare_parameter<double>("first_land_lowpass_far_distance", 1.6);
    declare_parameter<double>("first_land_lowpass_near_tau", 0.0);
    declare_parameter<double>("first_land_lowpass_far_tau", 2.0);
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
          target_yaw_ = initial_yaw_;
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
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), status_qos);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

    last_request_time_ = now();
    last_status_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    timer_ = create_wall_timer(20ms, std::bind(&OffboardTask2Node::timer_callback, this));

    RCLCPP_INFO(
      get_logger(), "Vehicle follow-and-land mission ready: topic=%s, offset=(%.3f, %.3f), height=%.2f m",
      vehicle_topic.c_str(), offset_x(), offset_y(), flight_height());
  }

  bool connected() const { return state_.connected; }

private:
  enum class Phase {
    WAIT_FOR_POSE, STREAM_SETPOINTS, ARM_AND_OFFBOARD, TAKEOFF,
    FOLLOW_APPROACH, FOLLOW_FAST_DESCEND, FOLLOW_SLOW_DESCEND, VEHICLE_LAND,
    IDLE_AFTER_VEHICLE_LAND, STREAM_VEHICLE_SETPOINTS, REARM_AND_OFFBOARD,
    VEHICLE_TAKEOFF, SECOND_FOLLOW, RETURN_HOME, DESCEND_FOR_LAND, HOME_LAND,
    FINISHED
  };

  void timer_callback()
  {
    if (!state_.connected) {
      publish_status_if_needed();
      return;
    }
    if (!pose_received_ || !reference_captured_) {
      phase_ = Phase::WAIT_FOR_POSE;
      publish_status_if_needed();
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
          phase_ = Phase::TAKEOFF;
          RCLCPP_INFO(get_logger(), "Taking off to %.2f m", target_z);
        }
        break;

      case Phase::TAKEOFF:
        // Keep X/Y fixed; following starts only after reaching flight height.
        publish_position(start_x_, start_y_, target_z);
        if (std::abs(pose_.pose.position.z - target_z) <= takeoff_tolerance()) {
          approach_stable_ = false;
          phase_ = Phase::FOLLOW_APPROACH;
          RCLCPP_INFO(get_logger(),
            "Takeoff complete; approaching vehicle target at fixed height");
        }
        break;

      case Phase::FOLLOW_APPROACH:
        update_horizontal_follow_target();
        publish_position(hold_x_, hold_y_, target_z);
        if (approach_stable_for_descent()) {
          descent_start_time_ = now();
          approach_stable_ = false;
          phase_ = Phase::FOLLOW_FAST_DESCEND;
          RCLCPP_INFO(get_logger(),
            "Vehicle target reached; starting fast follow descent");
        }
        break;

      case Phase::FOLLOW_FAST_DESCEND:
        run_follow_fast_descent(target_z);
        break;

      case Phase::FOLLOW_SLOW_DESCEND:
        run_follow_slow_descent(target_z);
        break;

      case Phase::VEHICLE_LAND:
        handle_vehicle_landing(land_switch_height());
        break;

      case Phase::IDLE_AFTER_VEHICLE_LAND:
        publish_vehicle_target_with_current_yaw(target_z);
        if ((now() - idle_start_time_).seconds() >= idle_after_land_seconds()) {
          stream_count_ = 0;
          second_takeoff_target_captured_ = false;
          last_request_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          phase_ = Phase::STREAM_VEHICLE_SETPOINTS;
          RCLCPP_INFO(get_logger(), "Idle complete; streaming vehicle takeoff setpoints");
        }
        break;

      case Phase::STREAM_VEHICLE_SETPOINTS:
        publish_vehicle_target_with_current_yaw(target_z);
        if (vehicle_target_available()) {
          if (++stream_count_ >= 100) {
            phase_ = Phase::REARM_AND_OFFBOARD;
            last_request_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            RCLCPP_INFO(get_logger(), "Vehicle takeoff setpoints streamed; requesting OFFBOARD and arm");
          }
        } else {
          stream_count_ = 0;
        }
        break;

      case Phase::REARM_AND_OFFBOARD:
        publish_vehicle_target_with_current_yaw(target_z);
        ensure_offboard_and_armed();
        if (state_.mode == "OFFBOARD" && state_.armed) {
          update_horizontal_follow_target();
          capture_second_takeoff_target();
          capture_takeoff_yaw("Vehicle takeoff armed");
          follow_stable_ = false;
          phase_ = Phase::VEHICLE_TAKEOFF;
          RCLCPP_INFO(get_logger(), "Taking off from vehicle target to %.2f m", target_z);
        }
        break;

      case Phase::VEHICLE_TAKEOFF:
        publish_second_takeoff_target(target_z);
        if (height_reached(target_z)) {
          follow_stable_ = false;
          phase_ = Phase::SECOND_FOLLOW;
          RCLCPP_INFO(get_logger(), "Vehicle takeoff complete; checking follow stability");
        }
        break;

      case Phase::SECOND_FOLLOW:
        publish_second_takeoff_target(target_z);
        if (second_takeoff_target_stable()) {
          phase_ = Phase::RETURN_HOME;
          RCLCPP_INFO(
            get_logger(), "Vehicle target stable for %.1f s; returning to takeoff point",
            follow_stable_time());
        }
        break;

      case Phase::RETURN_HOME:
        publish_position(start_x_, start_y_, target_z);
        if (home_reached(target_z)) {
          land_stable_ = false;
          phase_ = Phase::DESCEND_FOR_LAND;
          RCLCPP_INFO(
            get_logger(), "Takeoff point reached; descending to %.2f m before landing",
            land_request_height());
        }
        break;

      case Phase::DESCEND_FOR_LAND:
        publish_position(start_x_, start_y_, land_request_height());
        if (ready_to_request_land(land_request_height())) {
          last_request_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          phase_ = Phase::HOME_LAND;
          RCLCPP_INFO(
            get_logger(),
            "Landing request height reached and home stable for %.1f s; requesting landing",
            follow_stable_time());
        }
        break;

      case Phase::HOME_LAND:
        handle_home_landing(land_request_height());
        break;

      case Phase::FINISHED:
        break;
    }
    publish_track_start_command_if_needed();
    publish_status_if_needed();
  }

  std::string current_status() const
  {
    if (vehicle_follow_locked_) {
      return "FAULT";
    }

    switch (phase_) {
      case Phase::WAIT_FOR_POSE:
      case Phase::STREAM_SETPOINTS:
      case Phase::ARM_AND_OFFBOARD:
      case Phase::TAKEOFF:
      case Phase::STREAM_VEHICLE_SETPOINTS:
      case Phase::REARM_AND_OFFBOARD:
      case Phase::VEHICLE_TAKEOFF:
        return "TAKEOFF";

      case Phase::FOLLOW_APPROACH:
      case Phase::SECOND_FOLLOW:
        return "FOLLOW";

      case Phase::FOLLOW_FAST_DESCEND:
      case Phase::FOLLOW_SLOW_DESCEND:
      case Phase::VEHICLE_LAND:
      case Phase::IDLE_AFTER_VEHICLE_LAND:
      case Phase::RETURN_HOME:
      case Phase::DESCEND_FOR_LAND:
      case Phase::HOME_LAND:
        return "LAND";

      case Phase::FINISHED:
        return "COMPLETE";
    }

    return "FAULT";
  }

  void publish_status_if_needed()
  {
    const auto status = current_status();
    const auto current_time = now();
    const bool status_changed = status != last_published_status_;
    const bool period_elapsed = last_status_publish_time_.nanoseconds() == 0 ||
      (current_time - last_status_publish_time_).seconds() >= status_publish_period();
    if (!status_changed && !period_elapsed) {
      return;
    }

    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    last_published_status_ = status;
    last_status_publish_time_ = current_time;
    if (status_changed) {
      RCLCPP_INFO(get_logger(), "Mission status changed: %s", status.c_str());
    }
  }

  void handle_vehicle_landing(double target_z)
  {
    if (state_.mode != land_mode()) {
      publish_position(hold_x_, hold_y_, target_z);
      request_land_mode();
      return;
    }

    publish_position(hold_x_, hold_y_, target_z);
    request_disarm();
    if (!state_.armed) {
      idle_start_time_ = now();
      follow_stable_ = false;
      land_stable_ = false;
      phase_ = Phase::IDLE_AFTER_VEHICLE_LAND;
      RCLCPP_INFO(
        get_logger(), "Vehicle landing complete; idling for %.1f s",
        idle_after_land_seconds());
    }
  }

  void handle_home_landing(double target_z)
  {
    if (state_.mode != land_mode()) {
      publish_position(start_x_, start_y_, target_z);
      request_land_mode();
      return;
    }

    publish_position(start_x_, start_y_, target_z);
    request_disarm();
    if (!state_.armed) {
      phase_ = Phase::FINISHED;
      RCLCPP_INFO(get_logger(), "Follow-and-land mission finished");
    }
  }

  void publish_vehicle_target(double target_z)
  {
    update_horizontal_follow_target();
    publish_position(hold_x_, hold_y_, target_z);
  }

  void publish_vehicle_target_with_current_yaw(double target_z)
  {
    update_horizontal_follow_target();
    publish_position(hold_x_, hold_y_, target_z, yaw_from_pose(pose_));
  }

  void publish_second_takeoff_target(double target_z)
  {
    if (!second_takeoff_target_captured_) {
      capture_second_takeoff_target();
    }
    publish_position(second_takeoff_x_, second_takeoff_y_, target_z);
  }

  void capture_second_takeoff_target()
  {
    second_takeoff_x_ = hold_x_;
    second_takeoff_y_ = hold_y_;
    second_takeoff_target_captured_ = true;
    RCLCPP_INFO(
      get_logger(), "Second takeoff target locked at x=%.3f y=%.3f",
      second_takeoff_x_, second_takeoff_y_);
  }

  bool vehicle_target_available() const
  {
    return !vehicle_follow_locked_ && vehicle_pose_fresh();
  }

  bool vehicle_target_stable()
  {
    if (!vehicle_target_available()) {
      follow_stable_ = false;
      return false;
    }

    const double error = horizontal_error(hold_x_, hold_y_);
    if (error > follow_tolerance()) {
      follow_stable_ = false;
      return false;
    }

    if (!follow_stable_) {
      follow_stable_ = true;
      follow_stable_since_ = now();
      RCLCPP_INFO(
        get_logger(), "Vehicle target reached (error %.3f m); checking stability", error);
      return false;
    }

    return (now() - follow_stable_since_).seconds() >= follow_stable_time();
  }

  bool second_takeoff_target_stable()
  {
    if (!second_takeoff_target_captured_) {
      follow_stable_ = false;
      return false;
    }

    const double error = horizontal_error(second_takeoff_x_, second_takeoff_y_);
    if (error > follow_tolerance()) {
      follow_stable_ = false;
      return false;
    }

    if (!follow_stable_) {
      follow_stable_ = true;
      follow_stable_since_ = now();
      RCLCPP_INFO(
        get_logger(),
        "Second takeoff target reached (error %.3f m); checking stability",
        error);
      return false;
    }

    return (now() - follow_stable_since_).seconds() >= follow_stable_time();
  }

  bool height_reached(double target_z) const
  {
    return std::abs(pose_.pose.position.z - target_z) <= arrival_tolerance();
  }

  bool home_reached(double target_z) const
  {
    const double error = std::hypot(
      pose_.pose.position.x - start_x_, pose_.pose.position.y - start_y_);
    return error <= return_tolerance() && height_reached(target_z);
  }

  bool ready_to_request_land(double target_z)
  {
    const double error = std::hypot(
      pose_.pose.position.x - start_x_, pose_.pose.position.y - start_y_);
    if (error > return_tolerance() || !height_reached(target_z)) {
      land_stable_ = false;
      return false;
    }

    if (!land_stable_) {
      land_stable_ = true;
      land_stable_since_ = now();
      RCLCPP_INFO(
        get_logger(), "Landing pre-check reached (xy error %.3f m); checking stability",
        error);
      return false;
    }

    return (now() - land_stable_since_).seconds() >= follow_stable_time();
  }

  bool approach_stable_for_descent()
  {
    if (vehicle_follow_locked_ || !vehicle_pose_fresh()) {
      approach_stable_ = false;
      return false;
    }

    const double error = horizontal_error(hold_x_, hold_y_);
    if (error > approach_tolerance()) {
      approach_stable_ = false;
      return false;
    }

    if (!approach_stable_) {
      approach_stable_ = true;
      approach_stable_since_ = now();
      RCLCPP_INFO(
        get_logger(), "Approach target reached (error %.3f m); checking stability",
        error);
      return false;
    }

    return (now() - approach_stable_since_).seconds() >= approach_stable_time();
  }

  void request_land_mode()
  {
    if ((now() - last_request_time_).seconds() < 2.0 ||
      !set_mode_client_->service_is_ready())
    {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = land_mode();
    set_mode_client_->async_send_request(request);
    last_request_time_ = now();
    RCLCPP_INFO(get_logger(), "Landing mode requested: %s", land_mode().c_str());
  }

  void request_disarm()
  {
    if (!state_.armed || (now() - last_request_time_).seconds() < 1.0 ||
      !arm_client_->service_is_ready())
    {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false;
    arm_client_->async_send_request(
      request,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(), "Disarm response: success=%s, result=%u",
          response->success ? "true" : "false", response->result);
      });
    last_request_time_ = now();
    RCLCPP_INFO(get_logger(), "Disarm requested");
  }

  void run_follow_fast_descent(double takeoff_z)
  {
    update_horizontal_follow_target();
    const double elapsed = std::max(0.0, (now() - descent_start_time_).seconds());
    const double fast_z = fast_descent_height(takeoff_z);
    const double target_z = std::max(fast_z, takeoff_z - fast_descent_speed() * elapsed);
    publish_position(hold_x_, hold_y_, target_z);
    if (target_z <= fast_z + 1e-6 &&
      pose_.pose.position.z <= fast_z + arrival_tolerance())
    {
      descent_start_time_ = now();
      phase_ = Phase::FOLLOW_SLOW_DESCEND;
      RCLCPP_INFO(get_logger(), "Fast descent complete; slowing descent to land switch height");
    }
  }

  void run_follow_slow_descent(double takeoff_z)
  {
    update_horizontal_follow_target();
    const double elapsed = std::max(0.0, (now() - descent_start_time_).seconds());
    const double start_z = fast_descent_height(takeoff_z);
    const double switch_z = std::clamp(land_switch_height(), 0.08, start_z);
    const double target_z = std::max(switch_z, start_z - descent_speed() * elapsed);
    const double xy_error = horizontal_error(hold_x_, hold_y_);
    publish_position(hold_x_, hold_y_, target_z);
    if (target_z <= switch_z + 1e-6 &&
      pose_.pose.position.z <= switch_z + arrival_tolerance() &&
      !vehicle_follow_locked_ && vehicle_pose_fresh() &&
      xy_error <= landing_follow_tolerance())
    {
      phase_ = Phase::VEHICLE_LAND;
      last_request_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      RCLCPP_INFO(get_logger(), "Low altitude reached; requesting land mode");
    } else if (target_z <= switch_z + 1e-6 &&
      pose_.pose.position.z <= switch_z + arrival_tolerance())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "At land switch height, waiting for landing XY tolerance: error=%.3f m, tolerance=%.3f m",
        xy_error, landing_follow_tolerance());
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
    publish_position(x, y, z, target_yaw_);
  }

  void publish_position(double x, double y, double z, double yaw)
  {
    double command_x = x;
    double command_y = y;
    apply_first_land_lowpass(command_x, command_y);

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
    msg.position.x = command_x;
    msg.position.y = command_y;
    msg.position.z = z;
    msg.yaw = yaw;
    setpoint_pub_->publish(msg);
  }

  void apply_first_land_lowpass(double & x, double & y)
  {
    if (!first_land_lowpass_phase() || !pose_received_) {
      reset_first_land_lowpass();
      return;
    }

    const double target_distance = horizontal_error(x, y);
    if (target_distance < first_land_lowpass_deadband()) {
      reset_first_land_lowpass();
      return;
    }

    const auto current_time = now();
    if (!first_land_lowpass_active_) {
      first_land_filtered_x_ = pose_.pose.position.x;
      first_land_filtered_y_ = pose_.pose.position.y;
      first_land_lowpass_last_time_ = current_time;
      first_land_lowpass_active_ = true;
    }

    const double dt = std::clamp(
      (current_time - first_land_lowpass_last_time_).seconds(), 0.001, 0.1);
    first_land_lowpass_last_time_ = current_time;

    const double tau = first_land_lowpass_tau(target_distance);
    if (tau <= 0.0) {
      first_land_filtered_x_ = x;
      first_land_filtered_y_ = y;
    } else {
      const double alpha = dt / (tau + dt);
      first_land_filtered_x_ += alpha * (x - first_land_filtered_x_);
      first_land_filtered_y_ += alpha * (y - first_land_filtered_y_);
    }

    x = first_land_filtered_x_;
    y = first_land_filtered_y_;
  }

  void reset_first_land_lowpass()
  {
    first_land_lowpass_active_ = false;
  }

  bool first_land_lowpass_phase() const
  {
    switch (phase_) {
      case Phase::TAKEOFF:
      case Phase::FOLLOW_APPROACH:
      case Phase::FOLLOW_FAST_DESCEND:
      case Phase::FOLLOW_SLOW_DESCEND:
      case Phase::VEHICLE_LAND:
        return true;
      default:
        return false;
    }
  }

  double first_land_lowpass_tau(double distance) const
  {
    const double near_distance = first_land_lowpass_near_distance();
    const double far_distance = first_land_lowpass_far_distance();
    const double near_tau = first_land_lowpass_near_tau();
    const double far_tau = first_land_lowpass_far_tau();

    if (far_distance <= near_distance) {
      return distance >= far_distance ? far_tau : near_tau;
    }

    const double ratio = std::clamp(
      (distance - near_distance) / (far_distance - near_distance), 0.0, 1.0);
    return near_tau + ratio * (far_tau - near_tau);
  }

  void capture_takeoff_yaw(const std::string & label)
  {
    target_yaw_ = yaw_from_pose(pose_);
    RCLCPP_INFO(
      get_logger(), "%s yaw captured: %.1f deg", label.c_str(),
      target_yaw_ * 180.0 / M_PI);
  }

  void publish_track_start_command_if_needed()
  {
    if (phase_ == Phase::WAIT_FOR_POSE || phase_ == Phase::STREAM_SETPOINTS ||
      phase_ == Phase::ARM_AND_OFFBOARD || phase_ == Phase::FINISHED)
    {
      return;
    }
    if (state_.mode != "OFFBOARD" || !state_.armed) {
      return;
    }

    const auto current_time = now();
    if (last_track_command_time_.nanoseconds() != 0 &&
      (current_time - last_track_command_time_).seconds() < track_command_publish_period())
    {
      return;
    }

    std_msgs::msg::String message;
    message.data = get_parameter("track_start_command").as_string();
    track_command_pub_->publish(message);
    last_track_command_time_ = current_time;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Vehicle track command publishing: %s", message.data.c_str());
  }

  void ensure_offboard_and_armed()
  {
    if (state_.mode != "OFFBOARD") {
      offboard_confirmed_ = false;
    }
    if (!state_.armed) {
      arm_confirmed_ = false;
    }

    if (state_.mode != "OFFBOARD") {
      if ((now() - last_request_time_).seconds() < 2.0) {
        return;
      }
      if (get_parameter("auto_set_mode").as_bool() && set_mode_client_->service_is_ready()) {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        set_mode_client_->async_send_request(
          request,
          [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            RCLCPP_INFO(
              get_logger(), "OFFBOARD mode response: mode_sent=%s",
              future.get()->mode_sent ? "true" : "false");
          });
        RCLCPP_INFO(get_logger(), "OFFBOARD mode requested");
      }
      last_request_time_ = now();
      return;
    }

    if (!offboard_confirmed_) {
      offboard_confirmed_ = true;
      RCLCPP_INFO(get_logger(), "OFFBOARD mode confirmed");
    }

    if (!state_.armed && (now() - last_request_time_).seconds() >= 1.0) {
      if (get_parameter("auto_arm").as_bool() && arm_client_->service_is_ready()) {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        arm_client_->async_send_request(
          request,
          [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            const auto response = future.get();
            RCLCPP_INFO(
              get_logger(), "Arm response: success=%s, result=%u",
              response->success ? "true" : "false", response->result);
          });
        RCLCPP_INFO(get_logger(), "Arm requested");
      }
      last_request_time_ = now();
    }

    if (state_.armed && !arm_confirmed_) {
      arm_confirmed_ = true;
      RCLCPP_INFO(get_logger(), "Arm confirmed");
    }
  }

  double flight_height() const
  {
    return std::max(0.2, get_parameter("flight_height").as_double());
  }

  double offset_x() const { return get_parameter("offset_x").as_double(); }
  double offset_y() const { return get_parameter("offset_y").as_double(); }

  double follow_tolerance() const
  { return std::max(0.02, get_parameter("follow_tolerance").as_double()); }
  double approach_tolerance() const
  { return std::max(0.02, get_parameter("approach_tolerance").as_double()); }
  double approach_stable_time() const
  { return std::max(0.0, get_parameter("approach_stable_time").as_double()); }
  double landing_follow_tolerance() const
  { return std::max(0.02, get_parameter("landing_follow_tolerance").as_double()); }
  double fast_descent_height(double takeoff_z) const
  {
    const double switch_z = std::clamp(land_switch_height(), 0.08, takeoff_z);
    return std::clamp(get_parameter("fast_descent_height").as_double(), switch_z, takeoff_z);
  }
  double fast_descent_speed() const
  { return std::max(0.02, get_parameter("fast_descent_speed").as_double()); }
  double descent_speed() const
  { return std::max(0.02, get_parameter("descent_speed").as_double()); }
  double land_switch_height() const
  { return get_parameter("land_switch_height").as_double(); }
  double arrival_tolerance() const
  { return std::max(0.02, get_parameter("arrival_tolerance").as_double()); }
  double takeoff_tolerance() const
  { return std::max(0.02, get_parameter("takeoff_tolerance").as_double()); }
  double idle_after_land_seconds() const
  { return std::max(0.0, get_parameter("idle_after_land_seconds").as_double()); }
  double follow_stable_time() const
  { return std::max(0.1, get_parameter("follow_stable_time").as_double()); }
  double return_tolerance() const
  { return std::max(0.02, get_parameter("return_tolerance").as_double()); }
  double land_request_height() const
  {
    return std::clamp(
      get_parameter("land_request_height").as_double(), 0.2, flight_height());
  }
  std::string land_mode() const
  {
    const auto mode = get_parameter("land_mode").as_string();
    return mode.empty() ? "AUTO.LAND" : mode;
  }

  double vehicle_timeout() const
  {
    return std::max(0.1, get_parameter("vehicle_timeout").as_double());
  }

  double track_command_publish_period() const
  {
    return std::max(0.05, get_parameter("track_command_publish_period").as_double());
  }

  double status_publish_period() const
  {
    return std::max(0.1, get_parameter("status_publish_period").as_double());
  }

  double max_vehicle_jump() const
  {
    return std::max(0.01, get_parameter("max_vehicle_jump").as_double());
  }

  double first_land_lowpass_deadband() const
  {
    return std::max(0.0, get_parameter("first_land_lowpass_deadband").as_double());
  }

  double first_land_lowpass_near_distance() const
  {
    return std::max(0.0, get_parameter("first_land_lowpass_near_distance").as_double());
  }

  double first_land_lowpass_far_distance() const
  {
    return std::max(
      first_land_lowpass_near_distance(),
      get_parameter("first_land_lowpass_far_distance").as_double());
  }

  double first_land_lowpass_near_tau() const
  {
    return std::max(0.0, get_parameter("first_land_lowpass_near_tau").as_double());
  }

  double first_land_lowpass_far_tau() const
  {
    return std::max(0.0, get_parameter("first_land_lowpass_far_tau").as_double());
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
  bool follow_stable_{false};
  bool land_stable_{false};
  bool approach_stable_{false};
  bool offboard_confirmed_{false};
  bool arm_confirmed_{false};
  bool first_land_lowpass_active_{false};
  bool second_takeoff_target_captured_{false};
  int stream_count_{0};
  double start_x_{0.0};
  double start_y_{0.0};
  double initial_yaw_{0.0};
  double target_yaw_{0.0};
  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double hold_x_{0.0};
  double hold_y_{0.0};
  double first_land_filtered_x_{0.0};
  double first_land_filtered_y_{0.0};
  double second_takeoff_x_{0.0};
  double second_takeoff_y_{0.0};
  std::string last_published_status_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_track_command_time_;
  rclcpp::Time last_status_publish_time_;
  rclcpp::Time last_vehicle_time_;
  rclcpp::Time descent_start_time_;
  rclcpp::Time idle_start_time_;
  rclcpp::Time approach_stable_since_;
  rclcpp::Time follow_stable_since_;
  rclcpp::Time land_stable_since_;
  rclcpp::Time first_land_lowpass_last_time_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr track_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
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
