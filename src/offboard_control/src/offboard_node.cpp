#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class OffboardMissionNode : public rclcpp::Node {
public:
    OffboardMissionNode()
        : Node("offboard_node")
    {
        declare_parameter<bool>("auto_set_mode", true);
        declare_parameter<bool>("auto_arm", true);

        declare_parameter<int>("target1", 1);
        declare_parameter<int>("target2", 2);
        declare_parameter<double>("flight_z", 0.5);
        declare_parameter<double>("takeoff_hold_seconds", 0.5);
        declare_parameter<double>("qr_hold_seconds", 1.0);
        declare_parameter<double>("obstacle_boundary_hold_seconds", 0.5);
        declare_parameter<double>("target_hold_seconds", 1.0);
        declare_parameter<double>("special_hold_seconds", 1.0);
        declare_parameter<double>("servo_post_hold_seconds", 1.0);
        declare_parameter<double>("pre_land_left_offset", 0.5);
        declare_parameter<double>("pre_land_hold_seconds", 1.0);
        declare_parameter<double>("arrival_tolerance", 0.1);
        declare_parameter<double>("orbit_arrival_tolerance", 0.14);
        declare_parameter<double>("setpoint_lowpass_tau", 0.8);
        declare_parameter<double>("setpoint_snap_tolerance", 0.05);
        declare_parameter<std::string>("land_mode", "AUTO.LAND");
        declare_parameter<bool>("enable_rc_stop", true);
        declare_parameter<int>("rc_stop_channel", 7);
        declare_parameter<int>("rc_stop_pwm_threshold", 1800);
        declare_parameter<bool>("rc_stop_when_high", true);

        declare_parameter<double>("origin_x", -0.70);
        declare_parameter<double>("origin_y", 3.60);
        declare_parameter<double>("square_size", 1.4);
        declare_parameter<int>("orbit_laps", 2);
        declare_parameter<double>("orbit_speed", 0.75);
        declare_parameter<double>("corner_radius", 0.65);
        declare_parameter<double>("lookahead_distance", 0.10);
        declare_parameter<double>("projection_search_window", 0.8);
        declare_parameter<double>("projection_speed_multiplier", 1.0);
        declare_parameter<bool>("enable_velocity_feedforward", true);
        declare_parameter<double>("max_tracking_error", 0.30);
        declare_parameter<double>("tracking_recover_error", 0.20);
        declare_parameter<double>("lookahead_ramp_rate", 0.60);
        declare_parameter<double>("velocity_feedforward_ramp_rate", 1.20);
        declare_parameter<double>("orbit_final_lap_min_speed_scale", 0.85);
        declare_parameter<double>("orbit_finish_hold_seconds", 0.5);
        declare_parameter<bool>("keep_velocity_feedforward_in_recovery", true);
        declare_parameter<bool>("strict_path_setpoint", true);
        declare_parameter<bool>("fixed_trajectory_progress", true);
        declare_parameter<double>("orbit_setpoint_lowpass_tau", 0.0);
        declare_parameter<double>("orbit_setpoint_snap_tolerance", 0.04);
        declare_parameter<double>("setpoint_max_xy_speed", 0.0);
        declare_parameter<bool>("enable_servo_actions", true);
        declare_parameter<double>("servo_command_repeat_seconds", 1.0);
        declare_parameter<double>("gpio12_first_angle_deg", 0.0);
        declare_parameter<double>("gpio12_second_angle_deg", 120.0);
        declare_parameter<double>("gpio13_special_angle_deg", 0.0);

        target1_index_ = read_target_index("target1", 1);
        target2_index_ = read_target_index("target2", 2);

        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                current_state_ = *msg;
                if (current_state_.mode == "STABILIZED" && !emergency_stop_) {
                    emergency_stop_ = true;
                    RCLCPP_WARN(get_logger(),
                                "Manual STABILIZED mode detected. Stopping offboard mission.");
                }
            });

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "mavros/local_position/pose", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_pose_ = *msg;
                pose_received_ = true;
                if (!initial_yaw_captured_) {
                    initial_yaw_ = yaw_from_pose(*msg);
                    initial_yaw_captured_ = true;
                    RCLCPP_INFO(get_logger(),
                                "Captured initial yaw %.1f deg, keeping this heading.",
                                initial_yaw_ * 180.0 / M_PI);
                }
            });

        setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
            "mavros/setpoint_raw/local", 10);
        gpio12_angle_pub_ = create_publisher<std_msgs::msg::Float64>(
            "servo/gpio12_angle_deg", 10);
        gpio13_angle_pub_ = create_publisher<std_msgs::msg::Float64>(
            "servo/gpio13_angle_deg", 10);

        rc_sub_ = create_subscription<mavros_msgs::msg::RCIn>(
            "mavros/rc/in", rclcpp::SensorDataQoS(),
            [this](const mavros_msgs::msg::RCIn::SharedPtr msg) {
                check_rc_stop(*msg);
            });

        set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        last_request_time_ = now();
        timer_ = create_wall_timer(20ms, std::bind(&OffboardMissionNode::timer_callback, this));
        start_terminal_advance_thread();

        RCLCPP_INFO(get_logger(),
                    "Mission targets: target1=%d target2=%d, orbit_laps=%d.",
                    target1_index_, target2_index_, orbit_laps());
        RCLCPP_INFO(get_logger(),
                    "Point-test mode: terminal input is only used before the early non-servo mission points.");
        RCLCPP_INFO(get_logger(),
                    "Servo action sequence: target1 GPIO12 first, hover, target2 GPIO12 second, hover, special target GPIO13, hover, then offset landing.");
    }

    bool is_connected() const
    {
        return current_state_.connected;
    }

private:
    enum class Phase {
        INIT_STREAM,
        ARMING,
        TAKEOFF_HOLD,
        MOVE_TO_QR,
        QR_HOLD,
        MOVE_TO_ORBIT_START,
        ORBIT_BOUNDARY_HOLD,
        ORBIT,
        ORBIT_FINISH_HOLD,
        MOVE_TO_TARGET_1,
        TARGET_1_HOLD,
        TARGET_1_SERVO_ACTION,
        TARGET_1_POST_SERVO_HOLD,
        MOVE_TO_TARGET_2,
        TARGET_2_HOLD,
        TARGET_2_SERVO_ACTION,
        TARGET_2_POST_SERVO_HOLD,
        MOVE_TO_SPECIAL_TARGET,
        SPECIAL_HOLD,
        SPECIAL_SERVO_ACTION,
        SPECIAL_POST_SERVO_HOLD,
        MOVE_LEFT_BEFORE_LAND,
        PRE_LAND_HOLD,
        LANDING,
        FINISHED,
    };

    struct Target {
        double x;
        double y;
        double z;
    };

    struct TrajectoryPoint {
        double x;
        double y;
        double z;
        double vx;
        double vy;
        double vz;
    };

    struct Projection {
        double distance;
        double error;
    };

    enum class ServoChannel {
        NONE,
        GPIO12,
        GPIO13,
    };

    void timer_callback()
    {
        if (emergency_stop_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "RC/manual override active. Offboard mission stopped.");
            return;
        }

        if (!current_state_.connected) {
            return;
        }

        publish_active_servo_command();

        const Target takeoff{0.0, 0.0, flight_z()};
        const Target qr{0.0, 1.8, flight_z()};
        const Target orbit_boundary = obstacle_boundary_target();
        const Target special{-1.0, 6.0, flight_z()};
        const Target pre_land = pre_land_target(special);
        const Target target1 = target_from_index(target1_index_);
        const Target target2 = target_from_index(target2_index_);

        switch (phase_) {
        case Phase::INIT_STREAM:
            publish_position(takeoff);
            ++init_stream_count_;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[INIT_STREAM] Streaming takeoff setpoint.");
            if (init_stream_count_ >= 100) {
                phase_ = Phase::ARMING;
                RCLCPP_INFO(get_logger(), "[INIT_STREAM] Ready to request OFFBOARD and arming.");
            }
            break;

        case Phase::ARMING:
            publish_position(takeoff);
            ensure_offboard_and_armed();
            if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
                phase_ = Phase::TAKEOFF_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[ARMING] OFFBOARD and armed. Taking off.");
            }
            break;

        case Phase::TAKEOFF_HOLD:
            publish_position(takeoff);
            if (wait_for_terminal_advance(takeoff, arrival_tolerance())) {
                phase_ = Phase::MOVE_TO_QR;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[TAKEOFF_HOLD] Terminal advance received. Moving to QR code.");
            }
            break;

        case Phase::MOVE_TO_QR:
            publish_position(qr);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[MOVE_TO_QR] Flying to QR point %.2f %.2f %.2f.",
                                 qr.x, qr.y, qr.z);
            if (is_at_target(qr, arrival_tolerance())) {
                phase_ = Phase::QR_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[MOVE_TO_QR] Reached QR point.");
            }
            break;

        case Phase::QR_HOLD:
            publish_position(qr);
            if (wait_for_terminal_advance(qr, arrival_tolerance())) {
                phase_ = Phase::MOVE_TO_ORBIT_START;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[QR_HOLD] Terminal advance received. Moving to orbit boundary.");
            }
            break;

        case Phase::MOVE_TO_ORBIT_START:
            publish_position(orbit_boundary);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[MOVE_TO_ORBIT_START] Flying to %.2f %.2f %.2f.",
                                 orbit_boundary.x, orbit_boundary.y, orbit_boundary.z);
            if (is_at_target(orbit_boundary, orbit_arrival_tolerance())) {
                phase_ = Phase::ORBIT_BOUNDARY_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[MOVE_TO_ORBIT_START] Reached obstacle lower boundary center.");
            }
            break;

        case Phase::ORBIT_BOUNDARY_HOLD:
            publish_position(orbit_boundary);
            if (wait_for_terminal_advance(orbit_boundary, orbit_arrival_tolerance())) {
                if (orbit_laps() <= 0) {
                    phase_ = Phase::MOVE_TO_TARGET_1;
                    RCLCPP_INFO(get_logger(),
                                "[ORBIT_BOUNDARY_HOLD] Terminal advance received, orbit_laps=0. Moving to target 1.");
                } else {
                    start_orbit();
                    phase_ = Phase::ORBIT;
                    RCLCPP_INFO(get_logger(), "[ORBIT_BOUNDARY_HOLD] Terminal advance received. Starting orbit for %d lap(s).",
                                orbit_laps());
                }
            }
            break;

        case Phase::ORBIT:
            run_orbit_step();
            break;

        case Phase::ORBIT_FINISH_HOLD:
            publish_position(orbit_boundary);
            if (wait_for_timed_hold(
                    orbit_boundary, orbit_arrival_tolerance(), orbit_finish_hold_seconds())) {
                phase_ = Phase::MOVE_TO_TARGET_1;
                reset_hold();
                RCLCPP_INFO(get_logger(),
                            "[ORBIT_FINISH_HOLD] Hold complete. Moving to target 1.");
            }
            break;

        case Phase::MOVE_TO_TARGET_1:
            publish_position(target1);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[MOVE_TO_TARGET_1] Flying to target %d at %.2f %.2f %.2f.",
                                 target1_index_, target1.x, target1.y, target1.z);
            if (is_at_target(target1, arrival_tolerance())) {
                phase_ = Phase::TARGET_1_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(),
                            "[MOVE_TO_TARGET_1] Reached target %d. Holding before GPIO12 first action.",
                            target1_index_);
            }
            break;

        case Phase::TARGET_1_HOLD:
            publish_position(target1);
            if (wait_for_timed_hold_then_terminal_advance(target1, arrival_tolerance(), target_hold_seconds())) {
                phase_ = Phase::TARGET_1_SERVO_ACTION;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[TARGET_1_HOLD] Terminal advance received. Triggering GPIO12 first action.");
            }
            break;

        case Phase::TARGET_1_SERVO_ACTION:
            publish_position(target1);
            if (start_servo_when_at_target(
                    target1, arrival_tolerance(), ServoChannel::GPIO12,
                    get_parameter("gpio12_first_angle_deg").as_double(),
                    "target 1 GPIO12 first action")) {
                phase_ = Phase::TARGET_1_POST_SERVO_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[TARGET_1_SERVO_ACTION] Hovering after GPIO12 first action.");
            }
            break;

        case Phase::TARGET_1_POST_SERVO_HOLD:
            publish_position(target1);
            if (wait_for_timed_hold(target1, arrival_tolerance(), servo_post_hold_seconds())) {
                phase_ = Phase::MOVE_TO_TARGET_2;
                reset_hold();
                RCLCPP_INFO(get_logger(),
                            "[TARGET_1_POST_SERVO_HOLD] Hold complete. Moving to target 2.");
            }
            break;

        case Phase::MOVE_TO_TARGET_2:
            publish_position(target2);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[MOVE_TO_TARGET_2] Flying to target %d at %.2f %.2f %.2f.",
                                 target2_index_, target2.x, target2.y, target2.z);
            if (is_at_target(target2, arrival_tolerance())) {
                phase_ = Phase::TARGET_2_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(),
                            "[MOVE_TO_TARGET_2] Reached target %d. Holding before GPIO12 second action.",
                            target2_index_);
            }
            break;

        case Phase::TARGET_2_HOLD:
            publish_position(target2);
            if (wait_for_timed_hold_then_terminal_advance(target2, arrival_tolerance(), target_hold_seconds())) {
                phase_ = Phase::TARGET_2_SERVO_ACTION;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[TARGET_2_HOLD] Terminal advance received. Triggering GPIO12 second action.");
            }
            break;

        case Phase::TARGET_2_SERVO_ACTION:
            publish_position(target2);
            if (start_servo_when_at_target(
                    target2, arrival_tolerance(), ServoChannel::GPIO12,
                    get_parameter("gpio12_second_angle_deg").as_double(),
                    "target 2 GPIO12 second action")) {
                phase_ = Phase::TARGET_2_POST_SERVO_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[TARGET_2_SERVO_ACTION] Hovering after GPIO12 second action.");
            }
            break;

        case Phase::TARGET_2_POST_SERVO_HOLD:
            publish_position(target2);
            if (wait_for_timed_hold(target2, arrival_tolerance(), servo_post_hold_seconds())) {
                phase_ = Phase::MOVE_TO_SPECIAL_TARGET;
                reset_hold();
                RCLCPP_INFO(get_logger(),
                            "[TARGET_2_POST_SERVO_HOLD] Hold complete. Moving to special target.");
            }
            break;

        case Phase::MOVE_TO_SPECIAL_TARGET:
            publish_position(special);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[MOVE_TO_SPECIAL_TARGET] Flying to special target %.2f %.2f %.2f.",
                                 special.x, special.y, special.z);
            if (is_at_target(special, arrival_tolerance())) {
                phase_ = Phase::SPECIAL_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(),
                            "[MOVE_TO_SPECIAL_TARGET] Reached special target. Holding before GPIO13 action.");
            }
            break;

        case Phase::SPECIAL_HOLD:
            publish_position(special);
            if (wait_for_timed_hold_then_terminal_advance(special, arrival_tolerance(), special_hold_seconds())) {
                phase_ = Phase::SPECIAL_SERVO_ACTION;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[SPECIAL_HOLD] Terminal advance received. Triggering GPIO13 action.");
            }
            break;

        case Phase::SPECIAL_SERVO_ACTION:
            publish_position(special);
            if (start_servo_when_at_target(
                    special, arrival_tolerance(), ServoChannel::GPIO13,
                    get_parameter("gpio13_special_angle_deg").as_double(),
                    "special target GPIO13 action")) {
                phase_ = Phase::SPECIAL_POST_SERVO_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[SPECIAL_SERVO_ACTION] Hovering after GPIO13 action.");
            }
            break;

        case Phase::SPECIAL_POST_SERVO_HOLD:
            publish_position(special);
            if (wait_for_timed_hold(special, arrival_tolerance(), servo_post_hold_seconds())) {
                phase_ = Phase::MOVE_LEFT_BEFORE_LAND;
                reset_hold();
                RCLCPP_INFO(get_logger(),
                            "[SPECIAL_POST_SERVO_HOLD] Hold complete. Moving left before landing.");
            }
            break;

        case Phase::MOVE_LEFT_BEFORE_LAND:
            publish_position(pre_land);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[MOVE_LEFT_BEFORE_LAND] Flying to %.2f %.2f %.2f before landing.",
                                 pre_land.x, pre_land.y, pre_land.z);
            if (is_at_target(pre_land, arrival_tolerance())) {
                phase_ = Phase::PRE_LAND_HOLD;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[MOVE_LEFT_BEFORE_LAND] Reached pre-land point. Holding before landing.");
            }
            break;

        case Phase::PRE_LAND_HOLD:
            publish_position(pre_land);
            if (wait_for_timed_hold(pre_land, arrival_tolerance(), pre_land_hold_seconds())) {
                phase_ = Phase::LANDING;
                reset_hold();
                RCLCPP_INFO(get_logger(), "[PRE_LAND_HOLD] Hold complete. Landing.");
            }
            break;

        case Phase::LANDING:
            request_land();
            if (!current_state_.armed) {
                phase_ = Phase::FINISHED;
                RCLCPP_INFO(get_logger(), "[LANDING] Disarmed. Mission finished.");
            }
            break;

        case Phase::FINISHED:
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[FINISHED] Mission finished.");
            break;
        }
    }

    void ensure_offboard_and_armed()
    {
        if (current_state_.mode != "OFFBOARD") {
            if ((now() - last_request_time_).seconds() <= 2.0) {
                return;
            }

            if (get_parameter("auto_set_mode").as_bool()) {
                if (!set_mode_client_->service_is_ready()) {
                    RCLCPP_WARN(get_logger(), "set_mode service not ready.");
                } else {
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "OFFBOARD";
                    set_mode_client_->async_send_request(req);
                    RCLCPP_INFO(get_logger(), "Requesting OFFBOARD mode...");
                }
            } else {
                RCLCPP_INFO(get_logger(), "Waiting for manual OFFBOARD switch...");
            }
            last_request_time_ = now();
            return;
        }

        if (!current_state_.armed) {
            if ((now() - last_request_time_).seconds() <= 1.0) {
                return;
            }

            if (get_parameter("auto_arm").as_bool()) {
                if (!arming_client_->service_is_ready()) {
                    RCLCPP_WARN(get_logger(), "arming service not ready.");
                } else {
                    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    req->value = true;
                    arming_client_->async_send_request(req);
                    RCLCPP_INFO(get_logger(), "Requesting arming...");
                }
            } else {
                RCLCPP_INFO(get_logger(), "Waiting for manual arming...");
            }
            last_request_time_ = now();
        }
    }

    void run_orbit_step()
    {
        const double total = orbit_total_length();
        if (total <= 0.0) {
            phase_ = Phase::MOVE_TO_TARGET_1;
            RCLCPP_INFO(get_logger(), "[ORBIT] No orbit length requested. Moving to target 1.");
            return;
        }

        if (!pose_received_) {
            publish_trajectory(sample_orbit(orbit_s_), false, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for local position during orbit.");
            return;
        }

        const auto t = now();
        const double dt = std::clamp((t - last_orbit_update_time_).seconds(), 0.001, 0.1);
        last_orbit_update_time_ = t;

        const Projection projection = project_current_position(orbit_s_);
        const bool fixed_trajectory_progress =
            get_parameter("fixed_trajectory_progress").as_bool();
        const double speed_scale = final_lap_speed_scale(orbit_s_);
        const double scaled_orbit_speed = orbit_speed() * speed_scale;
        if (fixed_trajectory_progress) {
            orbit_s_ = std::min(total, orbit_s_ + scaled_orbit_speed * dt);
        } else {
            const double projected_s = std::max(orbit_s_, projection.distance);
            const double max_projection_speed =
                scaled_orbit_speed *
                std::max(0.1, get_parameter("projection_speed_multiplier").as_double());
            orbit_s_ = std::min(projected_s, orbit_s_ + max_projection_speed * dt);
        }

        const double max_error = std::max(0.05, get_parameter("max_tracking_error").as_double());
        double recover_error = get_parameter("tracking_recover_error").as_double();
        if (recover_error <= 0.0 || recover_error >= max_error) {
            recover_error = max_error * 0.65;
        }

        if (fixed_trajectory_progress) {
            tracking_recovery_active_ = false;
        } else if (projection.error > max_error) {
            tracking_recovery_active_ = true;
        } else if (tracking_recovery_active_ && projection.error <= recover_error) {
            tracking_recovery_active_ = false;
        }

        const double desired_lookahead =
            (fixed_trajectory_progress || tracking_recovery_active_) ? 0.0 : lookahead_distance();
        const double ramp_rate = std::max(0.02, get_parameter("lookahead_ramp_rate").as_double());
        active_lookahead_ = approach(active_lookahead_, desired_lookahead, ramp_rate * dt);

        const bool use_lookahead = !tracking_recovery_active_ && active_lookahead_ > 0.01;
        const double target_s = std::min(orbit_s_ + active_lookahead_, total);
        const TrajectoryPoint target = sample_orbit(target_s);
        const bool keep_recovery_velocity =
            get_parameter("keep_velocity_feedforward_in_recovery").as_bool();
        const bool use_velocity =
            velocity_feedforward_enabled() && (use_lookahead || keep_recovery_velocity);
        const double velocity_ramp_rate =
            std::max(0.05, get_parameter("velocity_feedforward_ramp_rate").as_double());
        active_velocity_feedforward_scale_ = approach(
            active_velocity_feedforward_scale_, use_velocity ? 1.0 : 0.0,
            velocity_ramp_rate * dt);
        publish_trajectory(target, use_velocity && active_velocity_feedforward_scale_ > 0.01,
                           active_velocity_feedforward_scale_ * final_lap_speed_scale(target_s));

        if (tracking_recovery_active_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Cross-track error %.2f m > %.2f m, reducing lookahead.",
                                 projection.error, max_error);
        }

        if (orbit_s_ >= total) {
            phase_ = Phase::ORBIT_FINISH_HOLD;
            reset_hold();
            RCLCPP_INFO(get_logger(),
                        "[ORBIT] Orbit complete. Holding lower boundary center for %.1f s.",
                        orbit_finish_hold_seconds());
        }
    }

    void start_orbit()
    {
        orbit_s_ = 0.0;
        active_lookahead_ = 0.0;
        active_velocity_feedforward_scale_ = 0.0;
        tracking_recovery_active_ = false;
        last_orbit_update_time_ = now();
    }

    void start_terminal_advance_thread()
    {
        std::thread([this]() {
            std::string line;
            while (rclcpp::ok() && std::getline(std::cin, line)) {
                terminal_advance_count_.fetch_add(1, std::memory_order_relaxed);
            }
        }).detach();
    }

    bool wait_for_terminal_advance(const Target& target, double tolerance)
    {
        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for local position while holding.");
            return false;
        }

        if (!is_at_target(target, tolerance)) {
            hold_started_ = false;
            return false;
        }

        if (!hold_started_) {
            hold_started_ = true;
            hold_start_advance_count_ = terminal_advance_count_.load(std::memory_order_relaxed);
            RCLCPP_INFO(get_logger(),
                        "Reached %.2f %.2f %.2f. Hovering until terminal input, then advancing.",
                        target.x, target.y, target.z);
            return false;
        }

        return terminal_advance_count_.load(std::memory_order_relaxed) != hold_start_advance_count_;
    }

    void reset_hold()
    {
        hold_started_ = false;
        hold_ready_for_terminal_advance_ = false;
    }

    bool wait_for_timed_hold_then_terminal_advance(
        const Target& target, double tolerance, double hold_seconds)
    {
        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for local position while holding.");
            return false;
        }

        if (!is_at_target(target, tolerance)) {
            hold_started_ = false;
            return false;
        }

        if (!hold_started_) {
            hold_started_ = true;
            hold_ready_for_terminal_advance_ = false;
            hold_start_time_ = now();
            RCLCPP_INFO(get_logger(),
                        "Reached %.2f %.2f %.2f. Hovering for %.1f s before terminal input.",
                        target.x, target.y, target.z, hold_seconds);
            return false;
        }

        if (!hold_ready_for_terminal_advance_) {
            if ((now() - hold_start_time_).seconds() < hold_seconds) {
                return false;
            }

            hold_ready_for_terminal_advance_ = true;
            hold_start_advance_count_ = terminal_advance_count_.load(std::memory_order_relaxed);
            RCLCPP_INFO(get_logger(),
                        "Hold complete at %.2f %.2f %.2f. Press Enter to run servo action.",
                        target.x, target.y, target.z);
            return false;
        }

        return terminal_advance_count_.load(std::memory_order_relaxed) != hold_start_advance_count_;
    }

    bool wait_for_timed_hold(const Target& target, double tolerance, double hold_seconds)
    {
        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for local position while holding.");
            return false;
        }

        if (!is_at_target(target, tolerance)) {
            hold_started_ = false;
            return false;
        }

        if (!hold_started_) {
            hold_started_ = true;
            hold_start_time_ = now();
            RCLCPP_INFO(get_logger(),
                        "Reached %.2f %.2f %.2f. Hovering for %.1f s.",
                        target.x, target.y, target.z, hold_seconds);
            return hold_seconds <= 0.0;
        }

        return (now() - hold_start_time_).seconds() >= hold_seconds;
    }

    bool start_servo_when_at_target(
        const Target& target, double tolerance, ServoChannel channel, double angle_deg,
        const std::string& label)
    {
        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for local position before servo control.");
            return false;
        }

        if (!is_at_target(target, tolerance)) {
            return false;
        }

        start_servo_command(channel, angle_deg, label);
        return true;
    }

    void start_servo_command(ServoChannel channel, double angle_deg, const std::string& label)
    {
        if (!get_parameter("enable_servo_actions").as_bool()) {
            RCLCPP_INFO(get_logger(), "Servo action %s %.1f deg skipped because enable_servo_actions=false.",
                        label.c_str(), angle_deg);
            return;
        }

        active_servo_channel_ = channel;
        active_servo_angle_deg_ = angle_deg;
        active_servo_until_ = now() + rclcpp::Duration::from_seconds(
            std::max(0.1, get_parameter("servo_command_repeat_seconds").as_double()));

        const auto publisher = servo_publisher_for(channel);
        if (publisher && publisher->get_subscription_count() == 0) {
            RCLCPP_WARN(get_logger(), "Servo action %s %.1f deg has no matching servo subscriber.",
                        label.c_str(), angle_deg);
        }

        const double updated_tau = simple_tau() + 0.1;
        set_parameter(rclcpp::Parameter("setpoint_lowpass_tau", updated_tau));
        RCLCPP_INFO(get_logger(), "Servo action: %s %.1f deg. setpoint_lowpass_tau now %.2f.",
                    label.c_str(), angle_deg, updated_tau);
        publish_servo_command(channel, angle_deg);
    }

    void publish_active_servo_command()
    {
        if (active_servo_channel_ == ServoChannel::NONE || now() > active_servo_until_) {
            active_servo_channel_ = ServoChannel::NONE;
            return;
        }

        publish_servo_command(active_servo_channel_, active_servo_angle_deg_);
    }

    void publish_servo_command(ServoChannel channel, double angle_deg)
    {
        const auto publisher = servo_publisher_for(channel);
        if (!publisher) {
            return;
        }

        std_msgs::msg::Float64 msg;
        msg.data = angle_deg;
        publisher->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_publisher_for(ServoChannel channel) const
    {
        if (channel == ServoChannel::GPIO12) {
            return gpio12_angle_pub_;
        }
        if (channel == ServoChannel::GPIO13) {
            return gpio13_angle_pub_;
        }
        return nullptr;
    }

    void request_land()
    {
        if (land_mode_requested_) {
            return;
        }

        if (!set_mode_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Land requested but set_mode service is not ready.");
            return;
        }

        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = get_parameter("land_mode").as_string();
        set_mode_client_->async_send_request(req);
        land_mode_requested_ = true;
        RCLCPP_INFO(get_logger(), "Requesting %s mode...", req->custom_mode.c_str());
    }

    void check_rc_stop(const mavros_msgs::msg::RCIn& msg)
    {
        if (!get_parameter("enable_rc_stop").as_bool()) {
            return;
        }

        const int channel = static_cast<int>(get_parameter("rc_stop_channel").as_int());
        if (channel <= 0 || static_cast<size_t>(channel) > msg.channels.size()) {
            return;
        }

        const int pwm = msg.channels[static_cast<size_t>(channel - 1)];
        const int threshold = static_cast<int>(get_parameter("rc_stop_pwm_threshold").as_int());
        const bool stop_when_high = get_parameter("rc_stop_when_high").as_bool();
        const bool triggered = stop_when_high ? pwm >= threshold : pwm <= threshold;

        if (triggered && !emergency_stop_) {
            emergency_stop_ = true;
            RCLCPP_ERROR(get_logger(),
                         "RC stop triggered on channel %d with PWM %d. Stopping offboard mission.",
                         channel, pwm);
        }
    }

    bool is_at_target(const Target& target, double tolerance) const
    {
        if (!pose_received_) {
            return false;
        }

        const auto& p = current_pose_.pose.position;
        const double dx = p.x - target.x;
        const double dy = p.y - target.y;
        const double dz = p.z - target.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) <= tolerance;
    }

    void publish_position(const Target& target)
    {
        update_simple_lowpass_setpoint(target);

        mavros_msgs::msg::PositionTarget msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        msg.position.x = shaped_x_;
        msg.position.y = shaped_y_;
        msg.position.z = shaped_z_;

        if (initial_yaw_captured_) {
            msg.yaw = initial_yaw_;
        } else {
            msg.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
        }

        setpoint_pub_->publish(msg);
    }

    void publish_trajectory(const TrajectoryPoint& target, bool use_velocity,
                            double velocity_scale = 1.0)
    {
        publish_trajectory(target, use_velocity, velocity_scale,
                           get_parameter("orbit_setpoint_lowpass_tau").as_double(),
                           get_parameter("orbit_setpoint_snap_tolerance").as_double());
    }

    void publish_trajectory(const TrajectoryPoint& target, bool use_velocity,
                            double velocity_scale, double tau, double snap_tolerance)
    {
        update_lowpass_setpoint(target, tau, snap_tolerance);

        mavros_msgs::msg::PositionTarget msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        if (!use_velocity) {
            msg.type_mask |=
                mavros_msgs::msg::PositionTarget::IGNORE_VX |
                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ;
        }

        msg.position.x = shaped_x_;
        msg.position.y = shaped_y_;
        msg.position.z = shaped_z_;

        const double clamped_velocity_scale = std::clamp(velocity_scale, 0.0, 1.0);
        msg.velocity.x = target.vx * clamped_velocity_scale;
        msg.velocity.y = target.vy * clamped_velocity_scale;
        msg.velocity.z = target.vz * clamped_velocity_scale;

        if (initial_yaw_captured_) {
            msg.yaw = initial_yaw_;
        } else {
            msg.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
        }

        setpoint_pub_->publish(msg);
    }

    void update_lowpass_setpoint(const TrajectoryPoint& target, double tau, double snap_tolerance)
    {
        const auto t = now();
        if (get_parameter("strict_path_setpoint").as_bool()) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
            shaped_setpoint_initialized_ = true;
            last_shaping_time_ = t;
            return;
        }

        if (!shaped_setpoint_initialized_) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
            shaped_setpoint_initialized_ = true;
            last_shaping_time_ = t;
            return;
        }

        const double dt = std::clamp((t - last_shaping_time_).seconds(), 0.001, 0.1);
        last_shaping_time_ = t;

        const double previous_x = shaped_x_;
        const double previous_y = shaped_y_;
        const double dx = target.x - shaped_x_;
        const double dy = target.y - shaped_y_;
        const double dz = target.z - shaped_z_;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance <= std::max(0.0, snap_tolerance)) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
        } else if (tau <= 0.0) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
        } else {
            const double alpha = dt / (tau + dt);
            shaped_x_ += alpha * dx;
            shaped_y_ += alpha * dy;
            shaped_z_ += alpha * dz;
        }

        const double max_xy_speed = get_parameter("setpoint_max_xy_speed").as_double();
        const double max_step = max_xy_speed * dt;
        const double step_x = shaped_x_ - previous_x;
        const double step_y = shaped_y_ - previous_y;
        const double step_distance = std::sqrt(step_x * step_x + step_y * step_y);
        if (max_xy_speed > 0.0 && step_distance > max_step && step_distance > 1e-6) {
            const double scale = max_step / step_distance;
            shaped_x_ = previous_x + step_x * scale;
            shaped_y_ = previous_y + step_y * scale;
        }
    }

    void update_simple_lowpass_setpoint(const Target& target)
    {
        const auto current_time = now();
        if (!shaped_setpoint_initialized_) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
            shaped_setpoint_initialized_ = true;
            last_shaping_time_ = current_time;
            return;
        }

        const double dt = std::clamp((current_time - last_shaping_time_).seconds(), 0.001, 0.1);
        last_shaping_time_ = current_time;

        const double dx = target.x - shaped_x_;
        const double dy = target.y - shaped_y_;
        const double dz = target.z - shaped_z_;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double snap_tolerance = std::max(0.0, simple_snap_tolerance());

        if (distance <= snap_tolerance) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
            return;
        }

        const double tau = simple_tau();
        if (tau <= 0.0) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
            return;
        }

        const double alpha = dt / (tau + dt);
        shaped_x_ += alpha * dx;
        shaped_y_ += alpha * dy;
        shaped_z_ += alpha * dz;
    }

    TrajectoryPoint sample_orbit(double distance) const
    {
        const double side = std::max(0.1, get_parameter("square_size").as_double());
        const double radius = std::clamp(
            get_parameter("corner_radius").as_double(), 0.0, side * 0.5);
        const double ox = get_parameter("origin_x").as_double();
        const double oy = get_parameter("origin_y").as_double();
        const double z = flight_z();
        const double speed = -orbit_speed();
        const double lap_length = orbit_lap_length();

        if (lap_length <= 0.0) {
            return {ox, oy, z, 0.0, 0.0, 0.0};
        }

        // Treat the lower boundary center as both the orbit start and lap endpoint.
        // Mission orbit is clockwise, so increasing mission distance walks the
        // geometric path in the reverse direction from the original sampler.
        const double start_offset = std::max(0.0, side * 0.5 - radius);
        double s = std::fmod(start_offset - std::max(0.0, distance), lap_length);
        if (s < 0.0) {
            s += lap_length;
        }
        if (radius <= 1e-6) {
            auto line = [z, speed](double x, double y, double tx, double ty) {
                return TrajectoryPoint{x, y, z, speed * tx, speed * ty, 0.0};
            };

            if (s < side) {
                return line(ox + s, oy, 1.0, 0.0);
            }
            s -= side;

            if (s < side) {
                return line(ox + side, oy + s, 0.0, 1.0);
            }
            s -= side;

            if (s < side) {
                return line(ox + side - s, oy + side, -1.0, 0.0);
            }
            s -= side;

            return line(ox, oy + side - s, 0.0, -1.0);
        }

        const double straight = std::max(0.0, side - 2.0 * radius);
        const double arc = 0.5 * M_PI * radius;

        auto line = [z, speed](double x, double y, double tx, double ty) {
            return TrajectoryPoint{x, y, z, speed * tx, speed * ty, 0.0};
        };

        auto arc_point = [z, speed, radius](double ccx, double ccy, double angle) {
            const double x = ccx + radius * std::cos(angle);
            const double y = ccy + radius * std::sin(angle);
            return TrajectoryPoint{x, y, z, -speed * std::sin(angle), speed * std::cos(angle), 0.0};
        };

        if (s < straight) {
            return line(ox + radius + s, oy, 1.0, 0.0);
        }
        s -= straight;

        if (s < arc) {
            return arc_point(ox + side - radius, oy + radius,
                             -0.5 * M_PI + s / radius);
        }
        s -= arc;

        if (s < straight) {
            return line(ox + side, oy + radius + s, 0.0, 1.0);
        }
        s -= straight;

        if (s < arc) {
            return arc_point(ox + side - radius, oy + side - radius, s / radius);
        }
        s -= arc;

        if (s < straight) {
            return line(ox + side - radius - s, oy + side, -1.0, 0.0);
        }
        s -= straight;

        if (s < arc) {
            return arc_point(ox + radius, oy + side - radius,
                             0.5 * M_PI + s / radius);
        }
        s -= arc;

        if (s < straight) {
            return line(ox, oy + side - radius - s, 0.0, -1.0);
        }
        s -= straight;

        if (s < arc) {
            return arc_point(ox + radius, oy + radius, M_PI + s / radius);
        }

        return line(ox + radius, oy, 1.0, 0.0);
    }

    double orbit_lap_length() const
    {
        const double side = std::max(0.1, get_parameter("square_size").as_double());
        const double radius = std::clamp(
            get_parameter("corner_radius").as_double(), 0.0, side * 0.5);
        return 4.0 * (side - 2.0 * radius) + 2.0 * M_PI * radius;
    }

    double orbit_total_length() const
    {
        return orbit_lap_length() * static_cast<double>(orbit_laps());
    }

    int orbit_laps() const
    {
        return std::max(0, static_cast<int>(get_parameter("orbit_laps").as_int()));
    }

    double orbit_speed() const
    {
        return std::max(0.05, get_parameter("orbit_speed").as_double());
    }

    double final_lap_speed_scale(double distance) const
    {
        const double lap_length = orbit_lap_length();
        const double total = orbit_total_length();
        if (lap_length <= 0.0 || total <= 0.0 || distance < total - lap_length) {
            return 1.0;
        }

        const double min_scale = std::clamp(
            get_parameter("orbit_final_lap_min_speed_scale").as_double(), 0.05, 1.0);
        const double final_lap_progress =
            std::clamp((distance - (total - lap_length)) / lap_length, 0.0, 1.0);
        const double smooth_progress =
            final_lap_progress * final_lap_progress * (3.0 - 2.0 * final_lap_progress);
        return 1.0 - (1.0 - min_scale) * smooth_progress;
    }

    double orbit_finish_hold_seconds() const
    {
        return std::max(0.0, get_parameter("orbit_finish_hold_seconds").as_double());
    }

    double lookahead_distance() const
    {
        return std::max(0.02, get_parameter("lookahead_distance").as_double());
    }

    bool velocity_feedforward_enabled() const
    {
        return get_parameter("enable_velocity_feedforward").as_bool();
    }

    Projection project_current_position(double center_distance) const
    {
        const auto& p = current_pose_.pose.position;
        const double total = orbit_total_length();
        const double window = std::max(0.2, get_parameter("projection_search_window").as_double());
        const double min_s = std::max(0.0, center_distance - window);
        const double max_s = std::min(total, center_distance + window);

        double best_s = min_s;
        double best_error_sq = std::numeric_limits<double>::infinity();

        auto evaluate = [this, &p, &best_s, &best_error_sq](double s) {
            const TrajectoryPoint point = sample_orbit(s);
            const double dx = p.x - point.x;
            const double dy = p.y - point.y;
            const double error_sq = dx * dx + dy * dy;
            if (error_sq < best_error_sq) {
                best_error_sq = error_sq;
                best_s = s;
            }
        };

        constexpr int coarse_samples = 80;
        for (int i = 0; i <= coarse_samples; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(coarse_samples);
            evaluate(min_s + (max_s - min_s) * ratio);
        }

        double refine_half_width = std::max(0.02, (max_s - min_s) / coarse_samples);
        for (int pass = 0; pass < 4; ++pass) {
            const double start = std::max(0.0, best_s - refine_half_width);
            const double end = std::min(total, best_s + refine_half_width);
            for (int i = 0; i <= 12; ++i) {
                const double ratio = static_cast<double>(i) / 12.0;
                evaluate(start + (end - start) * ratio);
            }
            refine_half_width *= 0.35;
        }

        return {best_s, std::sqrt(best_error_sq)};
    }

    Target target_from_index(int index) const
    {
        static constexpr std::array<Target, 4> targets{{
            {1.6, 1.8, 0.5},
            {-1.6, 1.8, 0.5},
            {1.6, 3.6, 0.5},
            {-1.6, 3.6, 0.5},
        }};

        Target target = targets[static_cast<size_t>(std::clamp(index, 1, 4) - 1)];
        target.z = flight_z();
        return target;
    }

    Target obstacle_boundary_target() const
    {
        const double ox = get_parameter("origin_x").as_double();
        const double oy = get_parameter("origin_y").as_double();
        const double side = std::max(0.1, get_parameter("square_size").as_double());
        return {ox + side * 0.5, oy, flight_z()};
    }

    Target pre_land_target(const Target& special) const
    {
        const double left_offset = std::max(0.0, get_parameter("pre_land_left_offset").as_double());
        return {special.x - left_offset, special.y, special.z};
    }

    int read_target_index(const std::string& name, int fallback)
    {
        const int raw = get_parameter(name).as_int();
        const int clamped = std::clamp(raw, 1, 4);
        if (raw != clamped) {
            RCLCPP_WARN(get_logger(), "%s=%d is outside 1-4. Using %d.",
                        name.c_str(), raw, clamped);
            return clamped;
        }
        return raw > 0 ? raw : fallback;
    }

    double flight_z() const
    {
        return get_parameter("flight_z").as_double();
    }

    double arrival_tolerance() const
    {
        return std::max(0.02, get_parameter("arrival_tolerance").as_double());
    }

    double orbit_arrival_tolerance() const
    {
        return std::max(0.02, get_parameter("orbit_arrival_tolerance").as_double());
    }

    double simple_tau() const
    {
        return get_parameter("setpoint_lowpass_tau").as_double();
    }

    double simple_snap_tolerance() const
    {
        return get_parameter("setpoint_snap_tolerance").as_double();
    }

    double target_hold_seconds() const
    {
        return std::max(0.0, get_parameter("target_hold_seconds").as_double());
    }

    double special_hold_seconds() const
    {
        return std::max(0.0, get_parameter("special_hold_seconds").as_double());
    }

    double servo_post_hold_seconds() const
    {
        return std::max(0.0, get_parameter("servo_post_hold_seconds").as_double());
    }

    double pre_land_hold_seconds() const
    {
        return std::max(0.0, get_parameter("pre_land_hold_seconds").as_double());
    }

    static double approach(double current, double target, double max_delta)
    {
        if (current < target) {
            return std::min(target, current + max_delta);
        }
        if (current > target) {
            return std::max(target, current - max_delta);
        }
        return current;
    }

    static double yaw_from_pose(const geometry_msgs::msg::PoseStamped& pose)
    {
        const auto& q = pose.pose.orientation;
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    Phase phase_{Phase::INIT_STREAM};
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_received_{false};

    int target1_index_{1};
    int target2_index_{2};
    int init_stream_count_{0};
    int hold_start_advance_count_{0};

    double initial_yaw_{0.0};
    bool initial_yaw_captured_{false};
    bool shaped_setpoint_initialized_{false};
    bool hold_started_{false};
    bool hold_ready_for_terminal_advance_{false};
    bool tracking_recovery_active_{false};
    bool land_mode_requested_{false};
    bool emergency_stop_{false};
    ServoChannel active_servo_channel_{ServoChannel::NONE};
    std::atomic<int> terminal_advance_count_{0};

    double shaped_x_{0.0};
    double shaped_y_{0.0};
    double shaped_z_{0.0};
    double orbit_s_{0.0};
    double active_lookahead_{0.0};
    double active_velocity_feedforward_scale_{0.0};
    double active_servo_angle_deg_{0.0};

    rclcpp::Time last_request_time_;
    rclcpp::Time last_shaping_time_;
    rclcpp::Time last_orbit_update_time_;
    rclcpp::Time hold_start_time_;
    rclcpp::Time active_servo_until_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gpio12_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gpio13_angle_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardMissionNode>();

    while (rclcpp::ok() && !node->is_connected()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(100ms);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "Waiting for FCU connection...");
    }

    RCLCPP_INFO(node->get_logger(), "FCU connected!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



/*
带载滤波：0.8
不带：1.1
*/
