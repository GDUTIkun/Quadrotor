#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

class WaypointOffboardNode : public rclcpp::Node {
public:
    WaypointOffboardNode()
        : Node("waypoint_offboard_node")
    {
        declare_parameter<bool>("auto_set_mode", true);
        declare_parameter<bool>("auto_arm", true);
        declare_parameter<std::string>("path_topic", "ground_station_flight_path");
        declare_parameter<std::string>("yolo_service_name", "/yolo/capture_and_detect");
        declare_parameter<double>("arrival_tolerance", 0.12);
        declare_parameter<double>("setpoint_lowpass_min_tau", 0.0);
        declare_parameter<double>("setpoint_lowpass_near_error", 0.5);
        declare_parameter<double>("setpoint_lowpass_near_tau", 0.7);
        declare_parameter<double>("setpoint_lowpass_far_error", 2.0);
        declare_parameter<double>("setpoint_lowpass_far_tau", 1.3);
        declare_parameter<double>("setpoint_snap_tolerance", 0.02);
        // declare_parameter<bool>("enable_terminal_input", true);
        declare_parameter<bool>("enable_terminal_input", false);
        declare_parameter<bool>("auto_advance_waypoints", true);
        declare_parameter<bool>("return_home_and_land_after_mission", true);
        declare_parameter<double>("landing_ground_z", 0.25);
        declare_parameter<double>("landing_descent_angle_deg", 45.0);
        declare_parameter<double>("landing_path_speed", 0.25);
        declare_parameter<double>("landing_min_path_speed", 0.08);
        declare_parameter<double>("landing_slowdown_distance", 0.8);
        declare_parameter<double>("landing_ground_z_tolerance", 0.15);
        declare_parameter<double>("landing_xy_tolerance", 0.25);
        declare_parameter<double>("landing_settle_time", 1.0);
        declare_parameter<double>("landing_touchdown_sink", 0.12);
        declare_parameter<bool>("request_auto_land_before_disarm", true);
        declare_parameter<double>("disarm_request_interval", 1.0);

        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                current_state_ = *msg;
            });

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "mavros/local_position/pose",
            rclcpp::SensorDataQoS(),
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

        const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            get_parameter("path_topic").as_string(), path_qos,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                on_path(*msg);
            });

        setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
            "mavros/setpoint_raw/local", 10);

        set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        yolo_client_ = create_client<std_srvs::srv::Trigger>(
            get_parameter("yolo_service_name").as_string());

        last_request_time_ = now();
        start_terminal_input();
        timer_ = create_wall_timer(20ms, std::bind(&WaypointOffboardNode::timer_callback, this));

        RCLCPP_INFO(get_logger(),
                    "Waiting for waypoint path on [%s]. Vehicle will not take off before a path is received.",
                    get_parameter("path_topic").as_string().c_str());
        RCLCPP_INFO(get_logger(),
                    "YOLO snapshot service will be requested once after each non-takeoff waypoint: [%s].",
                    get_parameter("yolo_service_name").as_string().c_str());
    }

    ~WaypointOffboardNode() override
    {
        terminal_thread_running_ = false;
    }

    bool is_connected() const
    {
        return current_state_.connected;
    }

private:
    enum class Phase {
        WAITING_PATH,
        STARTING_OFFBOARD,
        FLYING_WAYPOINT,
        HOLDING_WAYPOINT,
        RETURNING_HOME_APPROACH,
        LANDING_45_DEG,
        LANDING_SETTLE,
        DISARMING,
        MISSION_COMPLETE,
    };

    struct Target {
        double x;
        double y;
        double z;
    };

    void on_path(const nav_msgs::msg::Path& msg)
    {
        if (msg.poses.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty waypoint path. Ignoring.");
            return;
        }

        std::vector<Target> new_waypoints;
        new_waypoints.reserve(msg.poses.size());
        for (const auto& pose : msg.poses) {
            new_waypoints.push_back(Target{
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            });
        }

        {
            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            waypoints_ = std::move(new_waypoints);
            active_index_ = 0;
            waypoint_reached_ = false;
            advance_requested_ = false;
            landing_plan_initialized_ = false;
            phase_.store(Phase::STARTING_OFFBOARD);
        }

        const auto first = active_target();
        RCLCPP_INFO(get_logger(),
                    "Received %zu waypoints. First waypoint is takeoff point: %.3f %.3f %.3f.",
                    waypoint_count(), first.x, first.y, first.z);
    }

    void timer_callback()
    {
        if (!current_state_.connected) {
            return;
        }

        const Phase phase = phase_.load();
        if (phase == Phase::WAITING_PATH) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No waypoint path yet. Holding on ground and waiting.");
            return;
        }

        if (phase == Phase::MISSION_COMPLETE) {
            publish_position(landing_plan_initialized_ ? landing_touchdown_target_ : active_target(), false);
            return;
        }

        if (phase == Phase::RETURNING_HOME_APPROACH) {
            publish_position(landing_approach_target_);

            if (!pose_received_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Waiting for local position while returning to landing approach point.");
                return;
            }

            if (is_at_target(landing_approach_target_)) {
                start_45_degree_landing();
            }
            return;
        }

        if (phase == Phase::LANDING_45_DEG) {
            const Target landing_setpoint = current_landing_setpoint();
            publish_position(landing_setpoint, false);

            if (is_landing_complete()) {
                start_landing_settle();
            }
            return;
        }

        if (phase == Phase::LANDING_SETTLE) {
            publish_position(landing_touchdown_target_, false);

            if ((now() - landing_settle_start_time_).seconds() >=
                get_parameter("landing_settle_time").as_double()) {
                start_disarming();
            }
            return;
        }

        if (phase == Phase::DISARMING) {
            publish_position(landing_touchdown_target_, false);

            if (!current_state_.armed) {
                phase_.store(Phase::MISSION_COMPLETE);
                RCLCPP_INFO(get_logger(),
                            "Vehicle disarmed. Mission complete at takeoff point: %.3f %.3f %.3f.",
                            landing_ground_target_.x, landing_ground_target_.y, landing_ground_target_.z);
                return;
            }

            request_disarm_if_needed();
            return;
        }

        const Target target = active_target();
        publish_position(target);

        if (phase == Phase::STARTING_OFFBOARD) {
            ensure_offboard_and_armed();
            if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
                RCLCPP_INFO(get_logger(),
                            "OFFBOARD and armed. Flying to waypoint 1/%zu: %.3f %.3f %.3f.",
                            waypoint_count(), target.x, target.y, target.z);
                phase_.store(Phase::FLYING_WAYPOINT);
            }
            return;
        }

        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for local position while publishing active waypoint.");
            return;
        }

        if (phase == Phase::FLYING_WAYPOINT && is_at_target(target)) {
            waypoint_reached_ = true;
            phase_.store(Phase::HOLDING_WAYPOINT);
            request_yolo_snapshot_if_needed(active_index_);

            if (active_index_ + 1U >= waypoint_count()) {
                if (get_parameter("return_home_and_land_after_mission").as_bool()) {
                    begin_return_home_landing();
                } else {
                    phase_.store(Phase::MISSION_COMPLETE);
                    RCLCPP_INFO(get_logger(),
                                "Reached final waypoint %zu/%zu. Mission complete, holding position.",
                                active_index_ + 1U, waypoint_count());
                }
                return;
            }

            if (get_parameter("auto_advance_waypoints").as_bool()) {
                advance_requested_ = true;
                RCLCPP_INFO(get_logger(),
                            "Reached waypoint %zu/%zu. Auto-advancing to the next waypoint.",
                            active_index_ + 1U, waypoint_count());
            } else if (get_parameter("enable_terminal_input").as_bool()) {
                RCLCPP_INFO(get_logger(),
                            "Reached waypoint %zu/%zu. Press Enter once to fly to the next waypoint.",
                            active_index_ + 1U, waypoint_count());
            } else {
                RCLCPP_WARN(get_logger(),
                            "Reached waypoint %zu/%zu. Holding position because auto advance and terminal input are disabled.",
                            active_index_ + 1U, waypoint_count());
            }
            return;
        }

        if (phase == Phase::HOLDING_WAYPOINT && advance_requested_) {
            advance_requested_ = false;
            waypoint_reached_ = false;
            ++active_index_;
            const Target next = active_target();
            phase_.store(Phase::FLYING_WAYPOINT);
            RCLCPP_INFO(get_logger(),
                        "Flying to waypoint %zu/%zu: %.3f %.3f %.3f.",
                        active_index_ + 1U, waypoint_count(), next.x, next.y, next.z);
        }
    }

    void begin_return_home_landing()
    {
        const Target takeoff = first_target();
        const Target final_waypoint = active_target();

        landing_ground_target_ = Target{
            takeoff.x,
            takeoff.y,
            get_parameter("landing_ground_z").as_double(),
        };
        landing_touchdown_target_ = Target{
            landing_ground_target_.x,
            landing_ground_target_.y,
            landing_ground_target_.z - std::abs(get_parameter("landing_touchdown_sink").as_double()),
        };

        const double angle_deg = std::clamp(
            std::abs(get_parameter("landing_descent_angle_deg").as_double()), 1.0, 89.0);
        const double angle_rad = angle_deg * M_PI / 180.0;
        const double descent_height = std::abs(takeoff.z - landing_ground_target_.z);
        const double approach_distance = descent_height / std::tan(angle_rad);

        double dir_x = final_waypoint.x - landing_ground_target_.x;
        double dir_y = final_waypoint.y - landing_ground_target_.y;
        double dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y);

        if (dir_norm <= 1e-3 && pose_received_) {
            dir_x = current_pose_.pose.position.x - landing_ground_target_.x;
            dir_y = current_pose_.pose.position.y - landing_ground_target_.y;
            dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y);
        }

        if (dir_norm <= 1e-3) {
            dir_x = initial_yaw_captured_ ? std::cos(initial_yaw_) : 1.0;
            dir_y = initial_yaw_captured_ ? std::sin(initial_yaw_) : 0.0;
            dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y);
        }

        landing_approach_target_ = Target{
            landing_ground_target_.x + dir_x / dir_norm * approach_distance,
            landing_ground_target_.y + dir_y / dir_norm * approach_distance,
            takeoff.z,
        };
        landing_plan_initialized_ = true;
        phase_.store(Phase::RETURNING_HOME_APPROACH);

        RCLCPP_INFO(get_logger(),
                    "Reached final waypoint %zu/%zu. Returning to landing approach point %.3f %.3f %.3f, then %.1f deg landing to takeoff point %.3f %.3f %.3f. Touchdown hold setpoint: %.3f %.3f %.3f.",
                    active_index_ + 1U, waypoint_count(),
                    landing_approach_target_.x, landing_approach_target_.y, landing_approach_target_.z,
                    angle_deg,
                    landing_ground_target_.x, landing_ground_target_.y, landing_ground_target_.z,
                    landing_touchdown_target_.x, landing_touchdown_target_.y, landing_touchdown_target_.z);
    }

    void start_45_degree_landing()
    {
        landing_start_target_ = landing_approach_target_;
        landing_progress_ = 0.0;
        landing_last_update_time_ = now();
        landing_total_distance_ = distance_between(landing_start_target_, landing_ground_target_);
        phase_.store(Phase::LANDING_45_DEG);

        RCLCPP_INFO(get_logger(),
                    "Landing approach point reached. Starting slow 45 degree landing over %.2f m.",
                    landing_total_distance_);
    }

    Target current_landing_setpoint()
    {
        update_landing_progress();
        return Target{
            landing_start_target_.x + (landing_ground_target_.x - landing_start_target_.x) * landing_progress_,
            landing_start_target_.y + (landing_ground_target_.y - landing_start_target_.y) * landing_progress_,
            landing_start_target_.z + (landing_ground_target_.z - landing_start_target_.z) * landing_progress_,
        };
    }

    void update_landing_progress()
    {
        const auto current_time = now();
        const double dt = std::clamp((current_time - landing_last_update_time_).seconds(), 0.001, 0.1);
        landing_last_update_time_ = current_time;

        if (landing_total_distance_ <= 1e-6 || landing_progress_ >= 1.0) {
            landing_progress_ = 1.0;
            return;
        }

        const double normal_speed = std::max(0.05, get_parameter("landing_path_speed").as_double());
        const double min_speed = std::clamp(
            get_parameter("landing_min_path_speed").as_double(), 0.02, normal_speed);
        const double slowdown_distance =
            std::max(0.0, get_parameter("landing_slowdown_distance").as_double());
        const double remaining_distance = landing_total_distance_ * (1.0 - landing_progress_);

        double speed = normal_speed;
        if (slowdown_distance > 1e-6 && remaining_distance < slowdown_distance) {
            const double ratio = std::clamp(remaining_distance / slowdown_distance, 0.0, 1.0);
            speed = min_speed + (normal_speed - min_speed) * ratio;
        }

        landing_progress_ = std::clamp(
            landing_progress_ + speed * dt / landing_total_distance_, 0.0, 1.0);
    }

    bool is_landing_complete() const
    {
        if (!pose_received_ || landing_progress_ < 1.0) {
            return false;
        }

        const auto& p = current_pose_.pose.position;
        const double dx = p.x - landing_ground_target_.x;
        const double dy = p.y - landing_ground_target_.y;
        const double xy_error = std::sqrt(dx * dx + dy * dy);
        const double z_error = std::abs(p.z - landing_ground_target_.z);

        return xy_error <= get_parameter("landing_xy_tolerance").as_double() &&
               z_error <= get_parameter("landing_ground_z_tolerance").as_double();
    }

    void start_landing_settle()
    {
        landing_settle_start_time_ = now();
        phase_.store(Phase::LANDING_SETTLE);
        RCLCPP_INFO(get_logger(),
                    "45 degree landing target reached. Holding touchdown setpoint %.3f %.3f %.3f for %.2f s before disarm.",
                    landing_touchdown_target_.x, landing_touchdown_target_.y, landing_touchdown_target_.z,
                    get_parameter("landing_settle_time").as_double());
    }

    void start_disarming()
    {
        last_disarm_request_time_ = now();
        phase_.store(Phase::DISARMING);
        RCLCPP_INFO(get_logger(), "Landing settle complete. Requesting land mode and disarm.");
        request_land_mode_if_needed();
        send_disarm_request();
    }

    void request_disarm_if_needed()
    {
        if ((now() - last_disarm_request_time_).seconds() <
            get_parameter("disarm_request_interval").as_double()) {
            return;
        }

        last_disarm_request_time_ = now();
        request_land_mode_if_needed();
        send_disarm_request();
    }

    void request_land_mode_if_needed()
    {
        if (!get_parameter("request_auto_land_before_disarm").as_bool() ||
            current_state_.mode == "AUTO.LAND") {
            return;
        }

        if (!set_mode_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "set_mode service not ready. AUTO.LAND request delayed.");
            return;
        }

        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "AUTO.LAND";
        set_mode_client_->async_send_request(req);
        RCLCPP_INFO(get_logger(), "Requesting AUTO.LAND before disarm...");
    }

    void send_disarm_request()
    {
        if (!arming_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "arming service not ready. Disarm request delayed.");
            return;
        }

        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = false;
        arming_client_->async_send_request(req);
        RCLCPP_INFO(get_logger(), "Requesting disarm...");
    }

    static double distance_between(const Target& lhs, const Target& rhs)
    {
        const double dx = lhs.x - rhs.x;
        const double dy = lhs.y - rhs.y;
        const double dz = lhs.z - rhs.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void request_yolo_snapshot_if_needed(std::size_t waypoint_index)
    {
        if (waypoint_index == 0U) {
            RCLCPP_INFO(get_logger(), "Takeoff waypoint reached. Skipping YOLO snapshot request.");
            return;
        }

        if (!yolo_client_->service_is_ready()) {
            RCLCPP_WARN(
                get_logger(),
                "YOLO service %s is not ready. Snapshot request for waypoint %zu skipped.",
                get_parameter("yolo_service_name").as_string().c_str(), waypoint_index + 1U);
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const std::size_t waypoint_number = waypoint_index + 1U;
        auto response_callback =
            [this, waypoint_number](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                const auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(
                        get_logger(),
                        "YOLO snapshot request accepted after waypoint %zu: %s",
                        waypoint_number, response->message.c_str());
                    return;
                }

                RCLCPP_WARN(
                    get_logger(),
                    "YOLO snapshot request rejected after waypoint %zu: %s",
                    waypoint_number, response->message.c_str());
            };

        yolo_client_->async_send_request(request, response_callback);
        RCLCPP_INFO(
            get_logger(),
            "YOLO snapshot request sent after reaching waypoint %zu.",
            waypoint_number);
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

    bool is_at_target(const Target& target) const
    {
        const auto& p = current_pose_.pose.position;
        const double dx = p.x - target.x;
        const double dy = p.y - target.y;
        const double dz = p.z - target.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) <=
               get_parameter("arrival_tolerance").as_double();
    }

    static double yaw_from_pose(const geometry_msgs::msg::PoseStamped& pose)
    {
        const auto& q = pose.pose.orientation;
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void publish_position(const Target& target, bool smooth_xy = true)
    {
        if (smooth_xy) {
            update_lowpass_setpoint(target);
        } else {
            reset_shaped_setpoint(target);
        }

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

    void reset_shaped_setpoint(const Target& target)
    {
        shaped_x_ = target.x;
        shaped_y_ = target.y;
        shaped_z_ = target.z;
        shaped_setpoint_initialized_ = true;
        lowpass_target_ = target;
        lowpass_target_initialized_ = true;
        last_shaping_time_ = now();
    }

    void update_lowpass_setpoint(const Target& target)
    {
        const auto current_time = now();
        if (!shaped_setpoint_initialized_) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
            shaped_setpoint_initialized_ = true;
            lowpass_target_ = target;
            lowpass_target_initialized_ = true;
            last_shaping_time_ = current_time;
            return;
        }

        const double dt = std::clamp((current_time - last_shaping_time_).seconds(), 0.001, 0.1);
        last_shaping_time_ = current_time;

        if (!lowpass_target_initialized_ || !targets_match(target, lowpass_target_)) {
            const double target_step_distance = distance_between_lowpass_targets(target);
            active_lowpass_tau_ = calculate_lowpass_tau(target_step_distance);
            lowpass_target_ = target;
            lowpass_target_initialized_ = true;

            RCLCPP_INFO(get_logger(),
                        "Setpoint low-pass tau %.2f selected from target step distance %.2f m.",
                        active_lowpass_tau_, target_step_distance);
        }

        const double dx = target.x - shaped_x_;
        const double dy = target.y - shaped_y_;
        shaped_z_ = target.z;
        const double distance = std::sqrt(dx * dx + dy * dy);
        const double snap_tolerance = std::max(0.0, get_parameter("setpoint_snap_tolerance").as_double());

        if (distance <= snap_tolerance) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            return;
        }

        const double tau = active_lowpass_tau_;
        if (tau <= 0.0) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            return;
        }

        const double alpha = dt / (tau + dt);
        shaped_x_ += alpha * dx;
        shaped_y_ += alpha * dy;
    }

    double distance_between_lowpass_targets(const Target& target) const
    {
        if (!lowpass_target_initialized_) {
            return 0.0;
        }

        const double dx = target.x - lowpass_target_.x;
        const double dy = target.y - lowpass_target_.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    static bool targets_match(const Target& lhs, const Target& rhs)
    {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }

    double calculate_lowpass_tau(double distance) const
    {
        const double near_error = std::max(0.0, get_parameter("setpoint_lowpass_near_error").as_double());
        const double far_error = std::max(near_error, get_parameter("setpoint_lowpass_far_error").as_double());
        const double min_tau = get_parameter("setpoint_lowpass_min_tau").as_double();
        const double near_tau = get_parameter("setpoint_lowpass_near_tau").as_double();
        const double far_tau = get_parameter("setpoint_lowpass_far_tau").as_double();

        if (distance <= near_error) {
            if (near_error <= 0.0) {
                return near_tau;
            }
            const double ratio = std::clamp(distance / near_error, 0.0, 1.0);
            return min_tau + ratio * (near_tau - min_tau);
        }

        if (far_error <= near_error) {
            return distance >= far_error ? far_tau : near_tau;
        }

        const double ratio = std::clamp((distance - near_error) / (far_error - near_error), 0.0, 1.0);
        return near_tau + ratio * (far_tau - near_tau);
    }

    void start_terminal_input()
    {
        if (!get_parameter("enable_terminal_input").as_bool()) {
            return;
        }

        if (!isatty(STDIN_FILENO)) {
            RCLCPP_WARN(get_logger(), "Terminal input disabled because stdin is not a TTY.");
            return;
        }

        terminal_thread_running_ = true;
        terminal_thread_ = std::thread([this]() {
            RCLCPP_INFO(get_logger(), "Terminal input enabled. Press Enter to advance after reaching a waypoint.");
            std::string line;
            while (terminal_thread_running_ && rclcpp::ok() && std::getline(std::cin, line)) {
                handle_terminal_enter();
            }
        });
        terminal_thread_.detach();
    }

    void handle_terminal_enter()
    {
        const Phase phase = phase_.load();
        if (phase == Phase::WAITING_PATH) {
            RCLCPP_WARN(get_logger(), "Enter ignored: no waypoint path has been received yet.");
            return;
        }

        if (phase == Phase::STARTING_OFFBOARD || phase == Phase::FLYING_WAYPOINT ||
            phase == Phase::RETURNING_HOME_APPROACH || phase == Phase::LANDING_45_DEG ||
            phase == Phase::LANDING_SETTLE || phase == Phase::DISARMING) {
            RCLCPP_WARN(get_logger(), "Enter ignored: active waypoint has not been reached yet.");
            return;
        }

        if (phase == Phase::MISSION_COMPLETE) {
            RCLCPP_INFO(get_logger(), "Enter ignored: mission is already complete.");
            return;
        }

        advance_requested_ = true;
        RCLCPP_INFO(get_logger(), "Advance requested. Next waypoint will become active.");
    }

    Target active_target() const
    {
        std::lock_guard<std::mutex> lock(waypoint_mutex_);
        return waypoints_.at(active_index_);
    }

    Target first_target() const
    {
        std::lock_guard<std::mutex> lock(waypoint_mutex_);
        return waypoints_.front();
    }

    std::size_t waypoint_count() const
    {
        std::lock_guard<std::mutex> lock(waypoint_mutex_);
        return waypoints_.size();
    }

    std::atomic<Phase> phase_{Phase::WAITING_PATH};
    bool pose_received_{false};
    bool initial_yaw_captured_{false};
    bool waypoint_reached_{false};
    bool shaped_setpoint_initialized_{false};
    bool lowpass_target_initialized_{false};
    bool landing_plan_initialized_{false};
    double initial_yaw_{0.0};
    double shaped_x_{0.0};
    double shaped_y_{0.0};
    double shaped_z_{0.0};
    Target lowpass_target_{0.0, 0.0, 0.0};
    Target landing_approach_target_{0.0, 0.0, 0.0};
    Target landing_ground_target_{0.0, 0.0, 0.0};
    Target landing_touchdown_target_{0.0, 0.0, 0.0};
    Target landing_start_target_{0.0, 0.0, 0.0};
    double active_lowpass_tau_{0.0};
    double landing_total_distance_{0.0};
    double landing_progress_{0.0};
    std::size_t active_index_{0};
    std::atomic_bool advance_requested_{false};
    std::atomic_bool terminal_thread_running_{false};

    mutable std::mutex waypoint_mutex_;
    std::vector<Target> waypoints_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    rclcpp::Time last_request_time_;
    rclcpp::Time last_shaping_time_;
    rclcpp::Time landing_last_update_time_;
    rclcpp::Time landing_settle_start_time_;
    rclcpp::Time last_disarm_request_time_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr yolo_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread terminal_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointOffboardNode>();

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
