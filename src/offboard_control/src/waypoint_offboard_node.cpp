#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
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
        declare_parameter<double>("arrival_tolerance", 0.12);
        declare_parameter<bool>("enable_terminal_input", true);

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

        last_request_time_ = now();
        start_terminal_input();
        timer_ = create_wall_timer(20ms, std::bind(&WaypointOffboardNode::timer_callback, this));

        RCLCPP_INFO(get_logger(),
                    "Waiting for waypoint path on [%s]. Vehicle will not take off before a path is received.",
                    get_parameter("path_topic").as_string().c_str());
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
            publish_position(active_target());
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

            if (active_index_ + 1U >= waypoint_count()) {
                phase_.store(Phase::MISSION_COMPLETE);
                RCLCPP_INFO(get_logger(),
                            "Reached final waypoint %zu/%zu. Mission complete, holding position.",
                            active_index_ + 1U, waypoint_count());
                return;
            }

            RCLCPP_INFO(get_logger(),
                        "Reached waypoint %zu/%zu. Press Enter once to fly to the next waypoint.",
                        active_index_ + 1U, waypoint_count());
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

    void publish_position(const Target& target)
    {
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

        msg.position.x = target.x;
        msg.position.y = target.y;
        msg.position.z = target.z;
        if (initial_yaw_captured_) {
            msg.yaw = initial_yaw_;
        } else {
            msg.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
        }

        setpoint_pub_->publish(msg);
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

        if (phase == Phase::STARTING_OFFBOARD || phase == Phase::FLYING_WAYPOINT) {
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

    std::size_t waypoint_count() const
    {
        std::lock_guard<std::mutex> lock(waypoint_mutex_);
        return waypoints_.size();
    }

    std::atomic<Phase> phase_{Phase::WAITING_PATH};
    bool pose_received_{false};
    bool initial_yaw_captured_{false};
    bool waypoint_reached_{false};
    double initial_yaw_{0.0};
    std::size_t active_index_{0};
    std::atomic_bool advance_requested_{false};
    std::atomic_bool terminal_thread_running_{false};

    mutable std::mutex waypoint_mutex_;
    std::vector<Target> waypoints_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    rclcpp::Time last_request_time_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
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
