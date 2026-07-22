#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

class SimpleOffboardNode : public rclcpp::Node {
public:
    SimpleOffboardNode()
        : Node("simple_offboard_node")
    {
        declare_parameter<bool>("auto_set_mode", true);
        declare_parameter<bool>("auto_arm", true);
        declare_parameter<double>("takeoff_x", 0.0);
        declare_parameter<double>("takeoff_y", 0.0);
        declare_parameter<double>("takeoff_z", 0.5);
        declare_parameter<double>("arrival_tolerance", 0.12);
        declare_parameter<double>("setpoint_lowpass_tau", 1.6);
        declare_parameter<double>("setpoint_snap_tolerance", 0.03);
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

        setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
            "mavros/setpoint_raw/local", 10);

        set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        last_request_time_ = now();
        start_terminal_input();
        timer_ = create_wall_timer(20ms, std::bind(&SimpleOffboardNode::timer_callback, this));
    }

    ~SimpleOffboardNode() override
    {
        terminal_thread_running_ = false;
    }

    bool is_connected() const
    {
        return current_state_.connected;
    }

private:
    enum class Phase {
        INIT,
        TAKEOFF_HOVER,
        MANUAL_TARGET,
    };

    struct Target {
        double x;
        double y;
        double z;
    };

    void timer_callback()
    {
        if (!current_state_.connected) {
            return;
        }

        const Target takeoff_hover{
            get_parameter("takeoff_x").as_double(),
            get_parameter("takeoff_y").as_double(),
            get_parameter("takeoff_z").as_double(),
        };

        Target manual_target{};
        const bool has_manual_target = get_manual_target(manual_target);

        if (phase_ == Phase::INIT) {
            publish_position(takeoff_hover);
            ensure_offboard_and_armed();

            if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
                RCLCPP_INFO(get_logger(),
                            "OFFBOARD and armed. Taking off to hover point %.2f %.2f %.2f.",
                            takeoff_hover.x, takeoff_hover.y, takeoff_hover.z);
                phase_ = Phase::TAKEOFF_HOVER;
            }
            return;
        }

        if (phase_ == Phase::MANUAL_TARGET) {
            if (has_manual_target) {
                publish_position(manual_target);

                if (pose_received_ && is_at_target(manual_target)) {
                    if (!manual_target_reached_) {
                        manual_target_reached_ = true;
                        RCLCPP_INFO(get_logger(),
                                    "Reached terminal target %.2f %.2f %.2f. Holding position.",
                                    manual_target.x, manual_target.y, manual_target.z);
                    }
                } else if (manual_target_reached_) {
                    manual_target_reached_ = false;
                }
            }
            return;
        }

        if (phase_ == Phase::TAKEOFF_HOVER) {
            publish_position(takeoff_hover);

            if (!pose_received_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Waiting for local position while holding takeoff point.");
                return;
            }

            if (is_at_target(takeoff_hover)) {
                if (!takeoff_hover_reached_) {
                    takeoff_hover_reached_ = true;
                    RCLCPP_INFO(get_logger(),
                                "Reached takeoff hover point. Holding here until terminal target input.");
                }
            } else if (takeoff_hover_reached_) {
                takeoff_hover_reached_ = false;
                RCLCPP_INFO(get_logger(), "Left takeoff hover tolerance. Continuing to hold.");
            }

            if (takeoff_hover_reached_ && has_manual_target) {
                RCLCPP_INFO(get_logger(),
                            "Terminal target received. Flying to %.2f %.2f %.2f.",
                            manual_target.x, manual_target.y, manual_target.z);
                phase_ = Phase::MANUAL_TARGET;
            }
            return;
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
        return std::sqrt(dx * dx + dy * dy + dz * dz) <= get_parameter("arrival_tolerance").as_double();
    }

    static double yaw_from_pose(const geometry_msgs::msg::PoseStamped& pose)
    {
        const auto& q = pose.pose.orientation;
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void publish_position(const Target& target)
    {
        update_lowpass_setpoint(target);

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

    void update_lowpass_setpoint(const Target& target)
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
        const double snap_tolerance = std::max(0.0, get_parameter("setpoint_snap_tolerance").as_double());

        if (distance <= snap_tolerance) {
            shaped_x_ = target.x;
            shaped_y_ = target.y;
            shaped_z_ = target.z;
            return;
        }

        const double tau = get_parameter("setpoint_lowpass_tau").as_double();
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
            RCLCPP_INFO(get_logger(),
                        "Terminal input enabled. Enter: x y z | set x y z | hold x y z");
            std::string line;
            while (terminal_thread_running_ && rclcpp::ok() && std::getline(std::cin, line)) {
                handle_terminal_line(line);
            }
        });
        terminal_thread_.detach();
    }

    void handle_terminal_line(const std::string& line)
    {
        std::istringstream stream(line);
        std::string first;
        if (!(stream >> first)) {
            return;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        if (first == "set" || first == "hold" || first == "SET" || first == "HOLD") {
            if (stream >> x >> y >> z) {
                set_manual_target(x, y, z);
                return;
            }
            RCLCPP_WARN(get_logger(), "Invalid terminal command. Use: set x y z");
            return;
        }

        std::istringstream xyz_stream(line);
        if (xyz_stream >> x >> y >> z) {
            set_manual_target(x, y, z);
            return;
        }

        RCLCPP_WARN(get_logger(),
                    "Unknown terminal command '%s'. Use: x y z, set x y z, or hold x y z.",
                    line.c_str());
    }

    void set_manual_target(double x, double y, double z)
    {
        {
            std::lock_guard<std::mutex> lock(manual_target_mutex_);
            manual_target_ = Target{x, y, z};
            manual_target_set_ = true;
            manual_target_reached_ = false;
        }

        RCLCPP_INFO(get_logger(),
                    "Terminal target updated to %.3f %.3f %.3f. Low-pass shaping remains active.",
                    x, y, z);
    }

    bool get_manual_target(Target& target) const
    {
        std::lock_guard<std::mutex> lock(manual_target_mutex_);
        if (!manual_target_set_) {
            return false;
        }

        target = manual_target_;
        return true;
    }

    Phase phase_{Phase::INIT};
    bool pose_received_{false};
    double initial_yaw_{0.0};
    bool initial_yaw_captured_{false};
    bool takeoff_hover_reached_{false};
    bool shaped_setpoint_initialized_{false};
    double shaped_x_{0.0};
    double shaped_y_{0.0};
    double shaped_z_{0.0};

    mutable std::mutex manual_target_mutex_;
    Target manual_target_{0.0, 0.0, 0.6};
    bool manual_target_set_{false};
    std::atomic_bool manual_target_reached_{false};
    std::atomic_bool terminal_thread_running_{false};

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    rclcpp::Time last_request_time_;
    rclcpp::Time last_shaping_time_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread terminal_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleOffboardNode>();

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
坐标：(x, y, z)，向前+y,向右是+x,向上是+z
启动节点后，默认会起飞到(0, 0, 0.5)的位置悬停，等待终端输入目标点。
终端输入格式：
直接输入坐标：x y z
*/
