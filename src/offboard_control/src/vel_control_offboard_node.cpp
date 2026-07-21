#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

class VelControlOffboardNode : public rclcpp::Node {
public:
    VelControlOffboardNode()
        : Node("vel_control_offboard_node")
    {
        // Debug parameters: keep these together for quick field tuning.
        declare_parameter<double>("flight_z", 0.5);
        declare_parameter<double>("cruise_speed", 0.4);
        declare_parameter<double>("accel_limit", 0.2);
        declare_parameter<double>("brake_accel", 0.2);
        declare_parameter<double>("arrival_tolerance", 0.08);
        declare_parameter<double>("position_p_gain", 1.4);
        declare_parameter<double>("normal_deadband", 0.015);
        declare_parameter<double>("max_correction_speed", 0.20);
        declare_parameter<bool>("auto_set_mode", true);
        declare_parameter<bool>("auto_arm", true);
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

        last_commanded_position_ = Target{0.0, 0.0, flight_z()};
        last_request_time_ = now();
        last_velocity_update_time_ = now();
        start_terminal_input();
        timer_ = create_wall_timer(20ms, std::bind(&VelControlOffboardNode::timer_callback, this));

        RCLCPP_INFO(get_logger(),
                    "vel_control_offboard_node started. Publishing setpoints at 50 Hz.");
    }

    ~VelControlOffboardNode() override
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
        LINE_FLIGHT,
        HOLD,
    };

    enum class Axis {
        X,
        Y,
    };

    struct Target {
        double x;
        double y;
        double z;
    };

    struct LineCommand {
        Axis axis;
        Target target;
    };

    void timer_callback()
    {
        if (!current_state_.connected) {
            return;
        }

        const Target takeoff_hover{0.0, 0.0, flight_z()};
        initialize_last_commanded_position(takeoff_hover);

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

        if (phase_ == Phase::TAKEOFF_HOVER) {
            publish_position(takeoff_hover);

            if (!pose_received_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Waiting for local position while holding takeoff point.");
                return;
            }

            if (is_at_position(takeoff_hover)) {
                if (!takeoff_hover_reached_) {
                    takeoff_hover_reached_ = true;
                    {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        last_commanded_position_ = takeoff_hover;
                    }
                    RCLCPP_INFO(get_logger(),
                                "Reached takeoff hover point. Enter x end or y end.");
                }
                start_pending_command_if_available();
            } else if (takeoff_hover_reached_) {
                takeoff_hover_reached_ = false;
                RCLCPP_INFO(get_logger(), "Left takeoff hover tolerance. Continuing to hold.");
            }
            return;
        }

        if (phase_ == Phase::LINE_FLIGHT) {
            run_line_flight();
            return;
        }

        if (phase_ == Phase::HOLD) {
            publish_velocity(0.0, 0.0);
            start_pending_command_if_available();
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

    void run_line_flight()
    {
        if (!pose_received_) {
            publish_velocity(0.0, 0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for local position during line flight.");
            return;
        }

        LineCommand command{};
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command = active_command_;
        }

        const double remaining = tangent_target(command) - tangent_position(command.axis);
        const double signed_remaining = tangent_direction_ * remaining;
        if (signed_remaining <= arrival_tolerance()) {
            commanded_vx_ = 0.0;
            commanded_vy_ = 0.0;
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                last_commanded_position_ = command.target;
            }
            publish_velocity(0.0, 0.0);
            phase_ = Phase::HOLD;
            RCLCPP_INFO(get_logger(),
                        "Reached target %.3f %.3f. Holding and waiting for next command.",
                        command.target.x, command.target.y);
            return;
        }

        const double speed_limit = planned_tangent_speed(signed_remaining);
        const double tangent_velocity = tangent_direction_ * speed_limit;
        const double normal_velocity = normal_correction_velocity(command);

        double desired_vx = 0.0;
        double desired_vy = 0.0;
        if (command.axis == Axis::X) {
            desired_vx = tangent_velocity;
            desired_vy = normal_velocity;
        } else {
            desired_vx = normal_velocity;
            desired_vy = tangent_velocity;
        }

        const double dt = velocity_dt();
        const double step = std::max(0.0, get_parameter("accel_limit").as_double()) * dt;
        commanded_vx_ = approach(commanded_vx_, desired_vx, step);
        commanded_vy_ = approach(commanded_vy_, desired_vy, step);

        publish_velocity(commanded_vx_, commanded_vy_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Line flight vx=%.3f vy=%.3f remaining=%.3f.",
                             commanded_vx_, commanded_vy_, signed_remaining);
    }

    double planned_tangent_speed(double signed_remaining) const
    {
        const double cruise = std::max(0.0, get_parameter("cruise_speed").as_double());
        const double brake = std::max(0.01, get_parameter("brake_accel").as_double());
        const double brake_limited = std::sqrt(std::max(0.0, 2.0 * brake * signed_remaining));
        return std::min(cruise, brake_limited);
    }

    double normal_correction_velocity(const LineCommand& command) const
    {
        const double error = normal_target(command) - normal_position(command.axis);
        if (std::abs(error) < std::max(0.0, get_parameter("normal_deadband").as_double())) {
            return 0.0;
        }

        const double raw_velocity = get_parameter("position_p_gain").as_double() * error;
        const double max_velocity =
            std::max(0.0, get_parameter("max_correction_speed").as_double());
        return std::clamp(raw_velocity, -max_velocity, max_velocity);
    }

    void publish_position(const Target& target)
    {
        commanded_vx_ = 0.0;
        commanded_vy_ = 0.0;
        last_velocity_update_time_ = now();

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
        set_yaw(msg);
        setpoint_pub_->publish(msg);
    }

    void publish_velocity(double vx, double vy)
    {
        mavros_msgs::msg::PositionTarget msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_PX |
            mavros_msgs::msg::PositionTarget::IGNORE_PY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        msg.position.z = flight_z();
        msg.velocity.x = vx;
        msg.velocity.y = vy;
        set_yaw(msg);
        setpoint_pub_->publish(msg);
    }

    void set_yaw(mavros_msgs::msg::PositionTarget& msg)
    {
        if (initial_yaw_captured_) {
            msg.yaw = initial_yaw_;
        } else {
            msg.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
        }
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
            RCLCPP_INFO(get_logger(), "Terminal input enabled. Enter: x end | y end");
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
        std::string axis_text;
        double end = 0.0;
        if (!(stream >> axis_text >> end)) {
            RCLCPP_WARN(get_logger(), "Invalid terminal command. Use: x end or y end");
            return;
        }

        const char axis_char = static_cast<char>(std::tolower(axis_text.front()));
        if (axis_text.size() != 1 || (axis_char != 'x' && axis_char != 'y')) {
            RCLCPP_WARN(get_logger(), "Unknown axis '%s'. Use: x end or y end.",
                        axis_text.c_str());
            return;
        }

        queue_line_command(axis_char == 'x' ? Axis::X : Axis::Y, end);
    }

    void queue_line_command(Axis axis, double end)
    {
        LineCommand command{};
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command.axis = axis;
            command.target = last_commanded_position_;
            command.target.z = flight_z();
            if (axis == Axis::X) {
                command.target.x = end;
            } else {
                command.target.y = end;
            }
            pending_command_ = command;
            pending_command_set_ = true;
        }

        RCLCPP_INFO(get_logger(),
                    "Queued %s-axis target %.3f. Full target %.3f %.3f %.3f.",
                    axis == Axis::X ? "X" : "Y", end,
                    command.target.x, command.target.y, command.target.z);
    }

    void start_pending_command_if_available()
    {
        LineCommand command{};
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!pending_command_set_) {
                return;
            }
            command = pending_command_;
            active_command_ = command;
            pending_command_set_ = false;
        }

        tangent_direction_ = tangent_target(command) >= tangent_position(command.axis) ? 1.0 : -1.0;
        commanded_vx_ = 0.0;
        commanded_vy_ = 0.0;
        last_velocity_update_time_ = now();
        phase_ = Phase::LINE_FLIGHT;

        RCLCPP_INFO(get_logger(),
                    "Starting %s-axis line flight to %.3f with normal reference %.3f.",
                    command.axis == Axis::X ? "X" : "Y",
                    tangent_target(command), normal_target(command));
    }

    void initialize_last_commanded_position(const Target& takeoff_hover)
    {
        if (last_commanded_position_initialized_) {
            return;
        }
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_commanded_position_ = takeoff_hover;
        last_commanded_position_initialized_ = true;
    }

    bool is_at_position(const Target& target) const
    {
        if (!pose_received_) {
            return false;
        }

        const auto& p = current_pose_.pose.position;
        const double dx = p.x - target.x;
        const double dy = p.y - target.y;
        const double dz = p.z - target.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) <= arrival_tolerance();
    }

    double tangent_position(Axis axis) const
    {
        const auto& p = current_pose_.pose.position;
        return axis == Axis::X ? p.x : p.y;
    }

    double normal_position(Axis axis) const
    {
        const auto& p = current_pose_.pose.position;
        return axis == Axis::X ? p.y : p.x;
    }

    static double tangent_target(const LineCommand& command)
    {
        return command.axis == Axis::X ? command.target.x : command.target.y;
    }

    static double normal_target(const LineCommand& command)
    {
        return command.axis == Axis::X ? command.target.y : command.target.x;
    }

    double velocity_dt()
    {
        const auto t = now();
        const double dt = std::clamp((t - last_velocity_update_time_).seconds(), 0.001, 0.1);
        last_velocity_update_time_ = t;
        return dt;
    }

    static double approach(double current, double target, double max_step)
    {
        if (current < target) {
            return std::min(current + max_step, target);
        }
        return std::max(current - max_step, target);
    }

    static double yaw_from_pose(const geometry_msgs::msg::PoseStamped& pose)
    {
        const auto& q = pose.pose.orientation;
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    double flight_z() const
    {
        return get_parameter("flight_z").as_double();
    }

    double arrival_tolerance() const
    {
        return std::max(0.0, get_parameter("arrival_tolerance").as_double());
    }

    Phase phase_{Phase::INIT};
    bool pose_received_{false};
    bool initial_yaw_captured_{false};
    bool takeoff_hover_reached_{false};
    bool last_commanded_position_initialized_{false};
    double initial_yaw_{0.0};
    double commanded_vx_{0.0};
    double commanded_vy_{0.0};
    double tangent_direction_{1.0};

    mutable std::mutex command_mutex_;
    Target last_commanded_position_{0.0, 0.0, 0.5};
    LineCommand pending_command_{Axis::X, Target{0.0, 0.0, 0.5}};
    LineCommand active_command_{Axis::X, Target{0.0, 0.0, 0.5}};
    bool pending_command_set_{false};
    std::atomic_bool terminal_thread_running_{false};

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    rclcpp::Time last_request_time_;
    rclcpp::Time last_velocity_update_time_;

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
    auto node = std::make_shared<VelControlOffboardNode>();

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
flight_z=0.5
飞行高度，单位米。起飞悬停和直线飞行时都保持这个高度。
cruise_speed=0.25
沿目标轴的巡航速度，单位 m/s。比如 x 1.0 时，主要控制 vx 接近 0.25 或 -0.25。
accel_limit=0.25
速度斜坡限制，单位 m/s²。防止速度指令从 0 突然跳到巡航速度，数值越小，起步和变速越柔。
brake_accel=0.25
接近终点时的减速限制，单位 m/s²。节点会按剩余距离自动压低速度，数值越小，越早减速。
arrival_tolerance=0.08
到达判定距离，单位米。沿目标轴剩余距离小于 8cm 时认为到达，然后水平速度归零。
position_p_gain=1.4
另一轴位置环 P 增益。例如沿 X 飞时，用它根据 Y 误差生成 vy 纠偏速度。数值越大，回线越积极；太大会摆。
normal_deadband=0.015
另一轴纠偏死区，单位米。误差小于 1.5cm 时不纠偏，避免定位小抖动导致速度来回变号。
max_correction_speed=0.10
另一轴最大纠偏速度，单位 m/s。限制 P 环输出，避免横向纠偏太猛。
auto_set_mode=true
是否自动请求切到 OFFBOARD 模式。设为 false 时需要你手动切模式。
auto_arm=true
是否自动解锁。设为 false 时需要你手动 arm。
enable_terminal_input=true
是否开启终端输入。设为 false 后节点只会起飞悬停，不接收 x 终点 / y 终点 命令。

x 1.0
y 1.0
x -1.0
y 0.0

*/
