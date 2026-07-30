#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

class CartographerLaserTransfer : public rclcpp::Node
{
public:
    CartographerLaserTransfer() : Node("cartographer_laser_transfer")
    {
        rangefinder_z_offset_m_ = declare_parameter<double>("rangefinder_z_offset_m", 0.115);
        height_compensation_enabled_ = declare_parameter<bool>("height_compensation_enabled", true);
        target_surface_height_m_ = declare_parameter<double>("target_surface_height_m", 0.19);
        target_surface_height_min_m_ = declare_parameter<double>("target_surface_height_min_m", 0.11);
        target_surface_height_max_m_ = declare_parameter<double>("target_surface_height_max_m", 0.30);
        jump_enter_min_m_ = declare_parameter<double>("jump_enter_min_m", 0.10);
        jump_enter_max_m_ = declare_parameter<double>("jump_enter_max_m", 0.32);
        jump_release_min_m_ = declare_parameter<double>("jump_release_min_m", 0.10);
        jump_release_max_m_ = declare_parameter<double>("jump_release_max_m", 0.32);
        jump_confirm_frames_ = declare_parameter<int>("jump_confirm_frames", 3);
        jump_confirm_time_s_ = declare_parameter<double>("jump_confirm_time_s", 0.20);
        raw_z_filter_alpha_ = declare_parameter<double>("raw_z_filter_alpha", 0.35);
        corrected_z_filter_alpha_ = declare_parameter<double>("corrected_z_filter_alpha", 0.45);
        max_corrected_z_step_m_ = declare_parameter<double>("max_corrected_z_step_m", 0.04);
        fake_vz_zero_time_s_ = declare_parameter<double>("fake_vz_zero_time_s", 0.25);
        landing_release_lock_z_m_ = declare_parameter<double>("landing_release_lock_z_m", 0.65);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        vision_pose_debug_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/track2vision/vision_pose", 10);
        vision_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/mavros/odometry/out", 10);
        ev_stable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/track2vision/ev_stable", 10);
        ev_stability_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/track2vision/ev_stability_status", 10);
        range_px4_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
            "/stp23/range_px4", rclcpp::SensorDataQoS());
        raw_range_debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/track2vision/height/raw_range", 10);
        raw_z_debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/track2vision/height/raw_z", 10);
        corrected_z_debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/track2vision/height/corrected_z", 10);
        surface_bias_debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/track2vision/height/surface_bias", 10);
        height_state_debug_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/track2vision/height/state", 10);
        target_detected_debug_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/track2vision/height/target_detected", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&CartographerLaserTransfer::imu_callback, this, std::placeholders::_1));
        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::SensorDataQoS(),
            std::bind(&CartographerLaserTransfer::local_pose_callback, this, std::placeholders::_1));
        range_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/stp23/range",
            rclcpp::SensorDataQoS(),
            std::bind(&CartographerLaserTransfer::range_callback, this, std::placeholders::_1));
        landing_on_target_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/track2vision/height/landing_on_target",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                landing_on_target_ = msg->data;
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&CartographerLaserTransfer::publish_vision_odometry, this));

        RCLCPP_INFO(this->get_logger(), "cartographer_laser_transfer node started.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_imu_time_ = msg->header.stamp;
        imu_received_ = true;
    }

    void local_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_local_z_ = msg->pose.position.z;
        local_pose_received_ = true;
    }

    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        if (!std::isfinite(msg->range) || msg->range < msg->min_range || msg->range > msg->max_range)
        {
            return;
        }

        const rclcpp::Time stamp(msg->header.stamp);
        const double raw_range_m = static_cast<double>(msg->range);
        const double raw_z_enu = raw_range_m + rangefinder_z_offset_m_;

        latest_raw_range_m_ = raw_range_m;
        latest_raw_z_enu_ = raw_z_enu;

        if (!range_received_)
        {
            raw_z_filtered_ = raw_z_enu;
            corrected_z_filtered_ = raw_z_enu;
            latest_range_z_enu_ = raw_z_enu;
        }
        else
        {
            const double raw_alpha = std::clamp(raw_z_filter_alpha_, 0.0, 1.0);
            raw_z_filtered_ += raw_alpha * (raw_z_enu - raw_z_filtered_);
        }

        update_height_compensation(stamp, raw_z_enu);
        const double corrected_z = filter_corrected_z(raw_z_enu + surface_bias_m_);

        if (range_received_)
        {
            const double dt = (stamp - latest_range_time_).seconds();
            if (dt > 0.005 && dt < 0.5)
            {
                constexpr double max_vertical_speed_mps = 3.0;
                constexpr double velocity_deadband_mps = 0.02;
                constexpr double alpha = 0.35;

                double measured_vz = (corrected_z - latest_range_z_enu_) / dt;
                measured_vz = std::clamp(
                    measured_vz, -max_vertical_speed_mps, max_vertical_speed_mps);
                if (fake_vz_zero_active_ && stamp < fake_vz_zero_until_)
                {
                    measured_vz = 0.0;
                    filtered_vz_enu_ *= 0.50;
                }
                else
                {
                    fake_vz_zero_active_ = false;
                }
                if (std::abs(measured_vz) < velocity_deadband_mps)
                {
                    measured_vz = 0.0;
                }
                filtered_vz_enu_ += alpha * (measured_vz - filtered_vz_enu_);
            }
        }

        latest_range_time_ = stamp;
        latest_range_z_enu_ = corrected_z;
        previous_raw_z_enu_ = raw_z_enu;
        previous_raw_z_valid_ = true;
        range_received_ = true;

        publish_compensated_range(*msg, corrected_z);
        publish_height_debug();
    }

    void publish_vision_odometry()
    {
        if (!imu_received_)
        {
            publish_stability_wait("waiting_imu");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "IMU data not received yet, waiting...");
            return;
        }
        if (!range_received_ && !local_pose_received_)
        {
            publish_stability_wait("waiting_height");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Height source not received yet, waiting...");
            return;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            publish_stability_wait("tf_missing");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Could not get transform: %s", ex.what());
            return;
        }

        const double x = transformStamped.transform.translation.x;
        const double y = transformStamped.transform.translation.y;

        tf2::Quaternion q_orig;
        tf2::fromMsg(transformStamped.transform.rotation, q_orig);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

        tf2::Quaternion q_yaw;
        constexpr double yaw_offset_rad = 1.5707963267948966;
        const double vision_yaw = yaw + yaw_offset_rad;
        q_yaw.setRPY(0, 0, vision_yaw);

        const auto stamp = transformStamped.header.stamp;
        update_filtered_velocity(stamp, x, y, vision_yaw);
        update_stability_status(stamp, x, y, vision_yaw);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = range_received_ ? latest_range_z_enu_ : latest_local_z_;
        pose_msg.pose.orientation = tf2::toMsg(q_yaw);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.twist.twist.linear.x = filtered_vx_body_;
        odom_msg.twist.twist.linear.y = filtered_vy_body_;
        odom_msg.twist.twist.linear.z = range_received_ ? filtered_vz_enu_ : 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
        fill_covariances(odom_msg);

        vision_pose_debug_pub_->publish(pose_msg);
        vision_odom_pub_->publish(odom_msg);
    }

    void update_filtered_velocity(
        const builtin_interfaces::msg::Time &stamp,
        const double x,
        const double y,
        const double body_yaw)
    {
        const rclcpp::Time current_time(stamp);
        constexpr double duplicate_decay = 0.60;
        constexpr double velocity_window_s = 0.35;
        constexpr double min_window_s = 0.12;
        constexpr double stationary_displacement_m = 0.04;
        constexpr double max_speed_mps = 3.0;
        constexpr double max_accel_mps2 = 3.0;
        constexpr double filter_tau_s = 0.18;
        constexpr double velocity_deadband_mps = 0.03;

        if (!pose_samples_.empty() && current_time <= pose_samples_.back().time)
        {
            filtered_vx_body_ *= duplicate_decay;
            filtered_vy_body_ *= duplicate_decay;
            zero_small_velocity(velocity_deadband_mps);
            return;
        }

        pose_samples_.push_back({current_time, x, y});
        while (pose_samples_.size() > 2 &&
               (current_time - pose_samples_.front().time).seconds() > velocity_window_s)
        {
            pose_samples_.pop_front();
        }

        if (pose_samples_.size() < 2)
        {
            return;
        }

        const auto &oldest = pose_samples_.front();
        const double dt = (current_time - oldest.time).seconds();
        if (dt < min_window_s)
        {
            return;
        }

        const double dx = x - oldest.x;
        const double dy = y - oldest.y;
        double vx_map = 0.0;
        double vy_map = 0.0;

        if (std::hypot(dx, dy) >= stationary_displacement_m)
        {
            vx_map = dx / dt;
            vy_map = dy / dt;
        }

        if (std::hypot(vx_map, vy_map) > max_speed_mps)
        {
            vx_map = 0.0;
            vy_map = 0.0;
        }

        const double cos_yaw = std::cos(body_yaw);
        const double sin_yaw = std::sin(body_yaw);
        double vx_body = cos_yaw * vx_map + sin_yaw * vy_map;
        double vy_body = -sin_yaw * vx_map + cos_yaw * vy_map;

        if (std::hypot(vx_body, vy_body) < velocity_deadband_mps)
        {
            vx_body = 0.0;
            vy_body = 0.0;
        }

        const double alpha = std::clamp(dt / (filter_tau_s + dt), 0.0, 1.0);
        const double max_delta_v = max_accel_mps2 * dt;
        const double target_vx = filtered_vx_body_ + std::clamp(vx_body - filtered_vx_body_, -max_delta_v, max_delta_v);
        const double target_vy = filtered_vy_body_ + std::clamp(vy_body - filtered_vy_body_, -max_delta_v, max_delta_v);
        filtered_vx_body_ += alpha * (target_vx - filtered_vx_body_);
        filtered_vy_body_ += alpha * (target_vy - filtered_vy_body_);
        zero_small_velocity(velocity_deadband_mps);
    }

    void zero_small_velocity(const double deadband_mps)
    {
        if (std::hypot(filtered_vx_body_, filtered_vy_body_) < deadband_mps)
        {
            filtered_vx_body_ = 0.0;
            filtered_vy_body_ = 0.0;
        }
    }

    void fill_covariances(nav_msgs::msg::Odometry &odom_msg)
    {
        odom_msg.pose.covariance.fill(0.0);
        odom_msg.twist.covariance.fill(0.0);

        odom_msg.pose.covariance[0] = 0.04;
        odom_msg.pose.covariance[7] = 0.04;
        odom_msg.pose.covariance[14] = range_received_ ? 0.04 : 10000.0;
        odom_msg.pose.covariance[21] = 0.25;
        odom_msg.pose.covariance[28] = 0.25;
        odom_msg.pose.covariance[35] = 0.10;

        odom_msg.twist.covariance[0] = 0.04;
        odom_msg.twist.covariance[7] = 0.04;
        odom_msg.twist.covariance[14] = range_received_ ? 0.09 : 10000.0;
        odom_msg.twist.covariance[21] = 1.0;
        odom_msg.twist.covariance[28] = 1.0;
        odom_msg.twist.covariance[35] = 1.0;
    }

    void update_stability_status(
        const builtin_interfaces::msg::Time &stamp,
        const double x,
        const double y,
        const double yaw)
    {
        constexpr double stability_window_s = 8.0;
        constexpr double required_stable_s = 3.0;
        constexpr double max_tf_age_s = 0.50;
        constexpr double max_future_tf_s = 0.10;
        constexpr double max_xy_drift_m = 0.06;
        constexpr double max_xy_span_m = 0.12;
        constexpr double max_xy_rate_mps = 0.015;
        constexpr double pi = 3.14159265358979323846;
        constexpr double max_yaw_span_rad = 5.0 * pi / 180.0;
        constexpr std::size_t min_samples = 50;

        const rclcpp::Time sample_time(stamp);
        const rclcpp::Time now = this->get_clock()->now();

        if (stability_samples_.empty() || sample_time > stability_samples_.back().time)
        {
            stability_samples_.push_back({sample_time, x, y, yaw});
        }

        while (stability_samples_.size() > 2 &&
               (sample_time - stability_samples_.front().time).seconds() > stability_window_s)
        {
            stability_samples_.pop_front();
        }

        bool stable = false;
        std::string reason = "collecting";
        double window_s = 0.0;
        double xy_drift = 0.0;
        double xy_span = 0.0;
        double xy_rate = 0.0;
        double yaw_span_deg = 0.0;
        const double tf_age_s = (now - sample_time).seconds();

        if (!imu_received_)
        {
            reason = "waiting_imu";
        }
        else if (tf_age_s > max_tf_age_s)
        {
            reason = "tf_stale";
        }
        else if (tf_age_s < -max_future_tf_s)
        {
            reason = "tf_from_future";
        }
        else if (stability_samples_.size() < min_samples)
        {
            reason = "collecting";
        }
        else
        {
            const auto &first = stability_samples_.front();
            const auto &last = stability_samples_.back();
            window_s = (last.time - first.time).seconds();

            if (window_s < stability_window_s * 0.80)
            {
                reason = "short_window";
            }
            else
            {
                double min_x = first.x;
                double max_x = first.x;
                double min_y = first.y;
                double max_y = first.y;
                double min_yaw_rel = 0.0;
                double max_yaw_rel = 0.0;

                for (const auto &sample : stability_samples_)
                {
                    min_x = std::min(min_x, sample.x);
                    max_x = std::max(max_x, sample.x);
                    min_y = std::min(min_y, sample.y);
                    max_y = std::max(max_y, sample.y);

                    const double yaw_rel = normalize_angle(sample.yaw - first.yaw);
                    min_yaw_rel = std::min(min_yaw_rel, yaw_rel);
                    max_yaw_rel = std::max(max_yaw_rel, yaw_rel);
                }

                xy_drift = std::hypot(last.x - first.x, last.y - first.y);
                xy_span = std::hypot(max_x - min_x, max_y - min_y);
                xy_rate = xy_drift / std::max(window_s, 1e-3);
                yaw_span_deg = (max_yaw_rel - min_yaw_rel) * 180.0 / pi;

                if (xy_drift > max_xy_drift_m)
                {
                    reason = "xy_drift";
                }
                else if (xy_span > max_xy_span_m)
                {
                    reason = "xy_span";
                }
                else if (xy_rate > max_xy_rate_mps)
                {
                    reason = "xy_rate";
                }
                else if ((max_yaw_rel - min_yaw_rel) > max_yaw_span_rad)
                {
                    reason = "yaw_span";
                }
                else
                {
                    stable = true;
                    reason = "ok";
                }
            }
        }

        if (stable)
        {
            if (!stability_ok_started_)
            {
                stable_since_ = now;
                stability_ok_started_ = true;
            }
        }
        else
        {
            stability_ok_started_ = false;
        }

        double stable_for_s = 0.0;
        if (stability_ok_started_)
        {
            stable_for_s = (now - stable_since_).seconds();
        }

        const bool ready_to_takeoff = stable && stable_for_s >= required_stable_s;
        if (stable && !ready_to_takeoff)
        {
            reason = "stabilizing";
        }

        publish_stability_status(ready_to_takeoff, reason, stable_for_s, window_s,
                                 stability_samples_.size(), xy_drift, xy_span,
                                 xy_rate, yaw_span_deg, tf_age_s);
    }

    void publish_stability_wait(const std::string &reason)
    {
        stability_ok_started_ = false;
        publish_stability_status(false, reason, 0.0, 0.0, stability_samples_.size(),
                                 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    void publish_stability_status(
        const bool ready_to_takeoff,
        const std::string &reason,
        const double stable_for_s,
        const double window_s,
        const std::size_t sample_count,
        const double xy_drift,
        const double xy_span,
        const double xy_rate,
        const double yaw_span_deg,
        const double tf_age_s)
    {
        std_msgs::msg::Bool stable_msg;
        stable_msg.data = ready_to_takeoff;
        ev_stable_pub_->publish(stable_msg);

        std_msgs::msg::String status_msg;
        std::ostringstream ss;
        ss << (ready_to_takeoff ? "OK" : "WAIT")
           << " reason=" << reason
           << std::fixed << std::setprecision(3)
           << " stable_for=" << stable_for_s << "s"
           << " window=" << window_s << "s"
           << " samples=" << sample_count
           << " xy_drift=" << xy_drift << "m"
           << " xy_span=" << xy_span << "m"
           << " xy_rate=" << xy_rate << "m/s"
           << " yaw_span=" << yaw_span_deg << "deg"
           << " tf_age=" << tf_age_s << "s";
        status_msg.data = ss.str();
        ev_stability_status_pub_->publish(status_msg);
    }

    double normalize_angle(const double angle) const
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    enum class HeightCompensationState
    {
        GROUND_NORMAL,
        TARGET_CANDIDATE,
        TARGET_OVERHEAD,
        TARGET_RELEASE
    };

    bool within_window(const double value, const double min_abs, const double max_abs) const
    {
        return value >= min_abs && value <= max_abs;
    }

    bool release_locked() const
    {
        return landing_on_target_ ||
               ((height_state_ == HeightCompensationState::TARGET_OVERHEAD ||
                 height_state_ == HeightCompensationState::TARGET_RELEASE) &&
                latest_range_z_enu_ <= landing_release_lock_z_m_);
    }

    void update_height_compensation(const rclcpp::Time &stamp, const double raw_z_enu)
    {
        if (!height_compensation_enabled_)
        {
            height_state_ = HeightCompensationState::GROUND_NORMAL;
            surface_bias_m_ = 0.0;
            return;
        }

        const double instant_dz = previous_raw_z_valid_ ? raw_z_enu - previous_raw_z_enu_ : 0.0;
        const int confirm_frames = std::max(1, jump_confirm_frames_);

        switch (height_state_)
        {
        case HeightCompensationState::GROUND_NORMAL:
            if (within_window(-instant_dz, jump_enter_min_m_, jump_enter_max_m_))
            {
                candidate_start_time_ = stamp;
                candidate_baseline_z_ = previous_raw_z_enu_;
                candidate_drop_peak_m_ = std::clamp(-instant_dz, target_surface_height_min_m_, target_surface_height_max_m_);
                surface_bias_m_ = candidate_drop_peak_m_;
                enter_candidate_count_ = 1;
                release_candidate_count_ = 0;
                height_state_ = HeightCompensationState::TARGET_CANDIDATE;
                zero_fake_vertical_velocity(stamp);
            }
            break;

        case HeightCompensationState::TARGET_CANDIDATE:
        {
            const double sustained_drop = candidate_baseline_z_ - raw_z_filtered_;
            if (within_window(sustained_drop, jump_enter_min_m_, jump_enter_max_m_))
            {
                ++enter_candidate_count_;
                candidate_drop_peak_m_ = std::max(candidate_drop_peak_m_, sustained_drop);
                surface_bias_m_ = std::clamp(
                    std::max(candidate_drop_peak_m_, target_surface_height_m_),
                    target_surface_height_min_m_,
                    target_surface_height_max_m_);
            }
            else if ((stamp - candidate_start_time_).seconds() > jump_confirm_time_s_)
            {
                enter_candidate_count_ = 0;
                candidate_drop_peak_m_ = 0.0;
                surface_bias_m_ = 0.0;
                height_state_ = HeightCompensationState::GROUND_NORMAL;
                break;
            }

            if (enter_candidate_count_ >= confirm_frames)
            {
                surface_bias_m_ = std::clamp(
                    std::max(candidate_drop_peak_m_, target_surface_height_m_),
                    target_surface_height_min_m_,
                    target_surface_height_max_m_);
                target_detected_ = true;
                height_state_ = HeightCompensationState::TARGET_OVERHEAD;
                zero_fake_vertical_velocity(stamp);
                RCLCPP_INFO(
                    this->get_logger(),
                    "Target surface detected. Applying height bias %.3f m.",
                    surface_bias_m_);
            }
            break;
        }

        case HeightCompensationState::TARGET_OVERHEAD:
            if (!release_locked() && within_window(instant_dz, jump_release_min_m_, jump_release_max_m_))
            {
                release_start_time_ = stamp;
                release_baseline_z_ = previous_raw_z_enu_;
                release_restore_bias_m_ = surface_bias_m_;
                surface_bias_m_ = 0.0;
                release_candidate_count_ = 1;
                height_state_ = HeightCompensationState::TARGET_RELEASE;
                zero_fake_vertical_velocity(stamp);
            }
            break;

        case HeightCompensationState::TARGET_RELEASE:
        {
            if (release_locked())
            {
                surface_bias_m_ = release_restore_bias_m_;
                release_candidate_count_ = 0;
                height_state_ = HeightCompensationState::TARGET_OVERHEAD;
                break;
            }

            const double sustained_rise = raw_z_filtered_ - release_baseline_z_;
            if (within_window(sustained_rise, jump_release_min_m_, jump_release_max_m_))
            {
                ++release_candidate_count_;
            }
            else if ((stamp - release_start_time_).seconds() > jump_confirm_time_s_)
            {
                surface_bias_m_ = release_restore_bias_m_;
                release_candidate_count_ = 0;
                height_state_ = HeightCompensationState::TARGET_OVERHEAD;
                break;
            }

            if (release_candidate_count_ >= confirm_frames)
            {
                surface_bias_m_ = 0.0;
                target_detected_ = false;
                height_state_ = HeightCompensationState::GROUND_NORMAL;
                zero_fake_vertical_velocity(stamp);
                RCLCPP_INFO(this->get_logger(), "Target surface released. Height bias cleared.");
            }
            break;
        }
        }
    }

    double filter_corrected_z(const double target_z)
    {
        if (!range_received_)
        {
            corrected_z_filtered_ = target_z;
            return corrected_z_filtered_;
        }

        const double alpha = std::clamp(corrected_z_filter_alpha_, 0.0, 1.0);
        double step = alpha * (target_z - corrected_z_filtered_);
        const double max_step = std::max(0.0, max_corrected_z_step_m_);
        if (max_step > 0.0)
        {
            step = std::clamp(step, -max_step, max_step);
        }
        corrected_z_filtered_ += step;
        return corrected_z_filtered_;
    }

    void zero_fake_vertical_velocity(const rclcpp::Time &stamp)
    {
        filtered_vz_enu_ = 0.0;
        fake_vz_zero_until_ = stamp + rclcpp::Duration::from_seconds(std::max(0.0, fake_vz_zero_time_s_));
        fake_vz_zero_active_ = true;
    }

    std::string height_state_name() const
    {
        if (landing_on_target_)
        {
            return "LAND_ON_TARGET";
        }

        switch (height_state_)
        {
        case HeightCompensationState::GROUND_NORMAL:
            return "GROUND_NORMAL";
        case HeightCompensationState::TARGET_CANDIDATE:
            return "TARGET_CANDIDATE";
        case HeightCompensationState::TARGET_OVERHEAD:
            return "TARGET_OVERHEAD";
        case HeightCompensationState::TARGET_RELEASE:
            return "TARGET_RELEASE";
        }
        return "UNKNOWN";
    }

    void publish_compensated_range(const sensor_msgs::msg::Range &raw_msg, const double corrected_z)
    {
        sensor_msgs::msg::Range range_msg = raw_msg;
        range_msg.header.frame_id = raw_msg.header.frame_id;
        range_msg.range = static_cast<float>(std::clamp(
            corrected_z - rangefinder_z_offset_m_,
            static_cast<double>(raw_msg.min_range),
            static_cast<double>(raw_msg.max_range)));
        range_px4_pub_->publish(range_msg);
    }

    void publish_height_debug()
    {
        std_msgs::msg::Float64 value_msg;

        value_msg.data = latest_raw_range_m_;
        raw_range_debug_pub_->publish(value_msg);

        value_msg.data = latest_raw_z_enu_;
        raw_z_debug_pub_->publish(value_msg);

        value_msg.data = latest_range_z_enu_;
        corrected_z_debug_pub_->publish(value_msg);

        value_msg.data = surface_bias_m_;
        surface_bias_debug_pub_->publish(value_msg);

        std_msgs::msg::String state_msg;
        state_msg.data = height_state_name();
        height_state_debug_pub_->publish(state_msg);

        std_msgs::msg::Bool detected_msg;
        detected_msg.data = target_detected_;
        target_detected_debug_pub_->publish(detected_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_debug_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vision_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ev_stable_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ev_stability_status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_px4_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_range_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_z_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr corrected_z_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr surface_bias_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr height_state_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_detected_debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr landing_on_target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Time latest_imu_time_;
    bool imu_received_ = false;
    bool local_pose_received_ = false;
    double latest_local_z_ = 0.0;
    bool range_received_ = false;
    rclcpp::Time latest_range_time_;
    double latest_range_z_enu_ = 0.0;
    double latest_raw_range_m_ = 0.0;
    double latest_raw_z_enu_ = 0.0;
    double raw_z_filtered_ = 0.0;
    double corrected_z_filtered_ = 0.0;
    double previous_raw_z_enu_ = 0.0;
    bool previous_raw_z_valid_ = false;
    double filtered_vz_enu_ = 0.0;
    double rangefinder_z_offset_m_ = 0.115;
    bool height_compensation_enabled_ = true;
    double target_surface_height_m_ = 0.19;
    double target_surface_height_min_m_ = 0.11;
    double target_surface_height_max_m_ = 0.30;
    double jump_enter_min_m_ = 0.10;
    double jump_enter_max_m_ = 0.32;
    double jump_release_min_m_ = 0.10;
    double jump_release_max_m_ = 0.32;
    int jump_confirm_frames_ = 3;
    double jump_confirm_time_s_ = 0.20;
    double raw_z_filter_alpha_ = 0.35;
    double corrected_z_filter_alpha_ = 0.45;
    double max_corrected_z_step_m_ = 0.04;
    double fake_vz_zero_time_s_ = 0.25;
    double landing_release_lock_z_m_ = 0.65;
    double surface_bias_m_ = 0.0;
    bool landing_on_target_ = false;
    bool target_detected_ = false;
    HeightCompensationState height_state_ = HeightCompensationState::GROUND_NORMAL;
    rclcpp::Time candidate_start_time_;
    double candidate_baseline_z_ = 0.0;
    double candidate_drop_peak_m_ = 0.0;
    int enter_candidate_count_ = 0;
    rclcpp::Time release_start_time_;
    double release_baseline_z_ = 0.0;
    double release_restore_bias_m_ = 0.0;
    int release_candidate_count_ = 0;
    rclcpp::Time fake_vz_zero_until_;
    bool fake_vz_zero_active_ = false;

    struct PoseSample
    {
        rclcpp::Time time;
        double x = 0.0;
        double y = 0.0;
    };

    std::deque<PoseSample> pose_samples_;
    double filtered_vx_body_ = 0.0;
    double filtered_vy_body_ = 0.0;

    struct StabilitySample
    {
        rclcpp::Time time;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    std::deque<StabilitySample> stability_samples_;
    rclcpp::Time stable_since_;
    bool stability_ok_started_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartographerLaserTransfer>());
    rclcpp::shutdown();
    return 0;
}
