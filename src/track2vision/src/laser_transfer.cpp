#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
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

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&CartographerLaserTransfer::imu_callback, this, std::placeholders::_1));

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

    void publish_vision_odometry()
    {
        if (!imu_received_)
        {
            publish_stability_wait("waiting_imu");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "IMU data not received yet, waiting...");
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
        pose_msg.pose.position.z = 0.0;
        pose_msg.pose.orientation = tf2::toMsg(q_yaw);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.twist.twist.linear.x = filtered_vx_body_;
        odom_msg.twist.twist.linear.y = filtered_vy_body_;
        odom_msg.twist.twist.linear.z = 0.0;
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
        odom_msg.pose.covariance[14] = 1.0;
        odom_msg.pose.covariance[21] = 0.25;
        odom_msg.pose.covariance[28] = 0.25;
        odom_msg.pose.covariance[35] = 0.10;

        odom_msg.twist.covariance[0] = 0.04;
        odom_msg.twist.covariance[7] = 0.04;
        odom_msg.twist.covariance[14] = 1.0;
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

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_debug_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vision_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ev_stable_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ev_stability_status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Time latest_imu_time_;
    bool imu_received_ = false;

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
