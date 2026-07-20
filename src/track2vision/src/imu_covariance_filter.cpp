#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuCovarianceFilter : public rclcpp::Node
{
public:
    ImuCovarianceFilter() : Node("imu_covariance_filter")
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/track2vision/imu/data_valid",
            rclcpp::SensorDataQoS());

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(&ImuCovarianceFilter::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Filtering /mavros/imu/data into /track2vision/imu/data_valid.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (msg->linear_acceleration_covariance[0] == -1.0) {
            ++dropped_invalid_imu_count_;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Dropping IMU messages without linear acceleration covariance. Dropped: %zu",
                dropped_invalid_imu_count_);
            return;
        }

        if (!published_first_valid_imu_) {
            published_first_valid_imu_ = true;
            RCLCPP_INFO(this->get_logger(), "First valid IMU message received.");
        }

        imu_pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    size_t dropped_invalid_imu_count_ = 0;
    bool published_first_valid_imu_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuCovarianceFilter>());
    rclcpp::shutdown();
    return 0;
}
