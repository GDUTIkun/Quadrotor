#include "yolo_detect/yolo_ncnn.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace yolo_detect
{

class YoloSnapshotServiceNode final : public rclcpp::Node
{
public:
  YoloSnapshotServiceNode()
  : Node("yolo_snapshot_service_node")
  {
    const std::string default_model_dir =
      ament_index_cpp::get_package_share_directory("yolo_detect") +
      "/model/best_ncnn_model";

    const auto model_dir = declare_parameter<std::string>("model_dir", default_model_dir);
    const auto image_topic = declare_parameter<std::string>("image_topic", "/image_raw");
    const auto service_name =
      declare_parameter<std::string>("service_name", "/yolo/capture_and_detect");
    const auto detections_topic = declare_parameter<std::string>(
      "detections_topic", "/yolo/snapshot_detections");
    const auto snapshot_topic =
      declare_parameter<std::string>("snapshot_topic", "/yolo/snapshot_image");
    const auto annotated_topic = declare_parameter<std::string>(
      "annotated_topic", "/yolo/snapshot_annotated_image");
    const int input_size = declare_parameter<int>("input_size", 640);
    const double confidence = declare_parameter<double>("confidence_threshold", 0.25);
    const double iou = declare_parameter<double>("iou_threshold", 0.45);
    const int num_threads = declare_parameter<int>("num_threads", 3);
    const int max_detections = declare_parameter<int>("max_detections", 300);
    const bool use_vulkan = declare_parameter<bool>("use_vulkan", false);
    const bool use_fp16 = declare_parameter<bool>("use_fp16", true);
    publish_snapshot_ = declare_parameter<bool>("publish_snapshot", true);
    publish_annotated_ = declare_parameter<bool>("publish_annotated", true);
    save_annotated_ = declare_parameter<bool>("save_annotated", true);
    save_directory_ =
      declare_parameter<std::string>("save_directory", "/home/tan/ros2_ws/snapshots");
    if (save_annotated_) {
      std::filesystem::create_directories(save_directory_);
    }
    crop_width_ratio_ = declare_parameter<double>("crop_width_ratio", 0.5);
    crop_height_ratio_ = declare_parameter<double>("crop_height_ratio", 0.5);
    if (
      crop_width_ratio_ <= 0.0 || crop_width_ratio_ > 1.0 ||
      crop_height_ratio_ <= 0.0 || crop_height_ratio_ > 1.0)
    {
      throw std::invalid_argument("crop_width_ratio and crop_height_ratio must be in (0, 1]");
    }
    class_names_ = declare_parameter<std::vector<std::string>>(
      "class_names", {"elephant", "peacock", "monkey", "tiger", "wolf"});
    if (class_names_.size() != 5U) {
      throw std::invalid_argument("class_names must contain exactly 5 names");
    }

    detector_ = std::make_unique<YoloNcnn>(
      model_dir, input_size, static_cast<float>(confidence), static_cast<float>(iou),
      num_threads, max_detections, use_vulkan, use_fp16);

    detections_pub_ =
      create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic, 10);
    if (publish_snapshot_) {
      snapshot_pub_ = create_publisher<sensor_msgs::msg::Image>(snapshot_topic, 1);
    }
    if (publish_annotated_) {
      annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic, 1);
    }

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&YoloSnapshotServiceNode::image_callback, this, std::placeholders::_1));
    service_ = create_service<std_srvs::srv::Trigger>(
      service_name,
      std::bind(
        &YoloSnapshotServiceNode::service_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Snapshot detector ready: service=%s camera=%s center_crop=%.0f%%x%.0f%%",
      service_name.c_str(), image_topic.c_str(),
      crop_width_ratio_ * 100.0, crop_height_ratio_ * 100.0);
  }

private:
  void service_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (capture_requested_) {
      response->success = false;
      response->message = "A capture request is already waiting for the next camera frame";
      return;
    }
    capture_requested_ = true;
    response->success = true;
    response->message =
      "Capture accepted; the next camera frame will be inferred and published";
    RCLCPP_INFO(get_logger(), "Capture requested; waiting for the next frame");
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    if (!capture_requested_) {
      return;
    }
    capture_requested_ = false;

    try {
      const auto cv_image =
        cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
      const cv::Rect crop = center_crop(cv_image->image.size());
      auto inference = detector_->infer(cv_image->image(crop));
      for (auto & detection : inference.detections) {
        detection.box.x += static_cast<float>(crop.x);
        detection.box.y += static_cast<float>(crop.y);
      }

      if (publish_snapshot_) {
        snapshot_pub_->publish(*message);
      }
      publish_detections(*message, inference.detections);
      if (publish_annotated_ || save_annotated_) {
        publish_annotated(*message, cv_image->image, crop, inference.detections);
      }
      log_result(inference);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Snapshot inference failed: %s", error.what());
    }
  }

  cv::Rect center_crop(const cv::Size & image_size) const
  {
    const int width = std::clamp(
      static_cast<int>(std::round(image_size.width * crop_width_ratio_)),
      1, image_size.width);
    const int height = std::clamp(
      static_cast<int>(std::round(image_size.height * crop_height_ratio_)),
      1, image_size.height);
    return cv::Rect(
      (image_size.width - width) / 2,
      (image_size.height - height) / 2,
      width, height);
  }

  void publish_detections(
    const sensor_msgs::msg::Image & source,
    const std::vector<Detection> & detections)
  {
    vision_msgs::msg::Detection2DArray output;
    output.header = source.header;
    output.detections.reserve(detections.size());
    for (std::size_t index = 0; index < detections.size(); ++index) {
      const auto & detection = detections[index];
      vision_msgs::msg::Detection2D message;
      message.header = source.header;
      message.id = std::to_string(index);
      message.bbox.center.position.x = detection.box.x + detection.box.width * 0.5;
      message.bbox.center.position.y = detection.box.y + detection.box.height * 0.5;
      message.bbox.center.theta = 0.0;
      message.bbox.size_x = detection.box.width;
      message.bbox.size_y = detection.box.height;

      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id =
        class_names_.at(static_cast<std::size_t>(detection.class_id));
      hypothesis.hypothesis.score = detection.confidence;
      hypothesis.pose.pose.orientation.w = 1.0;
      message.results.push_back(std::move(hypothesis));
      output.detections.push_back(std::move(message));
    }
    detections_pub_->publish(std::move(output));
  }

  void publish_annotated(
    const sensor_msgs::msg::Image & source,
    const cv::Mat & image,
    const cv::Rect & crop,
    const std::vector<Detection> & detections)
  {
    static const std::array<cv::Scalar, 5> colors = {
      cv::Scalar(0, 165, 255), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 0),
      cv::Scalar(0, 0, 255), cv::Scalar(255, 128, 0)};
    cv::Mat annotated = image.clone();
    cv::rectangle(annotated, crop, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    for (const auto & detection : detections) {
      const auto color = colors.at(static_cast<std::size_t>(detection.class_id));
      cv::rectangle(annotated, detection.box, color, 2, cv::LINE_AA);
      std::ostringstream label;
      label << class_names_.at(static_cast<std::size_t>(detection.class_id)) << ' '
            << std::fixed << std::setprecision(2) << detection.confidence;
      int baseline = 0;
      const cv::Size text_size =
        cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
      const int label_x = std::max(0, static_cast<int>(detection.box.x));
      const int label_y =
        std::max(text_size.height + 4, static_cast<int>(detection.box.y));
      cv::rectangle(
        annotated,
        cv::Rect(
          label_x, label_y - text_size.height - 4,
          text_size.width + 6, text_size.height + 6),
        color, cv::FILLED);
      cv::putText(
        annotated, label.str(), cv::Point(label_x + 3, label_y),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    if (save_annotated_) {
      const std::string filename =
        save_directory_ + "/snapshot_" +
        std::to_string(source.header.stamp.sec) + "_" +
        std::to_string(source.header.stamp.nanosec) + ".jpg";
      if (!cv::imwrite(filename, annotated)) {
        throw std::runtime_error("Failed to save annotated snapshot: " + filename);
      }
      RCLCPP_INFO(get_logger(), "Saved inference image: %s", filename.c_str());
    }
    if (publish_annotated_) {
      annotated_pub_->publish(
        *cv_bridge::CvImage(
          source.header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg());
    }
  }

  void log_result(const InferenceResult & result) const
  {
    if (result.detections.empty()) {
      RCLCPP_INFO(
        get_logger(), "Snapshot result: none (inference=%.1fms)", result.inference_ms);
      return;
    }
    for (std::size_t index = 0; index < result.detections.size(); ++index) {
      const auto & detection = result.detections[index];
      RCLCPP_INFO(
        get_logger(),
        "Snapshot result [%zu]: class=%s confidence=%.3f "
        "bbox=(x=%.1f, y=%.1f, width=%.1f, height=%.1f)",
        index,
        class_names_.at(static_cast<std::size_t>(detection.class_id)).c_str(),
        detection.confidence,
        detection.box.x, detection.box.y, detection.box.width, detection.box.height);
    }
  }

  std::unique_ptr<YoloNcnn> detector_;
  std::vector<std::string> class_names_;
  bool capture_requested_{false};
  bool publish_snapshot_{true};
  bool publish_annotated_{true};
  bool save_annotated_{true};
  std::string save_directory_;
  double crop_width_ratio_{0.5};
  double crop_height_ratio_{0.5};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
};

}  // namespace yolo_detect

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<yolo_detect::YoloSnapshotServiceNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo_snapshot_service_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
