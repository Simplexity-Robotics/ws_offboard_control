#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>  // For directory creation
#include <chrono>  // For timestamp

class ImageProcessor : public rclcpp::Node {
public:
  ImageProcessor() : Node("image_processor"), image_counter(0) {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera_0", 10, std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bounding_box_center", 10);

    // Create the directory for saving images
    std::filesystem::create_directories("saved_images");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat hsv_frame, mask;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    
    // Define the range for orange/red color in HSV
    cv::Scalar lower_red1(0, 100, 100);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100);
    cv::Scalar upper_red2(179, 255, 255);
    
    cv::Mat mask1, mask2;
    cv::inRange(hsv_frame, lower_red1, upper_red1, mask1);
    cv::inRange(hsv_frame, lower_red2, upper_red2, mask2);
    mask = mask1 | mask2;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      auto largest_contour = std::max_element(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
        return cv::contourArea(a) < cv::contourArea(b);
      });

      cv::Rect bounding_box = cv::boundingRect(*largest_contour);
      cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);

      double center_x = bounding_box.x + bounding_box.width / 2.0;
      double center_y = bounding_box.y + bounding_box.height / 2.0;

      std_msgs::msg::Float64MultiArray center_msg;
      center_msg.data.push_back(center_x);
      center_msg.data.push_back(center_y);

      publisher_->publish(center_msg);

      // Save the image with the bounding box
      auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
      std::string filename = "saved_images/image_" + std::to_string(timestamp) + ".png";
      cv::imwrite(filename, frame);
    }

    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  int image_counter;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}
