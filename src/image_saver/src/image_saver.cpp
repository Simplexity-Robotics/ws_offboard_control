#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

class ImageSaver : public rclcpp::Node
{
public:
    ImageSaver() : Node("image_saver"), image_count_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_0", 10, std::bind(&ImageSaver::topic_callback, this, std::placeholders::_1));

        save_path_ = "saved_images";
        if (!fs::exists(save_path_))
        {
            fs::create_directory(save_path_);
        }
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Receiving image frame");
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        std::string image_filename = save_path_ + "/image_" + std::to_string(image_count_) + ".png";
        cv::imwrite(image_filename, frame);
        image_count_++;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    mutable size_t image_count_;
    std::string save_path_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaver>());
    rclcpp::shutdown();
    return 0;
}