#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>  // For creating directories

namespace fs = std::filesystem;

class BuoyDetector : public rclcpp::Node
{
public:
    BuoyDetector() : Node("buoy_detector"), image_counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Starting up BuoyDetector");
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_0", 10, std::bind(&BuoyDetector::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("contour_center", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
        image_publisher_mask = this->create_publisher<sensor_msgs::msg::Image>("processed_image_mask", 10);
        image_publisher_blurred = this->create_publisher<sensor_msgs::msg::Image>("processed_image_blurred", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

            // Center crop the image
            cv::Mat cropped_image;
            center_crop(image, cropped_image);

            // Convert to HSV color space
            cv::Mat hsv_image;
            cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);

            // Define two ranges for red color in HSV
            cv::Scalar lower_red1(0, 50, 0);
            cv::Scalar upper_red1(10, 255, 255);
            cv::Scalar lower_red2(140, 50, 0);
            cv::Scalar upper_red2(179, 255, 255);

            // Create binary masks for both red ranges
            cv::Mat mask1, mask2;
            cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
            cv::inRange(hsv_image, lower_red2, upper_red2, mask2);

            // Combine the masks
            cv::Mat mask = mask1 | mask2;

            // Apply morphological operations to reduce noise and fill gaps
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            // Apply GaussianBlur to reduce noise before contour detection
            cv::Mat blurred;
            cv::GaussianBlur(mask, blurred, cv::Size(9, 9), 2, 2);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(blurred, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Find the largest contour
            double max_area = 0;
            std::vector<cv::Point> largest_contour;
            for (const auto &contour : contours)
            {
                double area = cv::contourArea(contour);
                if (area > max_area)
                {
                    max_area = area;
                    largest_contour = contour;
                }
            }

            // If a contour was found
            if (!largest_contour.empty())
            {
                // Compute the center of the largest contour
                cv::Moments moments = cv::moments(largest_contour);
                cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);

                // Draw the largest contour and its center
                cv::Mat display_image = cropped_image.clone();
                cv::drawContours(display_image, std::vector<std::vector<cv::Point>>{largest_contour}, -1, cv::Scalar(0, 255, 0), 2);
                cv::circle(display_image, center, 5, cv::Scalar(0, 0, 255), -1);

                // Publish the center point of the largest contour
                geometry_msgs::msg::Point center_msg;
                center_msg.x = static_cast<double>(center.x);
                center_msg.y = static_cast<double>(center.y);
                center_msg.z = static_cast<double>(max_area);
                publisher_->publish(center_msg);

                // Publish the processed image
                sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", display_image).toImageMsg();
                sensor_msgs::msg::Image::SharedPtr out_msg_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
                sensor_msgs::msg::Image::SharedPtr out_msg_blurred = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", blurred).toImageMsg();

                image_publisher_->publish(*out_msg);
                image_publisher_mask->publish(*out_msg_mask);
                image_publisher_blurred->publish(*out_msg_blurred);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in image_callback: %s", e.what());
        }
    }

    // Function to center crop the image to a specified width and height
    void center_crop(const cv::Mat &input, cv::Mat &output)
    {
        int crop_width = 640;
        int crop_height = 480;

        int x = (input.cols - crop_width) / 2;
        int y = (input.rows - crop_height) / 2;

        cv::Rect crop_rect(x, y, crop_width, crop_height);

        // Ensure the crop rectangle is within the image bounds
        crop_rect &= cv::Rect(0, 0, input.cols, input.rows);

        output = input(crop_rect).clone();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_mask;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_blurred;

    int image_counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuoyDetector>());
    rclcpp::shutdown();
    return 0;
}

// OLD CODE BELOW


// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <filesystem>  // For creating directories

// namespace fs = std::filesystem;

// class BuoyDetector : public rclcpp::Node
// {
// public:
//     BuoyDetector() : Node("buoy_detector"), image_counter_(0)
//     {
//         RCLCPP_INFO(this->get_logger(), "Starting up BuoyDetector");
//         subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera_0", 10, std::bind(&BuoyDetector::image_callback, this, std::placeholders::_1));

//         publisher_ = this->create_publisher<geometry_msgs::msg::Point>("circle_center", 10);
//         largest_circle_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("largest_circle_center", 10);
//         image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
//         image_publisher_mask = this->create_publisher<sensor_msgs::msg::Image>("processed_image_mask", 10);
//         image_publisher_blurred = this->create_publisher<sensor_msgs::msg::Image>("processed_image_blurred", 10);


//         // Create directory for saving images if it doesn't exist
//         // try {
//         //     if (fs::create_directory("saved_images")) {
//         //         RCLCPP_INFO(this->get_logger(), "Directory 'saved_images' created successfully.");
//         //     } else {
//         //         RCLCPP_INFO(this->get_logger(), "Directory 'saved_images' already exists.");
//         //     }
//         // } catch (const std::exception& e) {
//         //     RCLCPP_ERROR(this->get_logger(), "Failed to create directory 'saved_images': %s", e.what());
//         // }
//     }

// private:
    

//     void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         try
//         {
//             cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

//             // Center crop the image
//             cv::Mat cropped_image;
//             center_crop(image, cropped_image);

//             // Convert to HSV color space
//             cv::Mat hsv_image;
//             cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);

//             // Define two ranges for red color in HSV
//             cv::Scalar lower_red1(0, 0, 0);
//             cv::Scalar upper_red1(30, 255, 255);
//             cv::Scalar lower_red2(140, 0, 0);
//             cv::Scalar upper_red2(179, 255, 255);

//             // Create binary masks for both red ranges
//             cv::Mat mask1, mask2;
//             cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
//             cv::inRange(hsv_image, lower_red2, upper_red2, mask2);

//             // Combine the masks
//             cv::Mat mask = mask1 | mask2;

//             // Apply morphological operations to reduce noise and fill gaps
//             cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
//             cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
//             cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

//             // Apply GaussianBlur to reduce noise before circle detection
//             cv::Mat blurred;
//             cv::GaussianBlur(mask, blurred, cv::Size(9, 9), 2, 2);

//             // Detect circles using HoughCircles
//             std::vector<cv::Vec3f> circles;
//             cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1,
//                             blurred.rows/16,  // minimum distance between circle centers
//                             100, 30, 1, 640 // other parameters
//             );

//             // Draw the detected circles on the original image
//             cv::Mat display_image = cropped_image.clone();
//             cv::Vec3f largest_circle(0, 0, 0);
//             for(const auto& circle : circles)
//             {
//                 cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
//                 int radius = cvRound(circle[2]);
//                 // draw the circle center
//                 cv::circle(display_image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
//                 // draw the circle outline
//                 cv::circle(display_image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);

//                 // Publish the center point of the detected circle
//                 geometry_msgs::msg::Point center_msg;
//                 center_msg.x = static_cast<double>(circle[0]);
//                 center_msg.y = static_cast<double>(circle[1]);
//                 center_msg.z = 0.0;  // Assuming 2D image
//                 publisher_->publish(center_msg);

//                 // Update largest circle if current circle is larger
//                 if(circle[2] > largest_circle[2]) {
//                     largest_circle = circle;
//                 }
//             }

//             // Publish the largest circle's center
//             if(largest_circle[2] > 0) {
//                 geometry_msgs::msg::Point largest_center_msg;
//                 largest_center_msg.x = static_cast<double>(largest_circle[0]);
//                 largest_center_msg.y = static_cast<double>(largest_circle[1]);
//                 largest_center_msg.z = static_cast<double>(largest_circle[2]); // THIS IS RADIUS
//                 largest_circle_publisher_->publish(largest_center_msg);

//                 // Draw the largest circle in a different color (e.g., yellow)
//                 cv::Point largest_center(cvRound(largest_circle[0]), cvRound(largest_circle[1]));
//                 int largest_radius = cvRound(largest_circle[2]);
//                 cv::circle(display_image, largest_center, largest_radius, cv::Scalar(0,255,255), 5, 8, 0);
//             }

//             // Publish the processed image
//             sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", display_image).toImageMsg();
//             sensor_msgs::msg::Image::SharedPtr out_msg_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
//             sensor_msgs::msg::Image::SharedPtr out_msg_blurred = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", blurred).toImageMsg();

//             image_publisher_->publish(*out_msg);
//             image_publisher_mask->publish(*out_msg_mask);
//             image_publisher_blurred->publish(*out_msg_blurred);

//             // // Save the processed image and mask
//             // std::string filename_mask = "saved_images/mask" + std::to_string(image_counter_) + ".png";
//             // std::string filename_original = "saved_images/image" + std::to_string(image_counter_) + ".png";
//             // std::string filename_processed = "saved_images/processed" + std::to_string(image_counter_) + ".png";
            
//             // if (cv::imwrite(filename_mask, mask) && 
//             //     cv::imwrite(filename_original, image) &&
//             //     cv::imwrite(filename_processed, display_image))
//             // {
//             //     RCLCPP_INFO(this->get_logger(), "Images saved successfully: %s", filename_mask.c_str());
//             // }
//             // else
//             // {
//             //     RCLCPP_ERROR(this->get_logger(), "Failed to save images");
//             // }
//             // image_counter_++;
//         }
//         catch (cv_bridge::Exception& e)
//         {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//             return;
//         }
//         catch (const std::exception& e)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Exception in image_callback: %s", e.what());
//         }
//     }

//     // Function to center crop the image to a specified width and height
//     void center_crop(const cv::Mat& input, cv::Mat& output)
//     {
//         int crop_width = 640;
//         int crop_height = 480;

//         int x = (input.cols - crop_width) / 2;
//         int y = (input.rows - crop_height) / 2;

//         cv::Rect crop_rect(x, y, crop_width, crop_height);

//         // Ensure the crop rectangle is within the image bounds
//         crop_rect &= cv::Rect(0, 0, input.cols, input.rows);

//         output = input(crop_rect).clone();
//     }

//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
//     rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
//     rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr largest_circle_publisher_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_mask;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_blurred;

//     int image_counter_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<BuoyDetector>());
//     rclcpp::shutdown();
//     return 0;
// }