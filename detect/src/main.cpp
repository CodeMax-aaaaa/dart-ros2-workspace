#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include "info/msg/green_light.hpp"
#include "detect/detect.h"
#include <chrono>

class DetectPublisher : public rclcpp::Node
{
public:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_test_;
    rclcpp::TimerBase::SharedPtr timer_;

    DetectPublisher(std::chrono::milliseconds interval)
        : Node("detect_node"), it_(std::make_shared<rclcpp::Node>("detect_publisher"))
    {
        publisher_ = this->create_publisher<info::msg::GreenLight>("detect/locate", 1);
        image_publisher_ = it_.advertise("detect/img", 1);
        image_subscriber_ = it_.subscribe("camera/img", 1, &DetectPublisher::image_callback, this);

        publisher_test_ = this->create_publisher<sensor_msgs::msg::Image>("test/image", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]()
            {
                // 创建一个简单的黑白格子图像
                int width = 100;
                int height = 100;
                cv::Mat image(height, width, CV_8UC1);

                for (int y = 0; y < height; ++y)
                {
                    for (int x = 0; x < width; ++x)
                    {
                        if ((x / 10 + y / 10) % 2 == 0)
                        {
                            image.at<uchar>(y, x) = 255; // 白色
                        }
                        else
                        {
                            image.at<uchar>(y, x) = 0; // 黑色
                        }
                    }
                }

                // 将OpenCV图像转换为ROS图像消息
                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = "camera";

                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
                publisher_test_->publish(*msg);
            });
    }

private:
    cv::Mat perform_detection(cv::Mat &frame, bool &is_detected, double &x, double &y)
    {
        if (detector_.detect(frame))
        {
            is_detected = true;
            cv::Point2f center;
            detector_.getResult(center);
            x = center.x;
            y = center.y;
        }
        return detector_.debugDraw();
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &frame_msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(frame_msg, "bgr8")->image;
        if (!frame.empty())
        {
            bool is_detected = false;
            double x = 0, y = 0;
            cv::Mat resultImg = perform_detection(frame, is_detected, x, y);
            auto message = info::msg::GreenLight();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "base_link";
            message.is_detected = is_detected;
            message.location.x = x;
            message.location.y = y;
            message.location.z = 0.0;

            RCLCPP_INFO(this->get_logger(), "Publishing: is_detected='%s', x='%f', y='%f'", is_detected ? "true" : "false", x, y);
            publisher_->publish(message);

            std_msgs::msg::Header header;
            header.stamp = this->now();
            auto image_msg = cv_bridge::CvImage(header, "bgr8", resultImg).toImageMsg();
            image_publisher_.publish(image_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get image from camera");
        }
    }

    TopArmorDetect detector_;
    rclcpp::Publisher<info::msg::GreenLight>::SharedPtr publisher_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_publisher_;
    image_transport::Subscriber image_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::chrono::milliseconds interval(1000 / 60);

    auto node = std::make_shared<DetectPublisher>(interval);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
