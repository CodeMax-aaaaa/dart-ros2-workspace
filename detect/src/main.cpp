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
    DetectPublisher(std::chrono::milliseconds interval)
        : Node("detect_node"), it_(std::make_shared<rclcpp::Node>("detect_publisher"))
    {
        publisher_ = this->create_publisher<info::msg::GreenLight>("detect/locate", 1);
        image_publisher_ = it_.advertise("detect/img", 1);
        image_subscriber_ = it_.subscribe("camera/img", 1, &DetectPublisher::image_callback, this);
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
