#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <string>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <unistd.h>

#include "DHVideoCapture.h"

using namespace std::chrono_literals;

class NodeCamera : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_test_;
    rclcpp::TimerBase::SharedPtr timer_;

    DHVideoCapture _videoCapture;
    int exposure_time;

    void initCamera(int exposure_time) // 1-3ms 0=>auto
    {
        RCLCPP_INFO(this->get_logger(), "Initializing camera...");
        if (!_videoCapture.open(0, 2))
        {
            RCLCPP_ERROR(this->get_logger(), "!!!!!!!!!!!!!!!!!!!!!no camera!!!!!!!!!!!!!!!!!!!!!!!!!");
            rclcpp::shutdown();
        }
        _videoCapture.setExposureTime(exposure_time);
        _videoCapture.setVideoFormat(1280, 1024, true);
        _videoCapture.setFPS(30.0);
        _videoCapture.setBalanceRatio(1.6, 1.3, 2.0, true);
        _videoCapture.setGain(1);
        _videoCapture.setGamma(1);

        _videoCapture.startStream();
        _videoCapture.closeStream();

        _videoCapture.startStream();
        RCLCPP_INFO(this->get_logger(), "Camera initialized.");
    }

    void camera_capture_thread()
    {
        // 从相机读取图像
        int width = 1280;
        int height = 1024;
        cv::Mat image(height, width, CV_8UC3);
        RCLCPP_INFO(this->get_logger(), "Start capturing...");
        std::stringstream ss;
        auto now = std::time(nullptr);
        ss << "camera." << std::put_time(std::localtime(&now), "%y%m%d%H%M%S") << ".avi";
        char cwd[256];
        getcwd(cwd, sizeof(cwd));
        RCLCPP_INFO_STREAM(this->get_logger(), "Recording to " << cwd << "/" << ss.str());
        // 录像到当前工作目录，文件名为camera.xxxxxxxxxx(yymmddhhmmss)，格式为avi，30fps
        cv::VideoWriter video(ss.str(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(width, height));

        while (rclcpp::ok())
        {
            _videoCapture.read(image);
            video.write(image);

            // 每3次发布图像 帧率10fps
            static int count = 0;
            if (count++ % 3 == 0)
            {
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                publisher_test_->publish(*msg);
            }
            std::this_thread::sleep_for(33ms); // 30fps
        }
        video.release();
        RCLCPP_INFO(this->get_logger(), "Capture thread stopped.");
    }

public:
    NodeCamera() : Node("camera")
    {
        exposure_time = 0;
        publisher_test_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 1);
        initCamera(exposure_time);
        std::thread(std::bind(&NodeCamera::camera_capture_thread, this)).detach();
        // 参数declare
        this->declare_parameter("exposure_time", 0);
        // 参数回调
        timer_ = this->create_wall_timer(1s, [this]()
                                         {
            int exposure_time_present = this->get_parameter("exposure_time").as_int();
            if(exposure_time_present != exposure_time)
            {
                exposure_time = exposure_time_present;
                _videoCapture.setExposureTime(exposure_time);
                RCLCPP_INFO(this->get_logger(), "Exposure time set to: %d", exposure_time);
            } });
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeCamera>());
    rclcpp::shutdown();
    return 0;
}