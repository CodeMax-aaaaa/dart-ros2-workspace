#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <string>
#include <unordered_map>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "camera_hal/camera_driver.hpp"
#include <detector/zbar_detect.h>
#include <detector/greenlight_detect.h>

#include <std_msgs/msg/string.hpp>
#include <dart_msgs/msg/green_light.hpp>

#include "camera_hal/camera_dh.hpp"
#include "camera_hal/camera_lccv.hpp"

using namespace std::chrono_literals;
using namespace CameraHAL;

class DartLauncherDetectorNode : public rclcpp_lifecycle::LifecycleNode
{
private:
    std::shared_ptr<CameraDriver> camera_lccv_;
    std::shared_ptr<CameraDriver> camera_dh_;
    QRCodeDetectorZB qr_detector_;
    TopArmorDetect greenlight_detector_;

    rclcpp_lifecycle::LifecyclePublisher<dart_msgs::msg::GreenLight>::SharedPtr greenlight_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr greenlight_image_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr qr_detect_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr qr_image_publisher_;

    std::shared_ptr<std::thread> lccv_thread_;
    std::shared_ptr<std::thread> dh_thread_;

    bool running_;
    bool lccv_enabled_;
    bool dh_enabled_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_set_parameter_handle;

    void camera_thread(std::shared_ptr<CameraDriver> camera, const std::string &camera_name, bool is_qr_detection)
    {
        RCLCPP_INFO(this->get_logger(), "Starting thread for %s camera...", camera_name.c_str());

        int width = this->get_parameter(camera_name + ".image_width").as_int();
        int height = this->get_parameter(camera_name + ".image_height").as_int();
        RCLCPP_DEBUG(this->get_logger(), "%s camera resolution: %dx%d", camera_name.c_str(), width, height);

        cv::Mat image(height, width, CV_8UC3);
        if (is_qr_detection)
        {
            // Activate publishers
            qr_image_publisher_->on_activate();
            qr_detect_publisher_->on_activate();

            std::string last_detected_qr_code_str;
            int reset_last_detected_qr_code_counter = 0;
            while (running_ && rclcpp::ok())
            {
                if (!camera->read(image))
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to read image from %s camera", camera_name.c_str());
                    std::this_thread::sleep_for(16ms);
                    continue;
                }
                RCLCPP_DEBUG(this->get_logger(), "Image read successfully from %s camera", camera_name.c_str());

                auto qr_codes = qr_detector_.detect(image);
                if (!qr_codes.empty() && qr_codes[0] != last_detected_qr_code_str)
                {
                    RCLCPP_INFO(this->get_logger(), "Detected QR codes from %s: %s", camera_name.c_str(), qr_codes[0].c_str());
                    cv::imwrite("./qr_image.jpg", image);
                    std_msgs::msg::String qr_msg;
                    qr_msg.data = qr_codes[0];
                    qr_detect_publisher_->publish(qr_msg);
                    last_detected_qr_code_str = qr_codes[0];
                }
                else
                {
                    if (!qr_codes.empty())
                    {
                        RCLCPP_DEBUG(this->get_logger(), "QR code detected but not new: %s", qr_codes[0].c_str());
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "No QR code detected in current frame of %s camera", camera_name.c_str());
                        if (last_detected_qr_code_str != "")
                        {
                            reset_last_detected_qr_code_counter++;
                            if (reset_last_detected_qr_code_counter > 30)
                            {
                                last_detected_qr_code_str = "";
                                reset_last_detected_qr_code_counter = 0;
                            }
                        }
                    }
                }

                std_msgs::msg::Header header;
                header.stamp = this->now();
                auto image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
                qr_image_publisher_->publish(*image_msg);
                std::this_thread::sleep_for(16ms);
            }
            camera->close();
        }
        else
        {
            while (running_ && rclcpp::ok())
            {
                try
                {
                    if (!camera->read(image))
                    {
                        RCLCPP_WARN(this->get_logger(), "Failed to read image from %s camera", camera_name.c_str());
                        std::this_thread::sleep_for(16ms);
                        continue;
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Exception while reading image from %s camera: %s", camera_name.c_str(), e.what());
                    std::this_thread::sleep_for(16ms);
                    continue;
                }

                RCLCPP_DEBUG(this->get_logger(), "Image read successfully from %s camera", camera_name.c_str());

                bool is_detected = false;
                double x = 0, y = 0;
                if (image.empty())
                    continue;
                cv::Mat resultImg = perform_greenlight_detection(image, is_detected, x, y);
                if (is_detected)
                {
                    RCLCPP_INFO(this->get_logger(), "%s camera detected green light at (%.2f, %.2f)", camera_name.c_str(), x, y);
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "No green light detected in current frame of %s camera", camera_name.c_str());
                }

                auto message = dart_msgs::msg::GreenLight();
                message.header.stamp = this->get_clock()->now();
                message.header.frame_id = camera_name;
                message.is_detected = is_detected;
                message.location.x = x;
                message.location.y = y;
                message.location.z = 0.0;
                greenlight_publisher_->publish(message);

                auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                greenlight_image_publisher_->publish(*image_msg);

                std::this_thread::sleep_for(16ms);
            }
            camera->close();
        }
        RCLCPP_INFO(this->get_logger(), "Thread for %s camera stopped.", camera_name.c_str());
    }

    cv::Mat perform_greenlight_detection(cv::Mat &frame, bool &is_detected, double &x, double &y)
    {
        if (greenlight_detector_.detect(frame))
        {
            is_detected = true;
            cv::Point2f center;
            greenlight_detector_.getResult(center);
            x = center.x;
            y = center.y;
            RCLCPP_DEBUG(this->get_logger(), "Green light detected: center at (%.2f, %.2f)", x, y);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "No green light detected in the current frame");
        }
        return greenlight_detector_.drawRaw();
    }

    void setup_parameters()
    {
        // this->declare_parameter("lccv.enable", lccv_enabled_);
        // this->declare_parameter("lccv.image_width", 640);
        // this->declare_parameter("lccv.image_height", 480);
        // this->declare_parameter("dh.enable", dh_enabled_);
        // this->declare_parameter("dh.image_width", 1280);
        // this->declare_parameter("dh.image_height", 1024);
        // this->declare_parameter("lccv.camera_params", std::vector<std::string>());
        // this->declare_parameter("dh.camera_params", std::vector<std::string>());
        RCLCPP_INFO(this->get_logger(), "Camera parameters declared.");
    }

    void on_parameter_event(const rclcpp::Parameter &param)
    {
        if (param.get_name() == "lccv.enable")
        {
            lccv_enabled_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "LCCV camera enabled set to: %s", lccv_enabled_ ? "true" : "false");
        }
        else if (param.get_name() == "dh.enable")
        {
            dh_enabled_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "DH camera enabled set to: %s", dh_enabled_ ? "true" : "false");
        }
    }

public:
    explicit DartLauncherDetectorNode(rclcpp::NodeOptions options)
        : rclcpp_lifecycle::LifecycleNode("dart_launcher_detector", options),
          running_(false), lccv_enabled_(true), dh_enabled_(true)
    {
        // setup_parameters();
        callback_set_parameter_handle = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult
            {
                for (const auto &param : params)
                {
                    RCLCPP_INFO(this->get_logger(), "Parameter update: %s", param.get_name().c_str());
                    on_parameter_event(param);
                }
                // 如果处于激活状态，则重新配置节点
                if (this->get_current_state().label() == "active")
                {
                    RCLCPP_INFO(this->get_logger(), "Reconfiguring node due to parameter change...");
                    this->deactivate();
                    this->cleanup();
                    this->configure();
                    this->activate();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Node not active, reactivation skipped.");
                    this->cleanup();
                    this->configure();
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });
    }

    // 新增一个通用函数，用于加载相机参数并打开相机
    bool load_and_open_camera(const std::string &camera_prefix, std::shared_ptr<CameraDriver> &camera_driver)
    {
        RCLCPP_INFO(this->get_logger(), "Trying to open %s camera...", camera_prefix.c_str());

        // 获取相机参数
        std::unordered_map<std::string, std::string> camera_params;
        auto param_list = this->list_parameters({camera_prefix + ".camera_params"}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
        for (const auto &param : param_list.names)
        {
            // 截取参数名
            std::string param_name = param.substr(param.find_last_of(".") + 1);
            RCLCPP_INFO(this->get_logger(), "Parameter %s: %s", param_name.c_str(), this->get_parameter(param).value_to_string().c_str());

            if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE || this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                camera_params[param_name] = this->get_parameter(param).value_to_string();
            }
            else if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
            {
                auto param_value = this->get_parameter(param).as_double_array();
                std::string param_value_str;
                for (const auto &value : param_value)
                {
                    param_value_str += std::to_string(value) + " ";
                }
                camera_params[param_name] = param_value_str;
            }
            else if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
            {
                auto param_value = this->get_parameter(param).as_integer_array();
                std::string param_value_str;
                for (const auto &value : param_value)
                {
                    param_value_str += std::to_string(value) + " ";
                }
                camera_params[param_name] = param_value_str;
            }
            else if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                camera_params[param_name] = this->get_parameter(param).as_string();
            }
        }

        // 创建相机驱动实例并尝试打开
        if (camera_prefix == "lccv")
        {
            camera_driver = std::make_shared<CameraDriver_LCCV>();
        }
        else if (camera_prefix == "dh")
        {
            camera_driver = std::make_shared<CameraDriver_DH>();
        }

        if (!camera_driver->open(camera_params))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s camera with provided parameters.", camera_prefix.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "%s camera opened successfully.", camera_prefix.c_str());
        return true;
    }

    // 修改 on_configure 函数，调用通用函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &pre_state)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring node...");

        RCLCPP_INFO(this->get_logger(), "Loading parameters...");

        // 必需参数检查
        const std::string required_params[] = {
            "lccv.enable",
            "lccv.image_width",
            "lccv.image_height",
            "dh.enable",
            "dh.image_width",
            "dh.image_height",
        };

        for (const auto &param : required_params)
        {
            if (this->get_parameter(param).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
            {
                RCLCPP_ERROR(this->get_logger(), "Required parameter %s not set.", param.c_str());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
        }

        if (this->get_parameter("lccv.enable").get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        {
            lccv_enabled_ = this->get_parameter("lccv.enable").as_bool();
        }
        if (this->get_parameter("dh.enable").get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        {
            dh_enabled_ = this->get_parameter("dh.enable").as_bool();
        }

        greenlight_publisher_ = this->create_publisher<dart_msgs::msg::GreenLight>("greenlight_detection", 10);
        greenlight_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("greenlight_processed_image", 10);
        qr_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("qr_processed_image", 10);
        qr_detect_publisher_ = this->create_publisher<std_msgs::msg::String>("qr_detected_str", 10);

        // 使用通用函数加载和打开相机
        if (lccv_enabled_ && !load_and_open_camera("lccv", camera_lccv_))
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        if (dh_enabled_ && !load_and_open_camera("dh", camera_dh_))
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "Configuration complete: Cameras initialized.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(
        const rclcpp_lifecycle::State &pre_state)
    {
        RCLCPP_INFO(this->get_logger(), "Activating node...");
        running_ = true;
        if (lccv_enabled_)
        {
            lccv_thread_ = std::make_shared<std::thread>(std::bind(&DartLauncherDetectorNode::camera_thread, this, camera_lccv_, "lccv", true));
            lccv_thread_->detach();
            RCLCPP_INFO(this->get_logger(), "LCCV camera thread started and detached.");
        }
        if (dh_enabled_)
        {
            dh_thread_ = std::make_shared<std::thread>(std::bind(&DartLauncherDetectorNode::camera_thread, this, camera_dh_, "dh", false));
            dh_thread_->detach();
            RCLCPP_INFO(this->get_logger(), "DH camera thread started and detached.");
        }
        RCLCPP_INFO(this->get_logger(), "Node activation complete.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &pre_state)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating node: Stopping threads...");
        running_ = false;
        if (lccv_thread_ && lccv_thread_->joinable())
        {
            lccv_thread_->join();
            RCLCPP_INFO(this->get_logger(), "LCCV camera thread joined.");
        }
        if (dh_thread_ && dh_thread_->joinable())
        {
            dh_thread_->join();
            RCLCPP_INFO(this->get_logger(), "DH camera thread joined.");
        }
        RCLCPP_INFO(this->get_logger(), "Node deactivation complete.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &pre_state)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up resources...");
        camera_lccv_.reset();
        camera_dh_.reset();
        greenlight_publisher_.reset();
        qr_image_publisher_.reset();
        RCLCPP_INFO(this->get_logger(), "Resources successfully cleaned up.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().use_intra_process_comms(false);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<DartLauncherDetectorNode>(options);
    RCLCPP_INFO(node->get_logger(), "Node started. Spinning...");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}