#ifndef NODE_LVGL_UI_HPP
#define NODE_LVGL_UI_HPP

#include <rclcpp/rclcpp.hpp>

#include "lvgl/lvgl.h"
#include "lv_drivers/display/fbdev.h"
#include "lv_drivers/indev/evdev.h"
#include <info/msg/dart_launcher_status.hpp>
#include <info/msg/dart_param.hpp>
#include <info/msg/green_light.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_guider.h"
#include "events_init.h"
#include "dart_config.hpp"

#include <unistd.h>
#include <thread>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <string>
#include "get_ip.hpp"

#define DISP_BUF_SIZE (600 * 1024)

// ROS 2 Interface
class NodeLVGLUI : public rclcpp::Node
{
public:
    bool callback_disabled = false;
    NodeLVGLUI();
    void loadParametersfromGUI();
    void calibration_yaw();

private:
    std::mutex mutex_ui_;
    rclcpp::TimerBase::SharedPtr timer_[2];
    rclcpp::Publisher<info::msg::DartParam>::SharedPtr dart_launcher_cmd_pub_;
    rclcpp::Subscription<info::msg::DartLauncherStatus>::SharedPtr dart_launcher_status_sub_;
    rclcpp::Subscription<info::msg::DartParam>::SharedPtr dart_launcher_present_param_sub_;
    rclcpp::Subscription<info::msg::GreenLight>::SharedPtr green_light_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cv_image_sub_;

    std::shared_ptr<info::msg::DartLauncherStatus> dart_launcher_status_;
    std::shared_ptr<info::msg::GreenLight> green_light_;

    void update_dart_launcher_status_callback(info::msg::DartLauncherStatus::SharedPtr msg);
    void update_dart_launcher_present_param_callback(info::msg::DartParam::SharedPtr msg);
    void update_green_light_callback(info::msg::GreenLight::SharedPtr msg);
    void update_ip_address();
    void update_parameters_to_gui();
    rcl_interfaces::msg::SetParametersResult set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
    void update_cv_image(sensor_msgs::msg::Image::SharedPtr msg);
    lv_color_t *img_buf;
    template <typename Func>
    friend void with_ui_lock(std::shared_ptr<NodeLVGLUI> node, Func func);
};

template <typename Func>
void with_ui_lock(std::shared_ptr<NodeLVGLUI> node, Func func)
{
    std::lock_guard<std::mutex> lock(node->mutex_ui_);
    func(); // 调用传递的lambda表达式
}

extern std::shared_ptr<NodeLVGLUI> node;

#endif