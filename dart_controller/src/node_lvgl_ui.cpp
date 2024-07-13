#include <rclcpp/rclcpp.hpp>

#include "lvgl/lvgl.h"
#include "lv_drivers/display/fbdev.h"
#include "lv_drivers/indev/evdev.h"
#include "gui_guider.h"
#include "events_init.h"
#include "custom.h"

#include <unistd.h>
#include <thread>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>

#include "get_ip.hpp"

#define DISP_BUF_SIZE (600 * 1024)

#include <chrono>
using namespace std::chrono_literals;
lv_ui guider_ui;

/*Set in lv_conf.h as `LV_TICK_CUSTOM_SYS_TIME_EXPR`*/
// uint32_t custom_tick_get()
// {
//   static uint64_t start_ms = 0;
//   if (start_ms == 0)
//   {
//     struct timeval tv_start;
//     gettimeofday(&tv_start, NULL);
//     start_ms = (tv_start.tv_sec * 1000000 + tv_start.tv_usec) / 1000;
//   }

//   struct timeval tv_now;
//   gettimeofday(&tv_now, NULL);
//   uint64_t now_ms;
//   now_ms = (tv_now.tv_sec * 1000000 + tv_now.tv_usec) / 1000;

//   uint32_t time_ms = now_ms - start_ms;
//   return time_ms;
// }

// custom_tick_get 使用chrono实现，精度到ms
uint32_t custom_tick_get()
{
  static auto start = std::chrono::high_resolution_clock::now();
  auto now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

// ROS 2 Interface
class NodeLVGLUI : public rclcpp::Node
{
public:
  NodeLVGLUI() : Node("lvgl_ui")
  {
    lv_log_register_print_cb(print_cb);

    lv_init();

    /*Linux frame buffer device init*/
    fbdev_init();

    /*A small buffer for LittlevGL to draw the screen's content*/
    static lv_color_t buf[2][DISP_BUF_SIZE];

    /*Initialize a descriptor for the buffer*/
    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, buf[0], buf[1], DISP_BUF_SIZE);

    /*Initialize and register a display driver*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf = &disp_buf;
    disp_drv.flush_cb = fbdev_flush;
    disp_drv.hor_res = 1024;
    disp_drv.ver_res = 600;
    lv_disp_drv_register(&disp_drv);

    evdev_init();

    static lv_indev_drv_t indev_drv_1;
    lv_indev_drv_init(&indev_drv_1); /*Basic initialization*/
    indev_drv_1.type = LV_INDEV_TYPE_POINTER;

    /*This function will be called periodically (by the library) to get the mouse position and state*/
    indev_drv_1.read_cb = evdev_read;
    lv_indev_drv_register(&indev_drv_1);

    /*Create a Demo*/
    setup_ui(&guider_ui);
    events_init(&guider_ui);
    custom_init(&guider_ui);

    informational_update();

    RCLCPP_INFO(this->get_logger(), "UI thread started");
    /*Handle LVGL tasks*/
    auto timer_callback_ui =
        [this]() -> void
    {
      lv_timer_handler();
    };

    auto timer_callback_informational_update =
        [this]() -> void
    {
      informational_update();
    };
    timer_[0] = this->create_wall_timer(5ms, timer_callback_ui);
    timer_[1] = this->create_wall_timer(1s, timer_callback_informational_update);
  }

  static void print_cb(const char *buf);

private:
  rclcpp::TimerBase::SharedPtr timer_[2];

  void informational_update();
};

std::shared_ptr<NodeLVGLUI> node;

void NodeLVGLUI::informational_update()
{
  // 获取IP地址
  std::string interface = "wlan0";
  std::string ip = getIPAddress(interface);

  RCLCPP_INFO_ONCE(this->get_logger(), "IP address: %s", ip.c_str());
  lv_label_set_text(guider_ui.Main_label_ip, ip.c_str());
}

void NodeLVGLUI::print_cb(const char *buf)
{
  // {"Info", "Warn", "Error", "User"}
  if (buf[0] == '[' && buf[1] == 'I' && buf[2] == 'n' && buf[3] == 'f' && buf[4] == 'o')
    RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), buf);
  else if (buf[0] == '[' && buf[1] == 'U' && buf[2] == 's' && buf[3] == 'e' && buf[4] == 'r')
    RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), buf);
  else if (buf[0] == '[' && buf[1] == 'E' && buf[2] == 'r' && buf[3] == 'r' && buf[4] == 'o' && buf[5] == 'r')
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("lvgl_ui"), *node->get_clock(), 1000, buf);
  else if (buf[0] == '[' && buf[1] == 'W' && buf[2] == 'a' && buf[3] == 'r' && buf[4] == 'n')
    RCLCPP_WARN(rclcpp::get_logger("lvgl_ui"), buf);
}

int main(void)
{
  // ROS 2 Interface begin
  rclcpp::init(0, nullptr);
  node = std::make_shared<NodeLVGLUI>();
  rclcpp::spin(node);
  RCLCPP_INFO(node->get_logger(), "UI thread stopped");
  lv_obj_clean(lv_scr_act()); // Clean up the active screen
  lv_timer_handler();         // Call the timer handler
  lv_deinit();                // Deinitialize LittlevGL
  rclcpp::shutdown();
  return true;
}