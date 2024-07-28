// SPDX-License-Identifier: MIT
// Copyright 2020 NXP

/**
 * @file custom.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>
#include "lvgl.h"
#include "custom.h"
#include <node_lvgl_ui.hpp>

/**********************
 *      TYPEDEFS
 **********************/
// msg_emerg_t
typedef enum
{
    MSG_EMERG_INFO,
    MSG_EMERG_WARN,
    MSG_EMERG_ERROR,
    MSG_EMERG_CRITICAL
} msg_emerg_t;

/*********************
 *      DEFINES
 *********************/
lv_anim_ready_cb_t ready_cb(lv_anim_t *a);
lv_anim_ready_cb_t dequeue_ready_cb(lv_anim_t *a);

// /**********************
//  *  STATIC PROTOTYPES
//  **********************/

// /**********************
//  *  STATIC VARIABLES
//  **********************/

void print_cb(const char *buf)
{
    // {"Info", "Warn", "Error", "User"}
    if (buf[0] == '[' && buf[1] == 'I' && buf[2] == 'n' && buf[3] == 'f' && buf[4] == 'o')
        RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), buf);
    else if (buf[0] == '[' && buf[1] == 'U' && buf[2] == 's' && buf[3] == 'e' && buf[4] == 'r')
        RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), buf);
    else if (buf[0] == '[' && buf[1] == 'E' && buf[2] == 'r' && buf[3] == 'r' && buf[4] == 'o' && buf[5] == 'r')
        RCLCPP_ERROR(rclcpp::get_logger("lvgl_ui"), buf);
    else if (buf[0] == '[' && buf[1] == 'W' && buf[2] == 'a' && buf[3] == 'r' && buf[4] == 'n')
        RCLCPP_WARN(rclcpp::get_logger("lvgl_ui"), buf);
    else
        RCLCPP_DEBUG(rclcpp::get_logger("lvgl_ui"), buf);
}

bool callback_spinbox_disabled = false;

void spinbox_event_cb(lv_event_t *event)
{
    // 防止回调死循环
    if (callback_spinbox_disabled)
        return;

    callback_spinbox_disabled = true;
    // 将非cv参数拷到cv参数中
    // 删除回调
    lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_angle_cv, lv_spinbox_get_value(guider_ui.Main_spinbox_yaw_angle));
    lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_angle_offset_cv, lv_spinbox_get_value(guider_ui.Main_spinbox_yaw_offset));

    // 按钮
    if (lv_obj_has_state(guider_ui.Main_sw_auto_yaw_calibration, LV_STATE_CHECKED))
        lv_obj_add_state(guider_ui.Main_sw_auto_yaw_calibration_cv, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(guider_ui.Main_sw_auto_yaw_calibration_cv, LV_STATE_CHECKED);
    // 调用node->spinbox_event_cb(obj, event);
    if (node && !(node->callback_disabled))
        node->loadParametersfromGUI();

    callback_spinbox_disabled = false;
}

void spinbox_event_cb_cv(lv_event_t *event)
{
    // 防止回调死循环
    if (callback_spinbox_disabled)
        return;

    callback_spinbox_disabled = true;
    // 将cv参数拷到非cv参数中
    lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_angle, lv_spinbox_get_value(guider_ui.Main_spinbox_yaw_angle_cv));
    lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_offset, lv_spinbox_get_value(guider_ui.Main_spinbox_yaw_angle_offset_cv));

    // 按钮
    if (lv_obj_has_state(guider_ui.Main_sw_auto_yaw_calibration_cv, LV_STATE_CHECKED))
        lv_obj_add_state(guider_ui.Main_sw_auto_yaw_calibration, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(guider_ui.Main_sw_auto_yaw_calibration, LV_STATE_CHECKED);

    // 调用node->spinbox_event_cb(obj, event);
    if (node && !(node->callback_disabled))
        node->loadParametersfromGUI();

    callback_spinbox_disabled = false;
}

void btn_reload_params_cb(lv_event_t *event)
{
    static bool operating = false;
    if (node && !(operating))
    {
        operating = true;
        std::thread([&]()
                    {
      auto client = node->create_client<std_srvs::srv::Empty>("/dart_config/reset_all_param_to_default");
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto result = client->async_send_request(request);
      with_ui_lock(node, [&]() {
        lv_label_set_text(guider_ui.Main_btn_reload_params_label, "恢复...");
      });
      // 在future中等待结果
      result.wait_until(std::chrono::steady_clock::now() + std::chrono::seconds(2));
      with_ui_lock(node, [&]() {
        lv_label_set_text(guider_ui.Main_btn_reload_params_label, "恢复默认");
       });
      operating = false; })
            .detach();
    }
}

void btn_yaw_calibration_cb(lv_event_t *event)
{
    if (node)
        node->calibration_yaw();
}

void set_switch_state(lv_obj_t *sw, bool state)
{
    if (state)
    {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    else
    {
        lv_obj_clear_state(sw, LV_STATE_CHECKED);
    }
}

void btn_restart_cb(lv_event_t *event)
{
    static bool operating = false;
    if (operating)
        return;

    operating = true;
    // 调用system命令重启飞镖服务
    system("sudo systemctl restart dart_ros2_run.service");
}

void btn_shutdown_cb(lv_event_t *event)
{

    // 调用system命令关机
    static bool operating = false;
    if (!operating)
    {
        operating = true;
        lv_label_set_text(guider_ui.Main_btn_shutdown_label, "关机...");
        system("sudo poweroff");
        operating = false;
    }
}

void btn_hotspot_cb(lv_event_t *event)
{
    // 调用system命令开启热点
    system("nmcli device wifi rescan && nmcli device wifi connect 3SE-120");
}

void btn_gnome_cb(lv_event_t *event)
{
    static bool operating = false;
    if (!operating)
    {
        operating = true;
        lv_label_set_text(guider_ui.Main_btn_gnome_label, "Gnome...");
        // 调用system命令开启gnome
        system("sudo systemctl isolate graphical.target");
        system("sudo systemctl stop dart_ros2_run.service");
        operating = false;
    }
}

void custom_init(lv_ui *ui)
{
    /* Add your codes here */
    lv_obj_clear_flag(guider_ui.Main, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(guider_ui.Main_msgbox, LV_OBJ_FLAG_SCROLLABLE);

    lv_label_set_text(guider_ui.Main_label_23, "不适用");
    lv_meter_set_indicator_value(guider_ui.Main_meter_fw_speed_2, guider_ui.Main_meter_fw_speed_2_scale_0_ndline_0, 0);
    lv_label_set_text(ui->Main_label_fw_speed_2, "0");
    lv_meter_set_indicator_value(ui->Main_meter_fw_speed_1, ui->Main_meter_fw_speed_1_scale_0_ndline_0, 0);
    lv_label_set_text(ui->Main_label_fw_speed_1, "0");

    // Event Init
    lv_obj_add_event_cb(guider_ui.Main_spinbox_fw_speed, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_fw_speed_offset, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_fw_speed_ratio, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_yaw_angle, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_yaw_offset, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_yaw_angle_cv, spinbox_event_cb_cv, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_yaw_angle_offset_cv, spinbox_event_cb_cv, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_sw_auto_rpm_calibration, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_sw_auto_yaw_calibration, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_sw_auto_yaw_calibration_cv, spinbox_event_cb_cv, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_yaw_calibration_x, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_reload_params, btn_reload_params_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_yaw_calibration, btn_yaw_calibration_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_yaw_calibration_cv, btn_yaw_calibration_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_restart, btn_restart_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_shutdown, btn_shutdown_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_hotspot, btn_hotspot_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_gnome, btn_gnome_cb, LV_EVENT_CLICKED, NULL);
}