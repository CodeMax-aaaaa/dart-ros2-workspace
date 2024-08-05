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
    if (node)
    {
        while (!(node->callback_set_parameter_handle))
        {
        } // 等待参数设置完成
        node->loadParametersfromGUI();
        if (node->get_parameter("auto_fw_calibration").as_bool())
        {
            node->calibration_fw();
            node->loadParametersfromGUI();
        }
    }

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
    if (node)
    {
        while (!(node->callback_set_parameter_handle))
        {
        } // 等待参数设置完成
        node->loadParametersfromGUI();
    }
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
    static bool operating = false;
    static std::thread thread;
    if (!operating)
    {
        operating = true;
        lv_label_set_text(guider_ui.Main_btn_hotspot_label, "连接热点...");
        // 开启线程处理
        thread = std::thread([]()
                             { system("nmcli device wifi rescan && nmcli device wifi connect 3SE-120"); 
                             operating = false; 
                             with_ui_lock(node, [&]() {
                                lv_label_set_text(guider_ui.Main_btn_hotspot_label, "连接热点");
                            }); });
        thread.detach();
    }
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

void btn_calibration_rpm_cb(lv_event_t *event)
{
    if (node)
    {
        node->calibration_fw();
        // node->set_parameter(rclcpp::Parameter("target_yaw_launch_angle_offset", node->target_yaw_launch_angle_offset));
        node->loadParametersfromGUI();
    }
}

void ddlist_dart_chosen_cb(lv_event_t *event)
{
    if (node)
    {
        while (!(node->callback_set_parameter_handle))
        {
        } // 等待参数设置完成
        node->loadParametersfromGUI();
        // 自动校准
        if (node->get_parameter("auto_fw_calibration").as_bool())
        {
            node->calibration_fw();
            node->loadParametersfromGUI();
            // node->set_parameter(rclcpp::Parameter("target_yaw_launch_angle_offset", node->target_yaw_launch_angle_offset));
        }
    }
}

void loadDartInfo(std::string dart_name)
{
    // 名称，yaw offset，调试，图像
    lv_textarea_set_text(guider_ui.Main_ta_dart_param_number, dart_name.c_str());
    lv_spinbox_set_value(guider_ui.Main_spinbox_dart_param_yaw_offset, node->dart_db_->getYawOffset(dart_name));

    // 绘制chart
    // auto dart_data = node->dart_db_->getDartData(dart_name); // 是排序好的，横轴distance，纵轴velocity

    // lv_chart_set_axis_tick(guider_ui.Main_chart_velocity_distance_curve, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 4, 4, true, 40);
    // lv_chart_set_axis_tick(guider_ui.Main_chart_velocity_distance_curve, LV_CHART_AXIS_PRIMARY_X, 10, 1, 10, 10, true, 40);

    // lv_chart_set_type(guider_ui.Main_chart_velocity_distance_curve, LV_CHART_TYPE_LINE);
    // lv_chart_set_point_count(guider_ui.Main_chart_velocity_distance_curve, dart_data.distance_velocity_points.size());
    // lv_chart_set_range(guider_ui.Main_chart_velocity_distance_curve, LV_CHART_AXIS_PRIMARY_Y, 16.0, 18.5);
    // lv_chart_set_range(guider_ui.Main_chart_velocity_distance_curve, LV_CHART_AXIS_PRIMARY_X, dart_data.distance_velocity_points.begin()->distance, dart_data.distance_velocity_points.end()->distance);

    // // zoom
    // lv_chart_set_zoom_x(guider_ui.Main_chart_velocity_distance_curve, 256); // no zoom

    // lv_chart_series_t *ser = lv_chart_add_series(guider_ui.Main_chart_velocity_distance_curve, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    // for (auto &point : dart_data.distance_velocity_points)
    // {
    //     lv_chart_set_next_value(guider_ui.Main_chart_velocity_distance_curve, ser, point.velocity);
    // }

    // lv_chart_set_axis_tick(guider_ui.Main_chart_vel_fw_vel_curve, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 4, 4, true, 40);
    // lv_chart_set_axis_tick(guider_ui.Main_chart_vel_fw_vel_curve, LV_CHART_AXIS_PRIMARY_X, 10, 1, 10, 10, true, 40);

    // lv_chart_set_type(guider_ui.Main_chart_vel_fw_vel_curve, LV_CHART_TYPE_LINE);
    // lv_chart_set_point_count(guider_ui.Main_chart_vel_fw_vel_curve, dart_data.fw_velocity_points.size());

    // lv_chart_set_range(guider_ui.Main_chart_vel_fw_vel_curve, LV_CHART_AXIS_PRIMARY_Y, 16.0, 18.5);
    // lv_chart_set_range(guider_ui.Main_chart_vel_fw_vel_curve, LV_CHART_AXIS_PRIMARY_X, dart_data.fw_velocity_points.begin()->fw_velocity, dart_data.fw_velocity_points.end()->fw_velocity);

    // // zoom
    // lv_chart_set_zoom_x(guider_ui.Main_chart_vel_fw_vel_curve, 256); // no zoom

    // lv_chart_series_t *ser2 = lv_chart_add_series(guider_ui.Main_chart_vel_fw_vel_curve, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    // for (auto &point : dart_data.fw_velocity_points)
    // {
    //     lv_chart_set_next_value(guider_ui.Main_chart_vel_fw_vel_curve, ser2, point.velocity);
    // }
    // // 更新
    // lv_chart_refresh(guider_ui.Main_chart_vel_fw_vel_curve);

    // 速度调试重绘
    ta_debug_velocity_cb(NULL);
}

void saveDartInfo(std::string dart_name)
{
    // 名称，yaw offset，调试，图像
    node->dart_db_->setDartName(dart_name, std::string(lv_textarea_get_text(guider_ui.Main_ta_dart_param_number)));
    node->dart_db_->setYawOffset(dart_name, lv_spinbox_get_value(guider_ui.Main_spinbox_dart_param_yaw_offset));
    node->dart_db_->saveToFile(std::string(YAML_PATH) + std::string("dart_db.yaml"));
}

lv_obj_t *obj_dart_list_seleted = NULL;

void btn_reload_dart_cb(lv_event_t *event)
{
    if (node)
    {
        if (obj_dart_list_seleted)
        {
            RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), "Dart %s is chosen", lv_list_get_btn_text(guider_ui.Main_list_darts, obj_dart_list_seleted));
            loadDartInfo(std::string(lv_list_get_btn_text(guider_ui.Main_list_darts, obj_dart_list_seleted)));
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("lvgl_ui"), "No dart is chosen");
        }
    }
}

void btn_save_dart_cb(lv_event_t *event)
{
    if (node)
    {
        if (obj_dart_list_seleted)
        {
            RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), "Dart %s is chosen", lv_list_get_btn_text(guider_ui.Main_list_darts, obj_dart_list_seleted));
            saveDartInfo(std::string(lv_list_get_btn_text(guider_ui.Main_list_darts, obj_dart_list_seleted)));
        }
    }
}

void dart_list_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    obj_dart_list_seleted = lv_event_get_target(e);
    if (node && code == LV_EVENT_CLICKED)
    {
        RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), "Dart %s is chosen", lv_list_get_btn_text(guider_ui.Main_list_darts, obj_dart_list_seleted));
        loadDartInfo(std::string(lv_list_get_btn_text(guider_ui.Main_list_darts, obj_dart_list_seleted)));
    }
}

// void chart_event_cb(lv_event_t *e)
// {
//     lv_event_code_t code = lv_event_get_code(e);
//     lv_obj_t *obj = lv_event_get_target(e);
//     if (node && code == LV_EVENT_CLICKED)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), "Chart is clicked");
//     }
// }

void ta_debug_velocity_cb(lv_event_t *e)
{
    if (node && obj_dart_list_seleted != NULL)
    {
        RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), "Text area is changed");
        // 检查是否为数字
        double requested_velocity = 0;
        try
        {
            requested_velocity = std::stod(std::string(lv_textarea_get_text(guider_ui.Main_ta_target_initial_speed_debug)));
        }
        catch (...)
        {
            requested_velocity = 0;
        }
        // 给出计算结果
        if (requested_velocity == 0)
            lv_label_set_text(guider_ui.Main_label_target_fw_velocity_result, "N/A");
        else
        {
            double result = node->dart_db_->calculateFWVelocity(std::string(lv_list_get_btn_text(guider_ui.Main_list_darts, obj_dart_list_seleted)), requested_velocity);
            lv_label_set_text(guider_ui.Main_label_target_fw_velocity_result, std::to_string(result).c_str());
            RCLCPP_INFO(rclcpp::get_logger("lvgl_ui"), "Requested velocity: %f, Result: %f", requested_velocity, result);
        }
    }
    else
    {
        lv_label_set_text(guider_ui.Main_label_target_fw_velocity_result, "N/A");
    }
}

void spinbox_fw_calibration_cb(lv_event_t *e)
{
    // 防止回调死循环
    if (callback_spinbox_disabled)
        return;

    callback_spinbox_disabled = true;
    if (node)
    {
        while (!(node->callback_set_parameter_handle))
        {
        } // 等待参数设置完成
        // 自动校准
        if (node->get_parameter("auto_fw_calibration").as_bool())
        {
            node->calibration_fw(false);
            // node->set_parameter(rclcpp::Parameter("target_yaw_launch_angle_offset", node->target_yaw_launch_angle_offset));
        }
        node->loadParametersfromGUI();
    }
    callback_spinbox_disabled = false;
}

void custom_init(lv_ui *ui)
{
    /* Add your codes here */
    lv_textarea_set_accepted_chars(ui->Main_ta_dart_param_number, "1234567890-_.ABCDEFGHIJKLMNOPQRSTtUVWXYZ");
    lv_spinbox_set_range(ui->Main_spinbox_fw_speed, 0, 7070);
    lv_spinbox_set_range(ui->Main_spinbox_fw_speed_offset, -2000, 2000);
    lv_spinbox_set_range(ui->Main_spinbox_fw_speed_ratio, 0, 3000);
    lv_spinbox_set_range(ui->Main_spinbox_yaw_angle, 5000, 360000);
    lv_spinbox_set_range(ui->Main_spinbox_yaw_offset, -50000, 50000);
    lv_spinbox_set_range(ui->Main_spinbox_yaw_angle_cv, 5000, 360000);
    lv_spinbox_set_range(ui->Main_spinbox_yaw_angle_offset_cv, -50000, 50000);
    lv_spinbox_set_range(ui->Main_spinbox_yaw_calibration_x, 0, 102400);
    lv_spinbox_set_range(ui->Main_spinbox_slot1_fw_offset, -7070, 7070);
    lv_spinbox_set_range(ui->Main_spinbox_slot2_fw_offset, -7070, 7070);
    lv_spinbox_set_range(ui->Main_spinbox_slot3_fw_offset, -7070, 7070);
    lv_spinbox_set_range(ui->Main_spinbox_slot4_fw_offset, -7070, 7070);
    lv_spinbox_set_range(ui->Main_spinbox_distance_X, 0, 4000);
    lv_spinbox_set_range(ui->Main_spinbox_target_delta_height, -500, 500);

	lv_label_set_text(ui->Init_label_2, "镖神保佑\n把把必中");

    // 创建并设置样式：选中项的样式
    static lv_style_t style_ddlist_selected_option;
    lv_style_init(&style_ddlist_selected_option);
    lv_style_set_bg_color(&style_ddlist_selected_option, lv_color_black());
    lv_style_set_text_color(&style_ddlist_selected_option, lv_color_white());
    lv_style_set_bg_opa(&style_ddlist_selected_option, LV_OPA_COVER);

    lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_1), &style_ddlist_selected_option, LV_PART_SELECTED);

    lv_obj_clear_flag(guider_ui.Main, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(guider_ui.Main_msgbox, LV_OBJ_FLAG_SCROLLABLE);
    // MSG Box label 可滚动
    lv_label_set_long_mode(lv_msgbox_get_text(guider_ui.Main_msgbox), LV_LABEL_LONG_SCROLL);

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
    lv_obj_add_event_cb(guider_ui.Main_spinbox_yaw_calibration_x, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_slot1_fw_offset, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_slot2_fw_offset, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_slot3_fw_offset, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_slot4_fw_offset, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_distance_X, spinbox_fw_calibration_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_spinbox_target_delta_height, spinbox_fw_calibration_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_add_event_cb(guider_ui.Main_sw_auto_rpm_calibration, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_sw_auto_yaw_calibration, spinbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_sw_auto_yaw_calibration_cv, spinbox_event_cb_cv, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_add_event_cb(guider_ui.Main_btn_reload_params, btn_reload_params_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_yaw_calibration, btn_yaw_calibration_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_yaw_calibration_cv, btn_yaw_calibration_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_restart, btn_restart_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_shutdown, btn_shutdown_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_hotspot, btn_hotspot_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_gnome, btn_gnome_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_rpm_calibration, btn_calibration_rpm_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_dart_param_save, btn_save_dart_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_btn_dart_param_reload, btn_reload_dart_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(guider_ui.Main_ddlist_launch_dart_1, ddlist_dart_chosen_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_ddlist_launch_dart_2, ddlist_dart_chosen_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_ddlist_launch_dart_3, ddlist_dart_chosen_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(guider_ui.Main_ddlist_launch_dart_4, ddlist_dart_chosen_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_add_event_cb(guider_ui.Main_ta_target_initial_speed_debug, ta_debug_velocity_cb, LV_EVENT_VALUE_CHANGED, NULL);
}