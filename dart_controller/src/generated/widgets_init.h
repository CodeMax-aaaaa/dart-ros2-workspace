/*
* Copyright 2024 NXP
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#ifndef WIDGET_INIT_H
#define WIDGET_INIT_H
#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "gui_guider.h"

__attribute__((unused)) void kb_event_cb(lv_event_t *e);
__attribute__((unused)) void ta_event_cb(lv_event_t *e);
#if LV_USE_ANALOGCLOCK != 0
void clock_count(int *hour, int *min, int *sec);
#endif

extern lv_obj_t * Main_spinbox_yaw_angle;
void lv_Main_spinbox_yaw_angle_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_yaw_angle_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_fw_speed;
void lv_Main_spinbox_fw_speed_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_fw_speed_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_yaw_offset;
void lv_Main_spinbox_yaw_offset_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_yaw_offset_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_fw_speed_offset;
void lv_Main_spinbox_fw_speed_offset_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_fw_speed_offset_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_fw_speed_ratio;
void lv_Main_spinbox_fw_speed_ratio_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_fw_speed_ratio_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_distance_X;
void lv_Main_spinbox_distance_X_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_distance_X_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_slot4_fw_offset;
void lv_Main_spinbox_slot4_fw_offset_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_slot4_fw_offset_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_slot3_fw_offset;
void lv_Main_spinbox_slot3_fw_offset_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_slot3_fw_offset_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_slot2_fw_offset;
void lv_Main_spinbox_slot2_fw_offset_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_slot2_fw_offset_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_slot1_fw_offset;
void lv_Main_spinbox_slot1_fw_offset_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_slot1_fw_offset_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_target_delta_height;
void lv_Main_spinbox_target_delta_height_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_target_delta_height_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_dart_param_yaw_offset;
void lv_Main_spinbox_dart_param_yaw_offset_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_dart_param_yaw_offset_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_yaw_angle_offset_cv;
void lv_Main_spinbox_yaw_angle_offset_cv_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_yaw_angle_offset_cv_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_yaw_angle_cv;
void lv_Main_spinbox_yaw_angle_cv_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_yaw_angle_cv_decrement_event_cb(lv_event_t * event);
extern lv_obj_t * Main_spinbox_yaw_calibration_x;
void lv_Main_spinbox_yaw_calibration_x_increment_event_cb(lv_event_t * event);
void lv_Main_spinbox_yaw_calibration_x_decrement_event_cb(lv_event_t * event);


#ifdef __cplusplus
}
#endif
#endif
