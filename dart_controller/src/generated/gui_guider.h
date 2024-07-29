/*
* Copyright 2024 NXP
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#ifndef GUI_GUIDER_H
#define GUI_GUIDER_H
#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

typedef struct
{
  
	lv_obj_t *Main;
	bool Main_del;
	lv_obj_t *g_kb_Main;
	lv_obj_t *Main_MainView;
	lv_obj_t *Main_MainView_tab_1;
	lv_obj_t *Main_MainView_tab_2;
	lv_obj_t *Main_MainView_tab_3;
	lv_obj_t *Main_label_2;
	lv_obj_t *Main_label_8;
	lv_obj_t *Main_label_12;
	lv_obj_t *Main_spinbox_yaw_angle;
	lv_obj_t *Main_spinbox_yaw_angle_btn;
	lv_obj_t *Main_spinbox_yaw_angle_btn_minus;
	lv_obj_t *Main_spinbox_fw_speed;
	lv_obj_t *Main_spinbox_fw_speed_btn;
	lv_obj_t *Main_spinbox_fw_speed_btn_minus;
	lv_obj_t *Main_spinbox_yaw_offset;
	lv_obj_t *Main_spinbox_yaw_offset_btn;
	lv_obj_t *Main_spinbox_yaw_offset_btn_minus;
	lv_obj_t *Main_label_13;
	lv_obj_t *Main_btn_yaw_calibration;
	lv_obj_t *Main_btn_yaw_calibration_label;
	lv_obj_t *Main_label_14;
	lv_obj_t *Main_spinbox_fw_speed_offset;
	lv_obj_t *Main_spinbox_fw_speed_offset_btn;
	lv_obj_t *Main_spinbox_fw_speed_offset_btn_minus;
	lv_obj_t *Main_label_15;
	lv_obj_t *Main_label_16;
	lv_obj_t *Main_spinbox_fw_speed_ratio;
	lv_obj_t *Main_spinbox_fw_speed_ratio_btn;
	lv_obj_t *Main_spinbox_fw_speed_ratio_btn_minus;
	lv_obj_t *Main_meter_fw_speed_1;
	lv_meter_indicator_t *Main_meter_fw_speed_1_scale_0_ndline_0;
	lv_obj_t *Main_meter_fw_speed_2;
	lv_meter_indicator_t *Main_meter_fw_speed_2_scale_0_ndline_0;
	lv_obj_t *Main_label_17;
	lv_obj_t *Main_label_18;
	lv_obj_t *Main_label_fw_speed_1;
	lv_obj_t *Main_label_fw_speed_2;
	lv_obj_t *Main_bar_yaw_angle;
	lv_obj_t *Main_label_21;
	lv_obj_t *Main_label_yaw_angle;
	lv_obj_t *Main_bar_launch_progress;
	lv_obj_t *Main_label_launch_progress;
	lv_obj_t *Main_label_23;
	lv_obj_t *Main_label_25;
	lv_obj_t *Main_label_yaw_location;
	lv_obj_t *Main_label_27;
	lv_obj_t *Main_btn_rpm_calibration;
	lv_obj_t *Main_btn_rpm_calibration_label;
	lv_obj_t *Main_label_29;
	lv_obj_t *Main_sw_auto_rpm_calibration;
	lv_obj_t *Main_label_30;
	lv_obj_t *Main_ddlist_launch_dart_1;
	lv_obj_t *Main_label_34;
	lv_obj_t *Main_ddlist_launch_dart_3;
	lv_obj_t *Main_label_33;
	lv_obj_t *Main_label_32;
	lv_obj_t *Main_ddlist_launch_dart_4;
	lv_obj_t *Main_ddlist_launch_dart_2;
	lv_obj_t *Main_spinbox_distance_X;
	lv_obj_t *Main_spinbox_distance_X_btn;
	lv_obj_t *Main_spinbox_distance_X_btn_minus;
	lv_obj_t *Main_sw_auto_yaw_calibration;
	lv_obj_t *Main_btn_reload_params;
	lv_obj_t *Main_btn_reload_params_label;
	lv_obj_t *Main_spinbox_slot4_fw_offset;
	lv_obj_t *Main_spinbox_slot4_fw_offset_btn;
	lv_obj_t *Main_spinbox_slot4_fw_offset_btn_minus;
	lv_obj_t *Main_label_55;
	lv_obj_t *Main_spinbox_slot3_fw_offset;
	lv_obj_t *Main_spinbox_slot3_fw_offset_btn;
	lv_obj_t *Main_spinbox_slot3_fw_offset_btn_minus;
	lv_obj_t *Main_label_54;
	lv_obj_t *Main_spinbox_slot2_fw_offset;
	lv_obj_t *Main_spinbox_slot2_fw_offset_btn;
	lv_obj_t *Main_spinbox_slot2_fw_offset_btn_minus;
	lv_obj_t *Main_label_53;
	lv_obj_t *Main_spinbox_slot1_fw_offset;
	lv_obj_t *Main_spinbox_slot1_fw_offset_btn;
	lv_obj_t *Main_spinbox_slot1_fw_offset_btn_minus;
	lv_obj_t *Main_label_52;
	lv_obj_t *Main_btn_restart;
	lv_obj_t *Main_btn_restart_label;
	lv_obj_t *Main_btn_shutdown;
	lv_obj_t *Main_btn_shutdown_label;
	lv_obj_t *Main_label_56;
	lv_obj_t *Main_btn_hotspot;
	lv_obj_t *Main_btn_hotspot_label;
	lv_obj_t *Main_btn_gnome;
	lv_obj_t *Main_btn_gnome_label;
	lv_obj_t *Main_spinbox_target_delta_height;
	lv_obj_t *Main_spinbox_target_delta_height_btn;
	lv_obj_t *Main_spinbox_target_delta_height_btn_minus;
	lv_obj_t *Main_label_57;
	lv_obj_t *Main_list_darts;
	lv_obj_t *Main_label_35;
	lv_obj_t *Main_cont_dart_param;
	lv_obj_t *Main_label_36;
	lv_obj_t *Main_ta_dart_param_number;
	lv_obj_t *Main_spinbox_dart_param_yaw_offset;
	lv_obj_t *Main_spinbox_dart_param_yaw_offset_btn;
	lv_obj_t *Main_spinbox_dart_param_yaw_offset_btn_minus;
	lv_obj_t *Main_label_39;
	lv_obj_t *Main_btn_dart_param_save;
	lv_obj_t *Main_btn_dart_param_save_label;
	lv_obj_t *Main_btn_dart_param_reload;
	lv_obj_t *Main_btn_dart_param_reload_label;
	lv_obj_t *Main_label_50;
	lv_obj_t *Main_ta_target_initial_speed_debug;
	lv_obj_t *Main_label_51;
	lv_obj_t *Main_label_target_fw_velocity_result;
	lv_obj_t *Main_label_40;
	lv_obj_t *Main_canvas_opencv;
	lv_obj_t *Main_label_43;
	lv_obj_t *Main_label_42;
	lv_obj_t *Main_spinbox_yaw_angle_offset_cv;
	lv_obj_t *Main_spinbox_yaw_angle_offset_cv_btn;
	lv_obj_t *Main_spinbox_yaw_angle_offset_cv_btn_minus;
	lv_obj_t *Main_spinbox_yaw_angle_cv;
	lv_obj_t *Main_spinbox_yaw_angle_cv_btn;
	lv_obj_t *Main_spinbox_yaw_angle_cv_btn_minus;
	lv_obj_t *Main_label_44;
	lv_obj_t *Main_label_yaw_location_cv;
	lv_obj_t *Main_spinbox_yaw_calibration_x;
	lv_obj_t *Main_spinbox_yaw_calibration_x_btn;
	lv_obj_t *Main_spinbox_yaw_calibration_x_btn_minus;
	lv_obj_t *Main_label_46;
	lv_obj_t *Main_label_47;
	lv_obj_t *Main_sw_auto_yaw_calibration_cv;
	lv_obj_t *Main_btn_yaw_calibration_cv;
	lv_obj_t *Main_btn_yaw_calibration_cv_label;
	lv_obj_t *Main_label_state;
	lv_obj_t *Main_label_3;
	lv_obj_t *Main_label_ip;
	lv_obj_t *Main_label_title;
	lv_obj_t *Main_label_voltage;
	lv_obj_t *Main_label_37;
	lv_obj_t *Main_msgbox;
	lv_obj_t *Init;
	bool Init_del;
	lv_obj_t *g_kb_Init;
	lv_obj_t *Init_label_3;
	lv_obj_t *Init_label_2;
}lv_ui;

typedef void (*ui_setup_scr_t)(lv_ui * ui);

void ui_init_style(lv_style_t * style);

void ui_load_scr_animation(lv_ui *ui, lv_obj_t ** new_scr, bool new_scr_del, bool * old_scr_del, ui_setup_scr_t setup_scr,
                           lv_scr_load_anim_t anim_type, uint32_t time, uint32_t delay, bool is_clean, bool auto_del);

void ui_move_animation(void * var, int32_t duration, int32_t delay, int32_t x_end, int32_t y_end, lv_anim_path_cb_t path_cb,
                       uint16_t repeat_cnt, uint32_t repeat_delay, uint32_t playback_time, uint32_t playback_delay,
                       lv_anim_start_cb_t start_cb, lv_anim_ready_cb_t ready_cb, lv_anim_deleted_cb_t deleted_cb);

void ui_scale_animation(void * var, int32_t duration, int32_t delay, int32_t width, int32_t height, lv_anim_path_cb_t path_cb,
                        uint16_t repeat_cnt, uint32_t repeat_delay, uint32_t playback_time, uint32_t playback_delay,
                        lv_anim_start_cb_t start_cb, lv_anim_ready_cb_t ready_cb, lv_anim_deleted_cb_t deleted_cb);

void ui_img_zoom_animation(void * var, int32_t duration, int32_t delay, int32_t zoom, lv_anim_path_cb_t path_cb,
                           uint16_t repeat_cnt, uint32_t repeat_delay, uint32_t playback_time, uint32_t playback_delay,
                           lv_anim_start_cb_t start_cb, lv_anim_ready_cb_t ready_cb, lv_anim_deleted_cb_t deleted_cb);

void ui_img_rotate_animation(void * var, int32_t duration, int32_t delay, lv_coord_t x, lv_coord_t y, int32_t rotate,
                   lv_anim_path_cb_t path_cb, uint16_t repeat_cnt, uint32_t repeat_delay, uint32_t playback_time,
                   uint32_t playback_delay, lv_anim_start_cb_t start_cb, lv_anim_ready_cb_t ready_cb, lv_anim_deleted_cb_t deleted_cb);

void init_scr_del_flag(lv_ui *ui);

void setup_ui(lv_ui *ui);


extern lv_ui guider_ui;


void setup_scr_Main(lv_ui *ui);
void setup_scr_Init(lv_ui *ui);

LV_IMG_DECLARE(_logo_3SE_1024x600);

LV_IMG_DECLARE(_logo_3SE_1024x600);

LV_FONT_DECLARE(lv_font_misans_12)
LV_FONT_DECLARE(lv_font_misans_20)
LV_FONT_DECLARE(lv_font_misans_16)
LV_FONT_DECLARE(lv_font_misans_25)
LV_FONT_DECLARE(lv_font_misans_35)
LV_FONT_DECLARE(lv_font_misans_24)
LV_FONT_DECLARE(lv_font_misans_9)
LV_FONT_DECLARE(lv_font_misans_32)
LV_FONT_DECLARE(lv_font_misans_26)
LV_FONT_DECLARE(lv_font_misans_28)
LV_FONT_DECLARE(lv_font_misans_18)
LV_FONT_DECLARE(lv_font_FZShaoLGFTJW_128)


#ifdef __cplusplus
}
#endif
#endif
