/*
 * Copyright 2024 NXP
 * NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be used strictly in
 * accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
 * activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
 * terms, then you may not retain, install, activate or otherwise use the software.
 */

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"

void setup_scr_Main(lv_ui *ui)
{
	// Write codes Main
	ui->Main = lv_obj_create(NULL);
	ui->g_kb_Main = lv_keyboard_create(ui->Main);
	lv_obj_add_event_cb(ui->g_kb_Main, kb_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_add_flag(ui->g_kb_Main, LV_OBJ_FLAG_HIDDEN);
	lv_obj_set_style_text_font(ui->g_kb_Main, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_size(ui->Main, 1024, 600);
	lv_obj_set_scrollbar_mode(ui->Main, LV_SCROLLBAR_MODE_OFF);

	// Write style for Main, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_img_src(ui->Main, &_logo_3SE_1024x600, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_img_opa(ui->Main, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_MainView
	ui->Main_MainView = lv_tabview_create(ui->Main, LV_DIR_LEFT, 60);
	lv_obj_set_pos(ui->Main_MainView, 0, 76);
	lv_obj_set_size(ui->Main_MainView, 1024, 524);
	lv_obj_set_scrollbar_mode(ui->Main_MainView, LV_SCROLLBAR_MODE_ACTIVE);

	// Write style for Main_MainView, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_MainView, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_MainView, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_MainView, &lv_font_misans_12, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_MainView, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_MainView, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_MainView, 16, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_MainView, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_MainView, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_MainView, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_MainView_extra_btnm_main_default
	static lv_style_t style_Main_MainView_extra_btnm_main_default;
	ui_init_style(&style_Main_MainView_extra_btnm_main_default);

	lv_style_set_bg_opa(&style_Main_MainView_extra_btnm_main_default, 124);
	lv_style_set_bg_color(&style_Main_MainView_extra_btnm_main_default, lv_color_hex(0x5a6173));
	lv_style_set_bg_grad_dir(&style_Main_MainView_extra_btnm_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_MainView_extra_btnm_main_default, 0);
	lv_style_set_radius(&style_Main_MainView_extra_btnm_main_default, 12);
	lv_obj_add_style(lv_tabview_get_tab_btns(ui->Main_MainView), &style_Main_MainView_extra_btnm_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_MainView_extra_btnm_items_default
	static lv_style_t style_Main_MainView_extra_btnm_items_default;
	ui_init_style(&style_Main_MainView_extra_btnm_items_default);

	lv_style_set_text_color(&style_Main_MainView_extra_btnm_items_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_MainView_extra_btnm_items_default, &lv_font_misans_20);
	lv_style_set_text_opa(&style_Main_MainView_extra_btnm_items_default, 255);
	lv_obj_add_style(lv_tabview_get_tab_btns(ui->Main_MainView), &style_Main_MainView_extra_btnm_items_default, LV_PART_ITEMS | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_CHECKED for &style_Main_MainView_extra_btnm_items_checked
	static lv_style_t style_Main_MainView_extra_btnm_items_checked;
	ui_init_style(&style_Main_MainView_extra_btnm_items_checked);

	lv_style_set_text_color(&style_Main_MainView_extra_btnm_items_checked, lv_color_hex(0xcc9d11));
	lv_style_set_text_font(&style_Main_MainView_extra_btnm_items_checked, &lv_font_misans_20);
	lv_style_set_text_opa(&style_Main_MainView_extra_btnm_items_checked, 255);
	lv_style_set_border_width(&style_Main_MainView_extra_btnm_items_checked, 4);
	lv_style_set_border_opa(&style_Main_MainView_extra_btnm_items_checked, 255);
	lv_style_set_border_color(&style_Main_MainView_extra_btnm_items_checked, lv_color_hex(0xccac11));
	lv_style_set_border_side(&style_Main_MainView_extra_btnm_items_checked, LV_BORDER_SIDE_RIGHT);
	lv_style_set_radius(&style_Main_MainView_extra_btnm_items_checked, 0);
	lv_style_set_bg_opa(&style_Main_MainView_extra_btnm_items_checked, 82);
	lv_style_set_bg_color(&style_Main_MainView_extra_btnm_items_checked, lv_color_hex(0x151402));
	lv_style_set_bg_grad_dir(&style_Main_MainView_extra_btnm_items_checked, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_tabview_get_tab_btns(ui->Main_MainView), &style_Main_MainView_extra_btnm_items_checked, LV_PART_ITEMS | LV_STATE_CHECKED);

	// Write codes 状态
	ui->Main_MainView_tab_1 = lv_tabview_add_tab(ui->Main_MainView, "状态");
	lv_obj_t *Main_MainView_tab_1_label = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(Main_MainView_tab_1_label, "");

	// Write codes Main_label_2
	ui->Main_label_2 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_2, "摩擦轮主速度/RPM");
	lv_label_set_long_mode(ui->Main_label_2, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_2, 938, 35);
	lv_obj_set_size(ui->Main_label_2, 302, 39);

	// Write style for Main_label_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_2, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_2, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_2, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_2, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_8
	ui->Main_label_8 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_8, "Yaw轴主角度");
	lv_label_set_long_mode(ui->Main_label_8, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_8, 606, 35);
	lv_obj_set_size(ui->Main_label_8, 182, 37);

	// Write style for Main_label_8, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_8, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_8, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_8, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_8, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_8, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_12
	ui->Main_label_12 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_12, "设定值");
	lv_label_set_long_mode(ui->Main_label_12, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_12, 606, -9);
	lv_obj_set_size(ui->Main_label_12, 182, 37);

	// Write style for Main_label_12, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_12, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_12, &lv_font_misans_25, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_12, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_12, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_12, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_yaw_angle
	ui->Main_spinbox_yaw_angle = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_yaw_angle, 666, 80);
	lv_obj_set_width(ui->Main_spinbox_yaw_angle, 180);
	lv_obj_set_height(ui->Main_spinbox_yaw_angle, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_yaw_angle, 6, 4);
	lv_spinbox_set_range(ui->Main_spinbox_yaw_angle, -999999, 999999);
	lv_coord_t Main_spinbox_yaw_angle_h = lv_obj_get_height(ui->Main_spinbox_yaw_angle);
	ui->Main_spinbox_yaw_angle_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_yaw_angle_btn, Main_spinbox_yaw_angle_h, Main_spinbox_yaw_angle_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_angle_btn, ui->Main_spinbox_yaw_angle, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_angle_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_angle_btn, lv_Main_spinbox_yaw_angle_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_yaw_angle_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_yaw_angle_btn_minus, Main_spinbox_yaw_angle_h, Main_spinbox_yaw_angle_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_angle_btn_minus, ui->Main_spinbox_yaw_angle, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_angle_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_angle_btn_minus, lv_Main_spinbox_yaw_angle_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_yaw_angle, 666, 80);

	// Write style for Main_spinbox_yaw_angle, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_angle, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_angle, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_angle, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_yaw_angle, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_yaw_angle, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_yaw_angle, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_yaw_angle, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_yaw_angle, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_angle, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_angle, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_angle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_yaw_angle, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_yaw_angle, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_angle, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_angle, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_angle, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_angle, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_angle, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_angle, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_yaw_angle_extra_btns_main_default
	static lv_style_t style_Main_spinbox_yaw_angle_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_yaw_angle_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_yaw_angle_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_yaw_angle_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_yaw_angle_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_yaw_angle_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_yaw_angle_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_yaw_angle_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_yaw_angle_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_yaw_angle_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_yaw_angle_btn, &style_Main_spinbox_yaw_angle_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_yaw_angle_btn_minus, &style_Main_spinbox_yaw_angle_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_fw_speed
	ui->Main_spinbox_fw_speed = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_fw_speed, 991, 80);
	lv_obj_set_width(ui->Main_spinbox_fw_speed, 180);
	lv_obj_set_height(ui->Main_spinbox_fw_speed, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_fw_speed, 4, 4);
	lv_spinbox_set_range(ui->Main_spinbox_fw_speed, -9999, 9999);
	lv_coord_t Main_spinbox_fw_speed_h = lv_obj_get_height(ui->Main_spinbox_fw_speed);
	ui->Main_spinbox_fw_speed_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_fw_speed_btn, Main_spinbox_fw_speed_h, Main_spinbox_fw_speed_h);
	lv_obj_align_to(ui->Main_spinbox_fw_speed_btn, ui->Main_spinbox_fw_speed, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_fw_speed_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_fw_speed_btn, lv_Main_spinbox_fw_speed_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_fw_speed_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_fw_speed_btn_minus, Main_spinbox_fw_speed_h, Main_spinbox_fw_speed_h);
	lv_obj_align_to(ui->Main_spinbox_fw_speed_btn_minus, ui->Main_spinbox_fw_speed, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_fw_speed_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_fw_speed_btn_minus, lv_Main_spinbox_fw_speed_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_fw_speed, 991, 80);

	// Write style for Main_spinbox_fw_speed, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_fw_speed, 237, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_fw_speed, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_fw_speed, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_fw_speed, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_fw_speed, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_fw_speed, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_fw_speed, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_fw_speed, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_fw_speed, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_fw_speed, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_fw_speed, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_fw_speed, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_fw_speed, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_fw_speed, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_fw_speed, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_fw_speed, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_fw_speed, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_fw_speed, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_fw_speed, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_fw_speed, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_fw_speed, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_fw_speed, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_fw_speed, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_fw_speed, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_fw_speed_extra_btns_main_default
	static lv_style_t style_Main_spinbox_fw_speed_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_fw_speed_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_fw_speed_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_fw_speed_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_fw_speed_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_fw_speed_extra_btns_main_default, 191);
	lv_style_set_bg_color(&style_Main_spinbox_fw_speed_extra_btns_main_default, lv_color_hex(0xb66d1b));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_fw_speed_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_fw_speed_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_fw_speed_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_fw_speed_btn, &style_Main_spinbox_fw_speed_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_fw_speed_btn_minus, &style_Main_spinbox_fw_speed_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_yaw_offset
	ui->Main_spinbox_yaw_offset = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_yaw_offset, 666, 189);
	lv_obj_set_width(ui->Main_spinbox_yaw_offset, 180);
	lv_obj_set_height(ui->Main_spinbox_yaw_offset, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_yaw_offset, 6, 4);
	lv_spinbox_set_range(ui->Main_spinbox_yaw_offset, -999999, 999999);
	lv_coord_t Main_spinbox_yaw_offset_h = lv_obj_get_height(ui->Main_spinbox_yaw_offset);
	ui->Main_spinbox_yaw_offset_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_yaw_offset_btn, Main_spinbox_yaw_offset_h, Main_spinbox_yaw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_offset_btn, ui->Main_spinbox_yaw_offset, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_offset_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_offset_btn, lv_Main_spinbox_yaw_offset_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_yaw_offset_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_yaw_offset_btn_minus, Main_spinbox_yaw_offset_h, Main_spinbox_yaw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_offset_btn_minus, ui->Main_spinbox_yaw_offset, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_offset_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_offset_btn_minus, lv_Main_spinbox_yaw_offset_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_yaw_offset, 666, 189);

	// Write style for Main_spinbox_yaw_offset, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_offset, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_offset, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_yaw_offset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_yaw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_yaw_offset, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_yaw_offset, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_yaw_offset, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_offset, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_offset, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_offset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_yaw_offset, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_yaw_offset, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_offset, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_offset, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_offset, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_offset, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_yaw_offset_extra_btns_main_default
	static lv_style_t style_Main_spinbox_yaw_offset_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_yaw_offset_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_yaw_offset_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_yaw_offset_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_yaw_offset_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_yaw_offset_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_yaw_offset_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_yaw_offset_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_yaw_offset_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_yaw_offset_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_yaw_offset_btn, &style_Main_spinbox_yaw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_yaw_offset_btn_minus, &style_Main_spinbox_yaw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_13
	ui->Main_label_13 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_13, "Yaw轴偏移角度");
	lv_label_set_long_mode(ui->Main_label_13, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_13, 606, 149);
	lv_obj_set_size(ui->Main_label_13, 182, 37);

	// Write style for Main_label_13, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_13, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_13, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_13, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_13, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_13, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_13, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_btn_yaw_calibration
	ui->Main_btn_yaw_calibration = lv_btn_create(ui->Main_MainView_tab_1);
	ui->Main_btn_yaw_calibration_label = lv_label_create(ui->Main_btn_yaw_calibration);
	lv_label_set_text(ui->Main_btn_yaw_calibration_label, "标定Yaw");
	lv_label_set_long_mode(ui->Main_btn_yaw_calibration_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_yaw_calibration_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_yaw_calibration, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_yaw_calibration_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_yaw_calibration, 305, 389);
	lv_obj_set_size(ui->Main_btn_yaw_calibration, 152, 60);

	// Write style for Main_btn_yaw_calibration, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_yaw_calibration, 183, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_yaw_calibration, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_yaw_calibration, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_yaw_calibration, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_yaw_calibration, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_yaw_calibration, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_yaw_calibration, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_yaw_calibration, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_yaw_calibration, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_yaw_calibration, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_14
	ui->Main_label_14 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_14, "摩擦轮偏移速度/RPM");
	lv_label_set_long_mode(ui->Main_label_14, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_14, 939, 149);
	lv_obj_set_size(ui->Main_label_14, 302, 39);

	// Write style for Main_label_14, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_14, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_14, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_14, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_14, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_14, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_14, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_14, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_fw_speed_offset
	ui->Main_spinbox_fw_speed_offset = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_fw_speed_offset, 991, 189);
	lv_obj_set_width(ui->Main_spinbox_fw_speed_offset, 180);
	lv_obj_set_height(ui->Main_spinbox_fw_speed_offset, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_fw_speed_offset, 4, 4);
	lv_spinbox_set_range(ui->Main_spinbox_fw_speed_offset, -9999, 9999);
	lv_coord_t Main_spinbox_fw_speed_offset_h = lv_obj_get_height(ui->Main_spinbox_fw_speed_offset);
	ui->Main_spinbox_fw_speed_offset_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_fw_speed_offset_btn, Main_spinbox_fw_speed_offset_h, Main_spinbox_fw_speed_offset_h);
	lv_obj_align_to(ui->Main_spinbox_fw_speed_offset_btn, ui->Main_spinbox_fw_speed_offset, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_fw_speed_offset_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_fw_speed_offset_btn, lv_Main_spinbox_fw_speed_offset_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_fw_speed_offset_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_fw_speed_offset_btn_minus, Main_spinbox_fw_speed_offset_h, Main_spinbox_fw_speed_offset_h);
	lv_obj_align_to(ui->Main_spinbox_fw_speed_offset_btn_minus, ui->Main_spinbox_fw_speed_offset, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_fw_speed_offset_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_fw_speed_offset_btn_minus, lv_Main_spinbox_fw_speed_offset_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_fw_speed_offset, 991, 189);

	// Write style for Main_spinbox_fw_speed_offset, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_fw_speed_offset, 237, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_fw_speed_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_fw_speed_offset, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_fw_speed_offset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_fw_speed_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_fw_speed_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_fw_speed_offset, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_fw_speed_offset, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_fw_speed_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_fw_speed_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_fw_speed_offset, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_fw_speed_offset, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_fw_speed_offset, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_fw_speed_offset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_fw_speed_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_fw_speed_offset, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_fw_speed_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_fw_speed_offset, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_fw_speed_offset, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_fw_speed_offset, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_fw_speed_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_fw_speed_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_fw_speed_offset, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_fw_speed_offset, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_fw_speed_offset_extra_btns_main_default
	static lv_style_t style_Main_spinbox_fw_speed_offset_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default, 191);
	lv_style_set_bg_color(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default, lv_color_hex(0xb66d1b));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_fw_speed_offset_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_fw_speed_offset_btn, &style_Main_spinbox_fw_speed_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_fw_speed_offset_btn_minus, &style_Main_spinbox_fw_speed_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_15
	ui->Main_label_15 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_15, "Auto");
	lv_label_set_long_mode(ui->Main_label_15, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_15, 487, 378);
	lv_obj_set_size(ui->Main_label_15, 62, 29);

	// Write style for Main_label_15, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_15, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_15, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_15, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_15, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_15, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_15, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_15, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_15, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_15, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_15, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_15, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_15, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_15, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_15, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_16
	ui->Main_label_16 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_16, "摩擦轮速度比");
	lv_label_set_long_mode(ui->Main_label_16, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_16, 939, 258);
	lv_obj_set_size(ui->Main_label_16, 302, 39);

	// Write style for Main_label_16, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_16, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_16, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_16, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_16, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_16, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_16, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_16, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_16, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_16, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_16, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_16, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_16, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_16, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_16, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_fw_speed_ratio
	ui->Main_spinbox_fw_speed_ratio = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_fw_speed_ratio, 992, 296);
	lv_obj_set_width(ui->Main_spinbox_fw_speed_ratio, 180);
	lv_obj_set_height(ui->Main_spinbox_fw_speed_ratio, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_fw_speed_ratio, 4, 1);
	lv_spinbox_set_range(ui->Main_spinbox_fw_speed_ratio, -9999, 9999);
	lv_coord_t Main_spinbox_fw_speed_ratio_h = lv_obj_get_height(ui->Main_spinbox_fw_speed_ratio);
	ui->Main_spinbox_fw_speed_ratio_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_fw_speed_ratio_btn, Main_spinbox_fw_speed_ratio_h, Main_spinbox_fw_speed_ratio_h);
	lv_obj_align_to(ui->Main_spinbox_fw_speed_ratio_btn, ui->Main_spinbox_fw_speed_ratio, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_fw_speed_ratio_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_fw_speed_ratio_btn, lv_Main_spinbox_fw_speed_ratio_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_fw_speed_ratio_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_fw_speed_ratio_btn_minus, Main_spinbox_fw_speed_ratio_h, Main_spinbox_fw_speed_ratio_h);
	lv_obj_align_to(ui->Main_spinbox_fw_speed_ratio_btn_minus, ui->Main_spinbox_fw_speed_ratio, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_fw_speed_ratio_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_fw_speed_ratio_btn_minus, lv_Main_spinbox_fw_speed_ratio_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_fw_speed_ratio, 992, 296);

	// Write style for Main_spinbox_fw_speed_ratio, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_fw_speed_ratio, 237, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_fw_speed_ratio, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_fw_speed_ratio, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_fw_speed_ratio, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_fw_speed_ratio, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_fw_speed_ratio, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_fw_speed_ratio, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_fw_speed_ratio, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_fw_speed_ratio, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_fw_speed_ratio, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_fw_speed_ratio, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_fw_speed_ratio, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_fw_speed_ratio, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_fw_speed_ratio, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_fw_speed_ratio, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_fw_speed_ratio, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_fw_speed_ratio, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_fw_speed_ratio, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_fw_speed_ratio, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_fw_speed_ratio, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_fw_speed_ratio, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_fw_speed_ratio, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_fw_speed_ratio, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_fw_speed_ratio, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_fw_speed_ratio_extra_btns_main_default
	static lv_style_t style_Main_spinbox_fw_speed_ratio_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, 191);
	lv_style_set_bg_color(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, lv_color_hex(0xb66d1b));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_fw_speed_ratio_btn, &style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_fw_speed_ratio_btn_minus, &style_Main_spinbox_fw_speed_ratio_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_meter_fw_speed_1
	ui->Main_meter_fw_speed_1 = lv_meter_create(ui->Main_MainView_tab_1);
	// add scale Main_meter_fw_speed_1_scale_0
	lv_meter_scale_t *Main_meter_fw_speed_1_scale_0 = lv_meter_add_scale(ui->Main_meter_fw_speed_1);
	lv_meter_set_scale_ticks(ui->Main_meter_fw_speed_1, Main_meter_fw_speed_1_scale_0, 51, 1, 6, lv_color_hex(0xffffff));
	lv_meter_set_scale_major_ticks(ui->Main_meter_fw_speed_1, Main_meter_fw_speed_1_scale_0, 10, 2, 12, lv_color_hex(0xffffff), 20);
	lv_meter_set_scale_range(ui->Main_meter_fw_speed_1, Main_meter_fw_speed_1_scale_0, 4500, 5500, 300, 120);

	// add needle line for Main_meter_fw_speed_1_scale_0.
	ui->Main_meter_fw_speed_1_scale_0_ndline_0 = lv_meter_add_needle_line(ui->Main_meter_fw_speed_1, Main_meter_fw_speed_1_scale_0, 3, lv_color_hex(0x00a1b5), 0);
	lv_meter_set_indicator_value(ui->Main_meter_fw_speed_1, ui->Main_meter_fw_speed_1_scale_0_ndline_0, 5000);
	lv_obj_set_pos(ui->Main_meter_fw_speed_1, 18, -4);
	lv_obj_set_size(ui->Main_meter_fw_speed_1, 250, 250);

	// Write style for Main_meter_fw_speed_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_meter_fw_speed_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_meter_fw_speed_1, lv_color_hex(0x06575c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_meter_fw_speed_1, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_color(ui->Main_meter_fw_speed_1, lv_color_hex(0x6b4010), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_main_stop(ui->Main_meter_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_stop(ui->Main_meter_fw_speed_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_meter_fw_speed_1, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_meter_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_meter_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_meter_fw_speed_1, Part: LV_PART_TICKS, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_meter_fw_speed_1, lv_color_hex(0xffffff), LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_meter_fw_speed_1, &lv_font_misans_9, LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_meter_fw_speed_1, 255, LV_PART_TICKS | LV_STATE_DEFAULT);

	// Write style for Main_meter_fw_speed_1, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_meter_fw_speed_1, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_meter_fw_speed_1, lv_color_hex(0x00a1b5), LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_meter_fw_speed_1, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_DEFAULT);

	// Write codes Main_meter_fw_speed_2
	ui->Main_meter_fw_speed_2 = lv_meter_create(ui->Main_MainView_tab_1);
	// add scale Main_meter_fw_speed_2_scale_0
	lv_meter_scale_t *Main_meter_fw_speed_2_scale_0 = lv_meter_add_scale(ui->Main_meter_fw_speed_2);
	lv_meter_set_scale_ticks(ui->Main_meter_fw_speed_2, Main_meter_fw_speed_2_scale_0, 51, 1, 6, lv_color_hex(0xffffff));
	lv_meter_set_scale_major_ticks(ui->Main_meter_fw_speed_2, Main_meter_fw_speed_2_scale_0, 10, 2, 12, lv_color_hex(0xffffff), 20);
	lv_meter_set_scale_range(ui->Main_meter_fw_speed_2, Main_meter_fw_speed_2_scale_0, 6000, 8000, 300, 120);

	// add needle line for Main_meter_fw_speed_2_scale_0.
	ui->Main_meter_fw_speed_2_scale_0_ndline_0 = lv_meter_add_needle_line(ui->Main_meter_fw_speed_2, Main_meter_fw_speed_2_scale_0, 3, lv_color_hex(0x00a1b5), 0);
	lv_meter_set_indicator_value(ui->Main_meter_fw_speed_2, ui->Main_meter_fw_speed_2_scale_0_ndline_0, 7070);
	lv_obj_set_pos(ui->Main_meter_fw_speed_2, 296, -4);
	lv_obj_set_size(ui->Main_meter_fw_speed_2, 250, 250);

	// Write style for Main_meter_fw_speed_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_meter_fw_speed_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_meter_fw_speed_2, lv_color_hex(0x06575c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_meter_fw_speed_2, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_color(ui->Main_meter_fw_speed_2, lv_color_hex(0x6b4010), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_main_stop(ui->Main_meter_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_stop(ui->Main_meter_fw_speed_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_meter_fw_speed_2, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_meter_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_meter_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_meter_fw_speed_2, Part: LV_PART_TICKS, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_meter_fw_speed_2, lv_color_hex(0xffffff), LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_meter_fw_speed_2, &lv_font_misans_9, LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_meter_fw_speed_2, 255, LV_PART_TICKS | LV_STATE_DEFAULT);

	// Write style for Main_meter_fw_speed_2, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_meter_fw_speed_2, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_meter_fw_speed_2, lv_color_hex(0x00a1b5), LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_meter_fw_speed_2, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_DEFAULT);

	// Write codes Main_label_17
	ui->Main_label_17 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_17, "一级平均");
	lv_label_set_long_mode(ui->Main_label_17, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_17, 61.5, 252);
	lv_obj_set_size(ui->Main_label_17, 163, 37);

	// Write style for Main_label_17, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_17, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_17, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_17, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_17, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_17, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_17, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_17, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_17, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_17, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_17, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_17, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_17, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_17, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_17, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_18
	ui->Main_label_18 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_18, "二级平均");
	lv_label_set_long_mode(ui->Main_label_18, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_18, 340.5, 252);
	lv_obj_set_size(ui->Main_label_18, 163, 37);

	// Write style for Main_label_18, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_18, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_18, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_18, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_18, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_18, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_18, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_18, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_18, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_18, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_18, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_18, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_18, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_18, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_18, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_fw_speed_1
	ui->Main_label_fw_speed_1 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_fw_speed_1, "0");
	lv_label_set_long_mode(ui->Main_label_fw_speed_1, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_fw_speed_1, 92, 285);
	lv_obj_set_size(ui->Main_label_fw_speed_1, 102, 41);

	// Write style for Main_label_fw_speed_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_fw_speed_1, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_fw_speed_1, &lv_font_misans_32, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_fw_speed_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_fw_speed_1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_fw_speed_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_fw_speed_1, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_fw_speed_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_fw_speed_2
	ui->Main_label_fw_speed_2 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_fw_speed_2, "0");
	lv_label_set_long_mode(ui->Main_label_fw_speed_2, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_fw_speed_2, 368, 285);
	lv_obj_set_size(ui->Main_label_fw_speed_2, 108, 41);

	// Write style for Main_label_fw_speed_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_fw_speed_2, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_fw_speed_2, &lv_font_misans_32, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_fw_speed_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_fw_speed_2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_fw_speed_2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_fw_speed_2, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_fw_speed_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_bar_yaw_angle
	ui->Main_bar_yaw_angle = lv_bar_create(ui->Main_MainView_tab_1);
	lv_obj_set_style_anim_time(ui->Main_bar_yaw_angle, 1000, 0);
	lv_bar_set_mode(ui->Main_bar_yaw_angle, LV_BAR_MODE_NORMAL);
	lv_bar_set_range(ui->Main_bar_yaw_angle, 0, 100);
	lv_bar_set_value(ui->Main_bar_yaw_angle, 50, LV_ANIM_OFF);
	lv_obj_set_pos(ui->Main_bar_yaw_angle, 57, 384);
	lv_obj_set_size(ui->Main_bar_yaw_angle, 200, 10);

	// Write style for Main_bar_yaw_angle, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_bar_yaw_angle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_bar_yaw_angle, lv_color_hex(0x393c41), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_bar_yaw_angle, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_bar_yaw_angle, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_bar_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_bar_yaw_angle, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_bar_yaw_angle, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_bar_yaw_angle, lv_color_hex(0x00a1b5), LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_bar_yaw_angle, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_bar_yaw_angle, 10, LV_PART_INDICATOR | LV_STATE_DEFAULT);

	// Write codes Main_label_21
	ui->Main_label_21 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_21, "Yaw轴角度");
	lv_label_set_long_mode(ui->Main_label_21, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_21, 56, 338);
	lv_obj_set_size(ui->Main_label_21, 182, 34);

	// Write style for Main_label_21, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_21, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_21, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_21, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_21, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_21, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_21, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_21, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_21, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_21, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_21, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_21, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_21, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_21, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_21, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_yaw_angle
	ui->Main_label_yaw_angle = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_yaw_angle, "0");
	lv_label_set_long_mode(ui->Main_label_yaw_angle, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_yaw_angle, 170, 336);
	lv_obj_set_size(ui->Main_label_yaw_angle, 135, 39);

	// Write style for Main_label_yaw_angle, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_yaw_angle, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_yaw_angle, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_yaw_angle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_yaw_angle, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_yaw_angle, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_yaw_angle, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_yaw_angle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_bar_launch_progress
	ui->Main_bar_launch_progress = lv_bar_create(ui->Main_MainView_tab_1);
	lv_obj_set_style_anim_time(ui->Main_bar_launch_progress, 1000, 0);
	lv_bar_set_mode(ui->Main_bar_launch_progress, LV_BAR_MODE_NORMAL);
	lv_bar_set_range(ui->Main_bar_launch_progress, 0, 100);
	lv_bar_set_value(ui->Main_bar_launch_progress, 50, LV_ANIM_OFF);
	lv_obj_set_pos(ui->Main_bar_launch_progress, 57, 443);
	lv_obj_set_size(ui->Main_bar_launch_progress, 200, 10);

	// Write style for Main_bar_launch_progress, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_bar_launch_progress, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_bar_launch_progress, lv_color_hex(0x393c41), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_bar_launch_progress, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_bar_launch_progress, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_bar_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_bar_launch_progress, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_bar_launch_progress, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_bar_launch_progress, lv_color_hex(0x00a1b5), LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_bar_launch_progress, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_bar_launch_progress, 10, LV_PART_INDICATOR | LV_STATE_DEFAULT);

	// Write codes Main_label_launch_progress
	ui->Main_label_launch_progress = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_launch_progress, "0/4");
	lv_label_set_long_mode(ui->Main_label_launch_progress, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_launch_progress, 167, 399);
	lv_obj_set_size(ui->Main_label_launch_progress, 59, 36);

	// Write style for Main_label_launch_progress, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_launch_progress, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_launch_progress, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_launch_progress, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_launch_progress, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_launch_progress, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_launch_progress, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_launch_progress, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_23
	ui->Main_label_23 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_23, "复位中等待中正在推出装填中不适用");
	lv_label_set_long_mode(ui->Main_label_23, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_23, 55, 402);
	lv_obj_set_size(ui->Main_label_23, 98, 28);

	// Write style for Main_label_23, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_23, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_23, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_23, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_23, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_23, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_23, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_23, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_23, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_23, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_23, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_23, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_23, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_23, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_23, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_25
	ui->Main_label_25 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_25, "识别位置");
	lv_label_set_long_mode(ui->Main_label_25, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_25, 306, 351);
	lv_obj_set_size(ui->Main_label_25, 182, 34);

	// Write style for Main_label_25, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_25, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_25, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_25, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_25, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_25, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_25, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_25, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_25, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_25, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_25, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_25, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_25, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_25, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_25, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_yaw_location
	ui->Main_label_yaw_location = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_yaw_location, "N/A");
	lv_label_set_long_mode(ui->Main_label_yaw_location, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_yaw_location, 402, 349);
	lv_obj_set_size(ui->Main_label_yaw_location, 191, 39);

	// Write style for Main_label_yaw_location, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_yaw_location, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_yaw_location, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_yaw_location, lv_color_hex(0xb5e3e9), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_yaw_location, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_yaw_location, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_yaw_location, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_yaw_location, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_yaw_location, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_yaw_location, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_yaw_location, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_yaw_location, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_yaw_location, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_yaw_location, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_yaw_location, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_27
	ui->Main_label_27 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_27, "目标距离Dis");
	lv_label_set_long_mode(ui->Main_label_27, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_27, 606, 258);
	lv_obj_set_size(ui->Main_label_27, 182, 34);

	// Write style for Main_label_27, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_27, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_27, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_27, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_27, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_27, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_27, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_27, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_27, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_27, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_27, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_27, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_27, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_27, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_27, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_btn_rpm_calibration
	ui->Main_btn_rpm_calibration = lv_btn_create(ui->Main_MainView_tab_1);
	ui->Main_btn_rpm_calibration_label = lv_label_create(ui->Main_btn_rpm_calibration);
	lv_label_set_text(ui->Main_btn_rpm_calibration_label, "解算rpm");
	lv_label_set_long_mode(ui->Main_btn_rpm_calibration_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_rpm_calibration_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_rpm_calibration, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_rpm_calibration_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_rpm_calibration, 935, 389);
	lv_obj_set_size(ui->Main_btn_rpm_calibration, 152, 60);

	// Write style for Main_btn_rpm_calibration, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_rpm_calibration, 183, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_rpm_calibration, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_rpm_calibration, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_rpm_calibration, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_rpm_calibration, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_rpm_calibration, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_rpm_calibration, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_rpm_calibration, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_rpm_calibration, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_rpm_calibration, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_29
	ui->Main_label_29 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_29, "Auto");
	lv_label_set_long_mode(ui->Main_label_29, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_29, 1121, 378);
	lv_obj_set_size(ui->Main_label_29, 62, 29);

	// Write style for Main_label_29, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_29, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_29, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_29, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_29, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_29, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_29, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_29, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_29, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_29, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_29, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_29, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_29, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_29, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_29, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_sw_auto_rpm_calibration
	ui->Main_sw_auto_rpm_calibration = lv_switch_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_sw_auto_rpm_calibration, 1112, 418);
	lv_obj_set_size(ui->Main_sw_auto_rpm_calibration, 64, 30);

	// Write style for Main_sw_auto_rpm_calibration, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_rpm_calibration, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_rpm_calibration, lv_color_hex(0x41485a), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_rpm_calibration, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_sw_auto_rpm_calibration, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_sw_auto_rpm_calibration, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_sw_auto_rpm_calibration, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_sw_auto_rpm_calibration, Part: LV_PART_INDICATOR, State: LV_STATE_CHECKED.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_rpm_calibration, 68, LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_rpm_calibration, lv_color_hex(0x00a1b5), LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_rpm_calibration, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_border_width(ui->Main_sw_auto_rpm_calibration, 0, LV_PART_INDICATOR | LV_STATE_CHECKED);

	// Write style for Main_sw_auto_rpm_calibration, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_rpm_calibration, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_rpm_calibration, lv_color_hex(0x00a1b5), LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_rpm_calibration, LV_GRAD_DIR_NONE, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_sw_auto_rpm_calibration, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_sw_auto_rpm_calibration, 10, LV_PART_KNOB | LV_STATE_DEFAULT);

	// Write codes Main_label_30
	ui->Main_label_30 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_30, "发射槽位1");
	lv_label_set_long_mode(ui->Main_label_30, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_30, 1265, 29);
	lv_obj_set_size(ui->Main_label_30, 302, 39);

	// Write style for Main_label_30, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_30, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_30, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_30, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_30, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_30, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_30, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_30, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_30, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_30, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_30, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_30, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_30, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_30, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_30, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_ddlist_launch_dart_1
	ui->Main_ddlist_launch_dart_1 = lv_dropdown_create(ui->Main_MainView_tab_1);
	lv_dropdown_set_options(ui->Main_ddlist_launch_dart_1, "7.9\n7.10\n7.11\n7.14\n7.15");
	lv_obj_set_pos(ui->Main_ddlist_launch_dart_1, 1265, 68);
	lv_obj_set_size(ui->Main_ddlist_launch_dart_1, 200, 53);

	// Write style for Main_ddlist_launch_dart_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_1, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_1, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_1, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_1, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_1, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_1, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_ddlist_launch_dart_1, Part: LV_PART_MAIN, State: LV_STATE_CHECKED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_1, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_1, 1, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_1, 12, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_1, 15, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_1, 6, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_1, 3, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_1, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_1, 0, LV_PART_MAIN | LV_STATE_CHECKED);

	// Write style for Main_ddlist_launch_dart_1, Part: LV_PART_MAIN, State: LV_STATE_FOCUSED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_1, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_1, 1, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_1, 12, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_1, 15, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_1, 6, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_1, 3, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_1, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_1, 0, LV_PART_MAIN | LV_STATE_FOCUSED);

	// Write style for Main_ddlist_launch_dart_1, Part: LV_PART_MAIN, State: LV_STATE_DISABLED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_1, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_1, 1, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0xe1e6ee), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_1, 12, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_1, 15, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_1, 6, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_1, 3, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_1, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_1, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_1, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_1, 0, LV_PART_MAIN | LV_STATE_DISABLED);

	// Write style state: LV_STATE_CHECKED for &style_Main_ddlist_launch_dart_1_extra_list_selected_checked
	static lv_style_t style_Main_ddlist_launch_dart_1_extra_list_selected_checked;
	ui_init_style(&style_Main_ddlist_launch_dart_1_extra_list_selected_checked);

	lv_style_set_border_width(&style_Main_ddlist_launch_dart_1_extra_list_selected_checked, 0);
	lv_style_set_radius(&style_Main_ddlist_launch_dart_1_extra_list_selected_checked, 3);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_1_extra_list_selected_checked, 0);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_1), &style_Main_ddlist_launch_dart_1_extra_list_selected_checked, LV_PART_SELECTED | LV_STATE_CHECKED);

	// Write style state: LV_STATE_DEFAULT for &style_Main_ddlist_launch_dart_1_extra_list_main_default
	static lv_style_t style_Main_ddlist_launch_dart_1_extra_list_main_default;
	ui_init_style(&style_Main_ddlist_launch_dart_1_extra_list_main_default);

	lv_style_set_max_height(&style_Main_ddlist_launch_dart_1_extra_list_main_default, 90);
	lv_style_set_text_color(&style_Main_ddlist_launch_dart_1_extra_list_main_default, lv_color_hex(0x111111));
	lv_style_set_text_font(&style_Main_ddlist_launch_dart_1_extra_list_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_ddlist_launch_dart_1_extra_list_main_default, 255);
	lv_style_set_border_width(&style_Main_ddlist_launch_dart_1_extra_list_main_default, 1);
	lv_style_set_border_opa(&style_Main_ddlist_launch_dart_1_extra_list_main_default, 255);
	lv_style_set_border_color(&style_Main_ddlist_launch_dart_1_extra_list_main_default, lv_color_hex(0x654f4f));
	lv_style_set_border_side(&style_Main_ddlist_launch_dart_1_extra_list_main_default, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_ddlist_launch_dart_1_extra_list_main_default, 10);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_1_extra_list_main_default, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_1_extra_list_main_default, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_1_extra_list_main_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_1), &style_Main_ddlist_launch_dart_1_extra_list_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_ddlist_launch_dart_1_extra_list_scrollbar_default
	static lv_style_t style_Main_ddlist_launch_dart_1_extra_list_scrollbar_default;
	ui_init_style(&style_Main_ddlist_launch_dart_1_extra_list_scrollbar_default);

	lv_style_set_radius(&style_Main_ddlist_launch_dart_1_extra_list_scrollbar_default, 3);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_1_extra_list_scrollbar_default, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_1_extra_list_scrollbar_default, lv_color_hex(0x5d5d5d));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_1_extra_list_scrollbar_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_1), &style_Main_ddlist_launch_dart_1_extra_list_scrollbar_default, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

	// Write codes Main_label_34
	ui->Main_label_34 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_34, "发射槽位2");
	lv_label_set_long_mode(ui->Main_label_34, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_34, 1265, 139);
	lv_obj_set_size(ui->Main_label_34, 302, 39);

	// Write style for Main_label_34, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_34, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_34, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_34, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_34, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_34, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_34, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_34, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_34, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_34, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_34, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_34, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_34, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_34, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_34, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_ddlist_launch_dart_3
	ui->Main_ddlist_launch_dart_3 = lv_dropdown_create(ui->Main_MainView_tab_1);
	lv_dropdown_set_options(ui->Main_ddlist_launch_dart_3, "7.9\n7.10\n7.11\n7.14\n7.15");
	lv_obj_set_pos(ui->Main_ddlist_launch_dart_3, 1265, 286);
	lv_obj_set_size(ui->Main_ddlist_launch_dart_3, 200, 53);

	// Write style for Main_ddlist_launch_dart_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_3, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_3, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_3, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_3, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_3, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_3, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_3, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_3, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_ddlist_launch_dart_3, Part: LV_PART_MAIN, State: LV_STATE_CHECKED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_3, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_3, 1, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_3, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_3, 12, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_3, 15, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_3, 6, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_3, 3, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_3, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_3, 0, LV_PART_MAIN | LV_STATE_CHECKED);

	// Write style for Main_ddlist_launch_dart_3, Part: LV_PART_MAIN, State: LV_STATE_FOCUSED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_3, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_3, 1, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_3, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_3, 12, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_3, 15, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_3, 6, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_3, 3, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_3, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_3, 0, LV_PART_MAIN | LV_STATE_FOCUSED);

	// Write style for Main_ddlist_launch_dart_3, Part: LV_PART_MAIN, State: LV_STATE_DISABLED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_3, &lv_font_misans_12, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_3, 1, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0xe1e6ee), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_3, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_3, 8, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_3, 6, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_3, 6, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_3, 3, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_3, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_3, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_3, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_3, 0, LV_PART_MAIN | LV_STATE_DISABLED);

	// Write style state: LV_STATE_CHECKED for &style_Main_ddlist_launch_dart_3_extra_list_selected_checked
	static lv_style_t style_Main_ddlist_launch_dart_3_extra_list_selected_checked;
	ui_init_style(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked);

	lv_style_set_border_width(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked, 2);
	lv_style_set_border_opa(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked, 255);
	lv_style_set_border_color(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked, lv_color_hex(0x654f4f));
	lv_style_set_border_side(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked, 3);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_3_extra_list_selected_checked, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_3), &style_Main_ddlist_launch_dart_3_extra_list_selected_checked, LV_PART_SELECTED | LV_STATE_CHECKED);

	// Write style state: LV_STATE_DEFAULT for &style_Main_ddlist_launch_dart_3_extra_list_main_default
	static lv_style_t style_Main_ddlist_launch_dart_3_extra_list_main_default;
	ui_init_style(&style_Main_ddlist_launch_dart_3_extra_list_main_default);

	lv_style_set_max_height(&style_Main_ddlist_launch_dart_3_extra_list_main_default, 90);
	lv_style_set_text_color(&style_Main_ddlist_launch_dart_3_extra_list_main_default, lv_color_hex(0x000000));
	lv_style_set_text_font(&style_Main_ddlist_launch_dart_3_extra_list_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_ddlist_launch_dart_3_extra_list_main_default, 255);
	lv_style_set_border_width(&style_Main_ddlist_launch_dart_3_extra_list_main_default, 1);
	lv_style_set_border_opa(&style_Main_ddlist_launch_dart_3_extra_list_main_default, 255);
	lv_style_set_border_color(&style_Main_ddlist_launch_dart_3_extra_list_main_default, lv_color_hex(0x654f4f));
	lv_style_set_border_side(&style_Main_ddlist_launch_dart_3_extra_list_main_default, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_ddlist_launch_dart_3_extra_list_main_default, 10);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_3_extra_list_main_default, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_3_extra_list_main_default, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_3_extra_list_main_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_3), &style_Main_ddlist_launch_dart_3_extra_list_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_ddlist_launch_dart_3_extra_list_scrollbar_default
	static lv_style_t style_Main_ddlist_launch_dart_3_extra_list_scrollbar_default;
	ui_init_style(&style_Main_ddlist_launch_dart_3_extra_list_scrollbar_default);

	lv_style_set_radius(&style_Main_ddlist_launch_dart_3_extra_list_scrollbar_default, 3);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_3_extra_list_scrollbar_default, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_3_extra_list_scrollbar_default, lv_color_hex(0x5d5d5d));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_3_extra_list_scrollbar_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_3), &style_Main_ddlist_launch_dart_3_extra_list_scrollbar_default, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

	// Write codes Main_label_33
	ui->Main_label_33 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_33, "发射槽位3");
	lv_label_set_long_mode(ui->Main_label_33, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_33, 1265, 249);
	lv_obj_set_size(ui->Main_label_33, 302, 39);

	// Write style for Main_label_33, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_33, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_33, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_33, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_33, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_33, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_33, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_33, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_33, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_33, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_33, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_33, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_33, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_33, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_33, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_32
	ui->Main_label_32 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_32, "发射槽位4");
	lv_label_set_long_mode(ui->Main_label_32, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_32, 1265, 359);
	lv_obj_set_size(ui->Main_label_32, 302, 39);

	// Write style for Main_label_32, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_32, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_32, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_32, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_32, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_32, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_32, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_32, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_32, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_32, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_32, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_32, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_32, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_32, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_32, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_ddlist_launch_dart_4
	ui->Main_ddlist_launch_dart_4 = lv_dropdown_create(ui->Main_MainView_tab_1);
	lv_dropdown_set_options(ui->Main_ddlist_launch_dart_4, "7.9\n7.10\n7.11\n7.14\n7.15");
	lv_obj_set_pos(ui->Main_ddlist_launch_dart_4, 1265, 395);
	lv_obj_set_size(ui->Main_ddlist_launch_dart_4, 200, 53);

	// Write style for Main_ddlist_launch_dart_4, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_4, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_4, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_4, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_4, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_4, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_4, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_4, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_4, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_ddlist_launch_dart_4, Part: LV_PART_MAIN, State: LV_STATE_CHECKED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_4, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_4, 1, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_4, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_4, 12, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_4, 15, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_4, 6, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_4, 3, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_4, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_4, 0, LV_PART_MAIN | LV_STATE_CHECKED);

	// Write style for Main_ddlist_launch_dart_4, Part: LV_PART_MAIN, State: LV_STATE_FOCUSED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_4, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_4, 1, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_4, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_4, 12, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_4, 15, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_4, 6, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_4, 3, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_4, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_4, 0, LV_PART_MAIN | LV_STATE_FOCUSED);

	// Write style for Main_ddlist_launch_dart_4, Part: LV_PART_MAIN, State: LV_STATE_DISABLED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_4, &lv_font_misans_12, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_4, 1, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0xe1e6ee), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_4, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_4, 8, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_4, 6, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_4, 6, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_4, 3, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_4, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_4, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_4, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_4, 0, LV_PART_MAIN | LV_STATE_DISABLED);

	// Write style state: LV_STATE_CHECKED for &style_Main_ddlist_launch_dart_4_extra_list_selected_checked
	static lv_style_t style_Main_ddlist_launch_dart_4_extra_list_selected_checked;
	ui_init_style(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked);

	lv_style_set_border_width(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked, 2);
	lv_style_set_border_opa(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked, 255);
	lv_style_set_border_color(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked, lv_color_hex(0x654f4f));
	lv_style_set_border_side(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked, 3);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_4_extra_list_selected_checked, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_4), &style_Main_ddlist_launch_dart_4_extra_list_selected_checked, LV_PART_SELECTED | LV_STATE_CHECKED);

	// Write style state: LV_STATE_DEFAULT for &style_Main_ddlist_launch_dart_4_extra_list_main_default
	static lv_style_t style_Main_ddlist_launch_dart_4_extra_list_main_default;
	ui_init_style(&style_Main_ddlist_launch_dart_4_extra_list_main_default);

	lv_style_set_max_height(&style_Main_ddlist_launch_dart_4_extra_list_main_default, 90);
	lv_style_set_text_color(&style_Main_ddlist_launch_dart_4_extra_list_main_default, lv_color_hex(0x000000));
	lv_style_set_text_font(&style_Main_ddlist_launch_dart_4_extra_list_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_ddlist_launch_dart_4_extra_list_main_default, 255);
	lv_style_set_border_width(&style_Main_ddlist_launch_dart_4_extra_list_main_default, 1);
	lv_style_set_border_opa(&style_Main_ddlist_launch_dart_4_extra_list_main_default, 255);
	lv_style_set_border_color(&style_Main_ddlist_launch_dart_4_extra_list_main_default, lv_color_hex(0x654f4f));
	lv_style_set_border_side(&style_Main_ddlist_launch_dart_4_extra_list_main_default, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_ddlist_launch_dart_4_extra_list_main_default, 10);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_4_extra_list_main_default, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_4_extra_list_main_default, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_4_extra_list_main_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_4), &style_Main_ddlist_launch_dart_4_extra_list_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_ddlist_launch_dart_4_extra_list_scrollbar_default
	static lv_style_t style_Main_ddlist_launch_dart_4_extra_list_scrollbar_default;
	ui_init_style(&style_Main_ddlist_launch_dart_4_extra_list_scrollbar_default);

	lv_style_set_radius(&style_Main_ddlist_launch_dart_4_extra_list_scrollbar_default, 3);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_4_extra_list_scrollbar_default, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_4_extra_list_scrollbar_default, lv_color_hex(0x5d5d5d));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_4_extra_list_scrollbar_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_4), &style_Main_ddlist_launch_dart_4_extra_list_scrollbar_default, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

	// Write codes Main_ddlist_launch_dart_2
	ui->Main_ddlist_launch_dart_2 = lv_dropdown_create(ui->Main_MainView_tab_1);
	lv_dropdown_set_options(ui->Main_ddlist_launch_dart_2, "7.9\n7.10\n7.11\n7.14\n7.15");
	lv_obj_set_pos(ui->Main_ddlist_launch_dart_2, 1265, 177);
	lv_obj_set_size(ui->Main_ddlist_launch_dart_2, 200, 53);

	// Write style for Main_ddlist_launch_dart_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_2, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_2, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_2, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_2, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_2, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_2, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_2, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_2, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_ddlist_launch_dart_2, Part: LV_PART_MAIN, State: LV_STATE_CHECKED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_2, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_2, 1, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_2, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_2, 12, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_2, 15, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_2, 6, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_2, 3, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_2, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_CHECKED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_2, 0, LV_PART_MAIN | LV_STATE_CHECKED);

	// Write style for Main_ddlist_launch_dart_2, Part: LV_PART_MAIN, State: LV_STATE_FOCUSED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_2, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_2, 1, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0x654f4f), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_2, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_2, 12, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_2, 15, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_2, 6, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_2, 3, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_2, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_2, 0, LV_PART_MAIN | LV_STATE_FOCUSED);

	// Write style for Main_ddlist_launch_dart_2, Part: LV_PART_MAIN, State: LV_STATE_DISABLED.
	lv_obj_set_style_text_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0x0D3055), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_text_font(ui->Main_ddlist_launch_dart_2, &lv_font_misans_12, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_text_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_width(ui->Main_ddlist_launch_dart_2, 1, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0xe1e6ee), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_border_side(ui->Main_ddlist_launch_dart_2, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_top(ui->Main_ddlist_launch_dart_2, 8, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_left(ui->Main_ddlist_launch_dart_2, 6, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_pad_right(ui->Main_ddlist_launch_dart_2, 6, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_radius(ui->Main_ddlist_launch_dart_2, 3, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_opa(ui->Main_ddlist_launch_dart_2, 255, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_color(ui->Main_ddlist_launch_dart_2, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ddlist_launch_dart_2, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DISABLED);
	lv_obj_set_style_shadow_width(ui->Main_ddlist_launch_dart_2, 0, LV_PART_MAIN | LV_STATE_DISABLED);

	// Write style state: LV_STATE_CHECKED for &style_Main_ddlist_launch_dart_2_extra_list_selected_checked
	static lv_style_t style_Main_ddlist_launch_dart_2_extra_list_selected_checked;
	ui_init_style(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked);

	lv_style_set_border_width(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked, 2);
	lv_style_set_border_opa(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked, 255);
	lv_style_set_border_color(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked, lv_color_hex(0x654f4f));
	lv_style_set_border_side(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked, 3);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_2_extra_list_selected_checked, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_2), &style_Main_ddlist_launch_dart_2_extra_list_selected_checked, LV_PART_SELECTED | LV_STATE_CHECKED);

	// Write style state: LV_STATE_DEFAULT for &style_Main_ddlist_launch_dart_2_extra_list_main_default
	static lv_style_t style_Main_ddlist_launch_dart_2_extra_list_main_default;
	ui_init_style(&style_Main_ddlist_launch_dart_2_extra_list_main_default);

	lv_style_set_max_height(&style_Main_ddlist_launch_dart_2_extra_list_main_default, 90);
	lv_style_set_text_color(&style_Main_ddlist_launch_dart_2_extra_list_main_default, lv_color_hex(0x000000));
	lv_style_set_text_font(&style_Main_ddlist_launch_dart_2_extra_list_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_ddlist_launch_dart_2_extra_list_main_default, 255);
	lv_style_set_border_width(&style_Main_ddlist_launch_dart_2_extra_list_main_default, 1);
	lv_style_set_border_opa(&style_Main_ddlist_launch_dart_2_extra_list_main_default, 255);
	lv_style_set_border_color(&style_Main_ddlist_launch_dart_2_extra_list_main_default, lv_color_hex(0x654f4f));
	lv_style_set_border_side(&style_Main_ddlist_launch_dart_2_extra_list_main_default, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_ddlist_launch_dart_2_extra_list_main_default, 10);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_2_extra_list_main_default, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_2_extra_list_main_default, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_2_extra_list_main_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_2), &style_Main_ddlist_launch_dart_2_extra_list_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_ddlist_launch_dart_2_extra_list_scrollbar_default
	static lv_style_t style_Main_ddlist_launch_dart_2_extra_list_scrollbar_default;
	ui_init_style(&style_Main_ddlist_launch_dart_2_extra_list_scrollbar_default);

	lv_style_set_radius(&style_Main_ddlist_launch_dart_2_extra_list_scrollbar_default, 3);
	lv_style_set_bg_opa(&style_Main_ddlist_launch_dart_2_extra_list_scrollbar_default, 255);
	lv_style_set_bg_color(&style_Main_ddlist_launch_dart_2_extra_list_scrollbar_default, lv_color_hex(0x5d5d5d));
	lv_style_set_bg_grad_dir(&style_Main_ddlist_launch_dart_2_extra_list_scrollbar_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(lv_dropdown_get_list(ui->Main_ddlist_launch_dart_2), &style_Main_ddlist_launch_dart_2_extra_list_scrollbar_default, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_distance_X
	ui->Main_spinbox_distance_X = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_distance_X, 666, 296);
	lv_obj_set_width(ui->Main_spinbox_distance_X, 180);
	lv_obj_set_height(ui->Main_spinbox_distance_X, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_distance_X, 6, 4);
	lv_spinbox_set_range(ui->Main_spinbox_distance_X, -999999, 999999);
	lv_coord_t Main_spinbox_distance_X_h = lv_obj_get_height(ui->Main_spinbox_distance_X);
	ui->Main_spinbox_distance_X_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_distance_X_btn, Main_spinbox_distance_X_h, Main_spinbox_distance_X_h);
	lv_obj_align_to(ui->Main_spinbox_distance_X_btn, ui->Main_spinbox_distance_X, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_distance_X_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_distance_X_btn, lv_Main_spinbox_distance_X_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_distance_X_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_distance_X_btn_minus, Main_spinbox_distance_X_h, Main_spinbox_distance_X_h);
	lv_obj_align_to(ui->Main_spinbox_distance_X_btn_minus, ui->Main_spinbox_distance_X, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_distance_X_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_distance_X_btn_minus, lv_Main_spinbox_distance_X_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_distance_X, 666, 296);

	// Write style for Main_spinbox_distance_X, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_distance_X, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_distance_X, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_distance_X, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_distance_X, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_distance_X, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_distance_X, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_distance_X, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_distance_X, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_distance_X, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_distance_X, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_distance_X, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_distance_X, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_distance_X, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_distance_X, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_distance_X, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_distance_X, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_distance_X, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_distance_X, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_distance_X, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_distance_X, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_distance_X, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_distance_X, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_distance_X, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_distance_X, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_distance_X_extra_btns_main_default
	static lv_style_t style_Main_spinbox_distance_X_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_distance_X_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_distance_X_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_distance_X_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_distance_X_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_distance_X_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_distance_X_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_distance_X_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_distance_X_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_distance_X_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_distance_X_btn, &style_Main_spinbox_distance_X_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_distance_X_btn_minus, &style_Main_spinbox_distance_X_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_initial_velocity
	ui->Main_spinbox_initial_velocity = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_initial_velocity, 665, 397);
	lv_obj_set_width(ui->Main_spinbox_initial_velocity, 180);
	lv_obj_set_height(ui->Main_spinbox_initial_velocity, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_initial_velocity, 6, 4);
	lv_spinbox_set_range(ui->Main_spinbox_initial_velocity, -999999, 999999);
	lv_coord_t Main_spinbox_initial_velocity_h = lv_obj_get_height(ui->Main_spinbox_initial_velocity);
	ui->Main_spinbox_initial_velocity_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_initial_velocity_btn, Main_spinbox_initial_velocity_h, Main_spinbox_initial_velocity_h);
	lv_obj_align_to(ui->Main_spinbox_initial_velocity_btn, ui->Main_spinbox_initial_velocity, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_initial_velocity_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_initial_velocity_btn, lv_Main_spinbox_initial_velocity_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_initial_velocity_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_initial_velocity_btn_minus, Main_spinbox_initial_velocity_h, Main_spinbox_initial_velocity_h);
	lv_obj_align_to(ui->Main_spinbox_initial_velocity_btn_minus, ui->Main_spinbox_initial_velocity, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_initial_velocity_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_initial_velocity_btn_minus, lv_Main_spinbox_initial_velocity_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_initial_velocity, 665, 397);

	// Write style for Main_spinbox_initial_velocity, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_initial_velocity, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_initial_velocity, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_initial_velocity, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_initial_velocity, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_initial_velocity, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_initial_velocity, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_initial_velocity, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_initial_velocity, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_initial_velocity, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_initial_velocity, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_initial_velocity, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_initial_velocity, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_initial_velocity, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_initial_velocity, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_initial_velocity, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_initial_velocity, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_initial_velocity, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_initial_velocity, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_initial_velocity, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_initial_velocity, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_initial_velocity, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_initial_velocity, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_initial_velocity, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_initial_velocity, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_initial_velocity_extra_btns_main_default
	static lv_style_t style_Main_spinbox_initial_velocity_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_initial_velocity_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_initial_velocity_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_initial_velocity_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_initial_velocity_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_initial_velocity_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_initial_velocity_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_initial_velocity_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_initial_velocity_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_initial_velocity_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_initial_velocity_btn, &style_Main_spinbox_initial_velocity_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_initial_velocity_btn_minus, &style_Main_spinbox_initial_velocity_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_41
	ui->Main_label_41 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_41, "目标初速度v");
	lv_label_set_long_mode(ui->Main_label_41, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_41, 607, 357);
	lv_obj_set_size(ui->Main_label_41, 182, 34);

	// Write style for Main_label_41, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_41, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_41, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_41, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_41, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_41, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_41, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_41, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_41, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_41, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_41, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_41, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_41, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_41, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_41, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_sw_auto_yaw_calibration
	ui->Main_sw_auto_yaw_calibration = lv_switch_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_sw_auto_yaw_calibration, 477, 418);
	lv_obj_set_size(ui->Main_sw_auto_yaw_calibration, 64, 30);

	// Write style for Main_sw_auto_yaw_calibration, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_yaw_calibration, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_yaw_calibration, lv_color_hex(0x41485a), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_yaw_calibration, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_sw_auto_yaw_calibration, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_sw_auto_yaw_calibration, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_sw_auto_yaw_calibration, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_sw_auto_yaw_calibration, Part: LV_PART_INDICATOR, State: LV_STATE_CHECKED.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_yaw_calibration, 68, LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_yaw_calibration, lv_color_hex(0x00a1b5), LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_yaw_calibration, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_border_width(ui->Main_sw_auto_yaw_calibration, 0, LV_PART_INDICATOR | LV_STATE_CHECKED);

	// Write style for Main_sw_auto_yaw_calibration, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_yaw_calibration, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_yaw_calibration, lv_color_hex(0x00a1b5), LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_yaw_calibration, LV_GRAD_DIR_NONE, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_sw_auto_yaw_calibration, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_sw_auto_yaw_calibration, 10, LV_PART_KNOB | LV_STATE_DEFAULT);

	// Write codes Main_btn_reload_params
	ui->Main_btn_reload_params = lv_btn_create(ui->Main_MainView_tab_1);
	ui->Main_btn_reload_params_label = lv_label_create(ui->Main_btn_reload_params);
	lv_label_set_text(ui->Main_btn_reload_params_label, "恢复默认");
	lv_label_set_long_mode(ui->Main_btn_reload_params_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_reload_params_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_reload_params, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_reload_params_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_reload_params, 1861, 40);
	lv_obj_set_size(ui->Main_btn_reload_params, 152, 409);

	// Write style for Main_btn_reload_params, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_reload_params, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_reload_params, lv_color_hex(0xb66d1b), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_reload_params, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_reload_params, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_reload_params, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_reload_params, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_reload_params, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_reload_params, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_reload_params, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_reload_params, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_slot4_fw_offset
	ui->Main_spinbox_slot4_fw_offset = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_slot4_fw_offset, 1571, 395);
	lv_obj_set_width(ui->Main_spinbox_slot4_fw_offset, 180);
	lv_obj_set_height(ui->Main_spinbox_slot4_fw_offset, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_slot4_fw_offset, 4, 4);
	lv_spinbox_set_range(ui->Main_spinbox_slot4_fw_offset, -9999, 9999);
	lv_coord_t Main_spinbox_slot4_fw_offset_h = lv_obj_get_height(ui->Main_spinbox_slot4_fw_offset);
	ui->Main_spinbox_slot4_fw_offset_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_slot4_fw_offset_btn, Main_spinbox_slot4_fw_offset_h, Main_spinbox_slot4_fw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_slot4_fw_offset_btn, ui->Main_spinbox_slot4_fw_offset, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_slot4_fw_offset_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_slot4_fw_offset_btn, lv_Main_spinbox_slot4_fw_offset_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_slot4_fw_offset_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_slot4_fw_offset_btn_minus, Main_spinbox_slot4_fw_offset_h, Main_spinbox_slot4_fw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_slot4_fw_offset_btn_minus, ui->Main_spinbox_slot4_fw_offset, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_slot4_fw_offset_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_slot4_fw_offset_btn_minus, lv_Main_spinbox_slot4_fw_offset_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_slot4_fw_offset, 1571, 395);

	// Write style for Main_spinbox_slot4_fw_offset, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_slot4_fw_offset, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_slot4_fw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_slot4_fw_offset, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_slot4_fw_offset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_slot4_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_slot4_fw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_slot4_fw_offset, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_slot4_fw_offset, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_slot4_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_slot4_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_slot4_fw_offset, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_slot4_fw_offset, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_slot4_fw_offset, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_slot4_fw_offset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_slot4_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_slot4_fw_offset, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_slot4_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_slot4_fw_offset, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_slot4_fw_offset, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_slot4_fw_offset, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_slot4_fw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_slot4_fw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_slot4_fw_offset, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_slot4_fw_offset, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_slot4_fw_offset_extra_btns_main_default
	static lv_style_t style_Main_spinbox_slot4_fw_offset_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_slot4_fw_offset_btn, &style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_slot4_fw_offset_btn_minus, &style_Main_spinbox_slot4_fw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_55
	ui->Main_label_55 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_55, "槽位4偏移速度");
	lv_label_set_long_mode(ui->Main_label_55, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_55, 1511, 356);
	lv_obj_set_size(ui->Main_label_55, 182, 34);

	// Write style for Main_label_55, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_55, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_55, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_55, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_55, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_55, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_55, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_55, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_55, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_55, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_55, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_55, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_55, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_55, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_55, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_slot3_fw_offset
	ui->Main_spinbox_slot3_fw_offset = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_slot3_fw_offset, 1571, 290);
	lv_obj_set_width(ui->Main_spinbox_slot3_fw_offset, 180);
	lv_obj_set_height(ui->Main_spinbox_slot3_fw_offset, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_slot3_fw_offset, 4, 4);
	lv_spinbox_set_range(ui->Main_spinbox_slot3_fw_offset, -9999, 9999);
	lv_coord_t Main_spinbox_slot3_fw_offset_h = lv_obj_get_height(ui->Main_spinbox_slot3_fw_offset);
	ui->Main_spinbox_slot3_fw_offset_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_slot3_fw_offset_btn, Main_spinbox_slot3_fw_offset_h, Main_spinbox_slot3_fw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_slot3_fw_offset_btn, ui->Main_spinbox_slot3_fw_offset, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_slot3_fw_offset_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_slot3_fw_offset_btn, lv_Main_spinbox_slot3_fw_offset_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_slot3_fw_offset_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_slot3_fw_offset_btn_minus, Main_spinbox_slot3_fw_offset_h, Main_spinbox_slot3_fw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_slot3_fw_offset_btn_minus, ui->Main_spinbox_slot3_fw_offset, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_slot3_fw_offset_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_slot3_fw_offset_btn_minus, lv_Main_spinbox_slot3_fw_offset_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_slot3_fw_offset, 1571, 290);

	// Write style for Main_spinbox_slot3_fw_offset, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_slot3_fw_offset, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_slot3_fw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_slot3_fw_offset, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_slot3_fw_offset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_slot3_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_slot3_fw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_slot3_fw_offset, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_slot3_fw_offset, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_slot3_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_slot3_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_slot3_fw_offset, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_slot3_fw_offset, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_slot3_fw_offset, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_slot3_fw_offset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_slot3_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_slot3_fw_offset, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_slot3_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_slot3_fw_offset, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_slot3_fw_offset, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_slot3_fw_offset, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_slot3_fw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_slot3_fw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_slot3_fw_offset, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_slot3_fw_offset, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_slot3_fw_offset_extra_btns_main_default
	static lv_style_t style_Main_spinbox_slot3_fw_offset_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_slot3_fw_offset_btn, &style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_slot3_fw_offset_btn_minus, &style_Main_spinbox_slot3_fw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_54
	ui->Main_label_54 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_54, "槽位3偏移速度");
	lv_label_set_long_mode(ui->Main_label_54, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_54, 1511, 251);
	lv_obj_set_size(ui->Main_label_54, 182, 34);

	// Write style for Main_label_54, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_54, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_54, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_54, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_54, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_54, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_54, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_54, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_54, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_54, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_54, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_54, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_54, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_54, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_54, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_slot2_fw_offset
	ui->Main_spinbox_slot2_fw_offset = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_slot2_fw_offset, 1571, 185);
	lv_obj_set_width(ui->Main_spinbox_slot2_fw_offset, 180);
	lv_obj_set_height(ui->Main_spinbox_slot2_fw_offset, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_slot2_fw_offset, 4, 4);
	lv_spinbox_set_range(ui->Main_spinbox_slot2_fw_offset, -9999, 9999);
	lv_coord_t Main_spinbox_slot2_fw_offset_h = lv_obj_get_height(ui->Main_spinbox_slot2_fw_offset);
	ui->Main_spinbox_slot2_fw_offset_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_slot2_fw_offset_btn, Main_spinbox_slot2_fw_offset_h, Main_spinbox_slot2_fw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_slot2_fw_offset_btn, ui->Main_spinbox_slot2_fw_offset, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_slot2_fw_offset_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_slot2_fw_offset_btn, lv_Main_spinbox_slot2_fw_offset_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_slot2_fw_offset_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_slot2_fw_offset_btn_minus, Main_spinbox_slot2_fw_offset_h, Main_spinbox_slot2_fw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_slot2_fw_offset_btn_minus, ui->Main_spinbox_slot2_fw_offset, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_slot2_fw_offset_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_slot2_fw_offset_btn_minus, lv_Main_spinbox_slot2_fw_offset_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_slot2_fw_offset, 1571, 185);

	// Write style for Main_spinbox_slot2_fw_offset, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_slot2_fw_offset, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_slot2_fw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_slot2_fw_offset, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_slot2_fw_offset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_slot2_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_slot2_fw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_slot2_fw_offset, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_slot2_fw_offset, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_slot2_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_slot2_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_slot2_fw_offset, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_slot2_fw_offset, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_slot2_fw_offset, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_slot2_fw_offset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_slot2_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_slot2_fw_offset, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_slot2_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_slot2_fw_offset, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_slot2_fw_offset, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_slot2_fw_offset, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_slot2_fw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_slot2_fw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_slot2_fw_offset, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_slot2_fw_offset, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_slot2_fw_offset_extra_btns_main_default
	static lv_style_t style_Main_spinbox_slot2_fw_offset_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_slot2_fw_offset_btn, &style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_slot2_fw_offset_btn_minus, &style_Main_spinbox_slot2_fw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_53
	ui->Main_label_53 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_53, "槽位2偏移速度");
	lv_label_set_long_mode(ui->Main_label_53, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_53, 1511, 143);
	lv_obj_set_size(ui->Main_label_53, 182, 37);

	// Write style for Main_label_53, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_53, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_53, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_53, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_53, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_53, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_53, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_53, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_53, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_53, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_53, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_53, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_53, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_53, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_53, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_slot1_fw_offset
	ui->Main_spinbox_slot1_fw_offset = lv_spinbox_create(ui->Main_MainView_tab_1);
	lv_obj_set_pos(ui->Main_spinbox_slot1_fw_offset, 1571, 80);
	lv_obj_set_width(ui->Main_spinbox_slot1_fw_offset, 180);
	lv_obj_set_height(ui->Main_spinbox_slot1_fw_offset, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_slot1_fw_offset, 4, 4);
	lv_spinbox_set_range(ui->Main_spinbox_slot1_fw_offset, -9999, 9999);
	lv_coord_t Main_spinbox_slot1_fw_offset_h = lv_obj_get_height(ui->Main_spinbox_slot1_fw_offset);
	ui->Main_spinbox_slot1_fw_offset_btn = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_slot1_fw_offset_btn, Main_spinbox_slot1_fw_offset_h, Main_spinbox_slot1_fw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_slot1_fw_offset_btn, ui->Main_spinbox_slot1_fw_offset, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_slot1_fw_offset_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_slot1_fw_offset_btn, lv_Main_spinbox_slot1_fw_offset_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_slot1_fw_offset_btn_minus = lv_btn_create(ui->Main_MainView_tab_1);
	lv_obj_set_size(ui->Main_spinbox_slot1_fw_offset_btn_minus, Main_spinbox_slot1_fw_offset_h, Main_spinbox_slot1_fw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_slot1_fw_offset_btn_minus, ui->Main_spinbox_slot1_fw_offset, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_slot1_fw_offset_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_slot1_fw_offset_btn_minus, lv_Main_spinbox_slot1_fw_offset_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_slot1_fw_offset, 1571, 79);

	// Write style for Main_spinbox_slot1_fw_offset, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_slot1_fw_offset, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_slot1_fw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_slot1_fw_offset, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_slot1_fw_offset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_slot1_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_slot1_fw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_slot1_fw_offset, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_slot1_fw_offset, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_slot1_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_slot1_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_slot1_fw_offset, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_slot1_fw_offset, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_slot1_fw_offset, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_slot1_fw_offset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_slot1_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_slot1_fw_offset, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_slot1_fw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_slot1_fw_offset, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_slot1_fw_offset, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_slot1_fw_offset, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_slot1_fw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_slot1_fw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_slot1_fw_offset, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_slot1_fw_offset, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_slot1_fw_offset_extra_btns_main_default
	static lv_style_t style_Main_spinbox_slot1_fw_offset_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_slot1_fw_offset_btn, &style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_slot1_fw_offset_btn_minus, &style_Main_spinbox_slot1_fw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_52
	ui->Main_label_52 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_52, "槽位1偏移速度");
	lv_label_set_long_mode(ui->Main_label_52, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_52, 1511, 35);
	lv_obj_set_size(ui->Main_label_52, 182, 37);

	// Write style for Main_label_52, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_52, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_52, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_52, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_52, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_52, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_52, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_52, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_52, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_52, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_52, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_52, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_52, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_52, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_52, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_btn_restart
	ui->Main_btn_restart = lv_btn_create(ui->Main_MainView_tab_1);
	ui->Main_btn_restart_label = lv_label_create(ui->Main_btn_restart);
	lv_label_set_text(ui->Main_btn_restart_label, "重启服务");
	lv_label_set_long_mode(ui->Main_btn_restart_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_restart_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_restart, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_restart_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_restart, 2176, 40);
	lv_obj_set_size(ui->Main_btn_restart, 152, 177);

	// Write style for Main_btn_restart, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_restart, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_restart, lv_color_hex(0x08765a), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_restart, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_restart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_restart, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_restart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_restart, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_restart, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_restart, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_restart, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_btn_shutdown
	ui->Main_btn_shutdown = lv_btn_create(ui->Main_MainView_tab_1);
	ui->Main_btn_shutdown_label = lv_label_create(ui->Main_btn_shutdown);
	lv_label_set_text(ui->Main_btn_shutdown_label, "关机");
	lv_label_set_long_mode(ui->Main_btn_shutdown_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_shutdown_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_shutdown, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_shutdown_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_shutdown, 2176, 269);
	lv_obj_set_size(ui->Main_btn_shutdown, 152, 177);

	// Write style for Main_btn_shutdown, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_shutdown, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_shutdown, lv_color_hex(0xff3554), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_shutdown, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_shutdown, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_shutdown, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_shutdown, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_shutdown, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_shutdown, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_shutdown, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_shutdown, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_56
	ui->Main_label_56 = lv_label_create(ui->Main_MainView_tab_1);
	lv_label_set_text(ui->Main_label_56, "" LV_SYMBOL_WARNING " 谨慎操作");
	lv_label_set_long_mode(ui->Main_label_56, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_56, 2088, 127);
	lv_obj_set_size(ui->Main_label_56, 28, 295);

	// Write style for Main_label_56, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_56, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_56, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_56, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_56, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_56, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_56, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_56, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_56, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_56, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_56, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_56, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_56, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_56, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_56, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_btn_hotspot
	ui->Main_btn_hotspot = lv_btn_create(ui->Main_MainView_tab_1);
	ui->Main_btn_hotspot_label = lv_label_create(ui->Main_btn_hotspot);
	lv_label_set_text(ui->Main_btn_hotspot_label, "连接热点");
	lv_label_set_long_mode(ui->Main_btn_hotspot_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_hotspot_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_hotspot, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_hotspot_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_hotspot, 2375, 40);
	lv_obj_set_size(ui->Main_btn_hotspot, 152, 177);

	// Write style for Main_btn_hotspot, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_hotspot, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_hotspot, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_hotspot, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_hotspot, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_hotspot, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_hotspot, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_hotspot, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_hotspot, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_hotspot, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_hotspot, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_btn_gnome
	ui->Main_btn_gnome = lv_btn_create(ui->Main_MainView_tab_1);
	ui->Main_btn_gnome_label = lv_label_create(ui->Main_btn_gnome);
	lv_label_set_text(ui->Main_btn_gnome_label, "Gnome");
	lv_label_set_long_mode(ui->Main_btn_gnome_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_gnome_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_gnome, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_gnome_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_gnome, 2375, 269);
	lv_obj_set_size(ui->Main_btn_gnome, 152, 177);

	// Write style for Main_btn_gnome, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_gnome, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_gnome, lv_color_hex(0x861078), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_gnome, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_gnome, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_gnome, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_gnome, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_gnome, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_gnome, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_gnome, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_gnome, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes 飞镖
	ui->Main_MainView_tab_2 = lv_tabview_add_tab(ui->Main_MainView, "飞镖");
	lv_obj_t *Main_MainView_tab_2_label = lv_label_create(ui->Main_MainView_tab_2);
	lv_label_set_text(Main_MainView_tab_2_label, "");

	// Write codes Main_list_darts
	ui->Main_list_darts = lv_list_create(ui->Main_MainView_tab_2);
	lv_obj_set_pos(ui->Main_list_darts, 32, 53);
	lv_obj_set_size(ui->Main_list_darts, 251, 418);
	lv_obj_set_scrollbar_mode(ui->Main_list_darts, LV_SCROLLBAR_MODE_AUTO);

	// Write style state: LV_STATE_DEFAULT for &style_Main_list_darts_main_main_default
	static lv_style_t style_Main_list_darts_main_main_default;
	ui_init_style(&style_Main_list_darts_main_main_default);

	lv_style_set_pad_top(&style_Main_list_darts_main_main_default, 10);
	lv_style_set_pad_left(&style_Main_list_darts_main_main_default, 10);
	lv_style_set_pad_right(&style_Main_list_darts_main_main_default, 10);
	lv_style_set_pad_bottom(&style_Main_list_darts_main_main_default, 10);
	lv_style_set_bg_opa(&style_Main_list_darts_main_main_default, 255);
	lv_style_set_bg_color(&style_Main_list_darts_main_main_default, lv_color_hex(0xf1f1f1));
	lv_style_set_bg_grad_dir(&style_Main_list_darts_main_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_list_darts_main_main_default, 1);
	lv_style_set_border_opa(&style_Main_list_darts_main_main_default, 255);
	lv_style_set_border_color(&style_Main_list_darts_main_main_default, lv_color_hex(0xe1e6ee));
	lv_style_set_border_side(&style_Main_list_darts_main_main_default, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_list_darts_main_main_default, 10);
	lv_style_set_shadow_width(&style_Main_list_darts_main_main_default, 0);
	lv_obj_add_style(ui->Main_list_darts, &style_Main_list_darts_main_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_FOCUSED for &style_Main_list_darts_main_main_focused
	static lv_style_t style_Main_list_darts_main_main_focused;
	ui_init_style(&style_Main_list_darts_main_main_focused);

	lv_style_set_pad_top(&style_Main_list_darts_main_main_focused, 10);
	lv_style_set_pad_left(&style_Main_list_darts_main_main_focused, 10);
	lv_style_set_pad_right(&style_Main_list_darts_main_main_focused, 10);
	lv_style_set_pad_bottom(&style_Main_list_darts_main_main_focused, 10);
	lv_style_set_bg_opa(&style_Main_list_darts_main_main_focused, 255);
	lv_style_set_bg_color(&style_Main_list_darts_main_main_focused, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_list_darts_main_main_focused, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_list_darts_main_main_focused, 1);
	lv_style_set_border_opa(&style_Main_list_darts_main_main_focused, 255);
	lv_style_set_border_color(&style_Main_list_darts_main_main_focused, lv_color_hex(0xe1e6ee));
	lv_style_set_border_side(&style_Main_list_darts_main_main_focused, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_list_darts_main_main_focused, 10);
	lv_style_set_shadow_width(&style_Main_list_darts_main_main_focused, 0);
	lv_obj_add_style(ui->Main_list_darts, &style_Main_list_darts_main_main_focused, LV_PART_MAIN | LV_STATE_FOCUSED);

	// Write style state: LV_STATE_DISABLED for &style_Main_list_darts_main_main_disabled
	static lv_style_t style_Main_list_darts_main_main_disabled;
	ui_init_style(&style_Main_list_darts_main_main_disabled);

	lv_style_set_pad_top(&style_Main_list_darts_main_main_disabled, 10);
	lv_style_set_pad_left(&style_Main_list_darts_main_main_disabled, 10);
	lv_style_set_pad_right(&style_Main_list_darts_main_main_disabled, 10);
	lv_style_set_pad_bottom(&style_Main_list_darts_main_main_disabled, 10);
	lv_style_set_bg_opa(&style_Main_list_darts_main_main_disabled, 255);
	lv_style_set_bg_color(&style_Main_list_darts_main_main_disabled, lv_color_hex(0xffffff));
	lv_style_set_bg_grad_dir(&style_Main_list_darts_main_main_disabled, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_list_darts_main_main_disabled, 1);
	lv_style_set_border_opa(&style_Main_list_darts_main_main_disabled, 255);
	lv_style_set_border_color(&style_Main_list_darts_main_main_disabled, lv_color_hex(0xe1e6ee));
	lv_style_set_border_side(&style_Main_list_darts_main_main_disabled, LV_BORDER_SIDE_FULL);
	lv_style_set_radius(&style_Main_list_darts_main_main_disabled, 3);
	lv_style_set_shadow_width(&style_Main_list_darts_main_main_disabled, 0);
	lv_obj_add_style(ui->Main_list_darts, &style_Main_list_darts_main_main_disabled, LV_PART_MAIN | LV_STATE_DISABLED);

	// Write style state: LV_STATE_DEFAULT for &style_Main_list_darts_main_scrollbar_default
	static lv_style_t style_Main_list_darts_main_scrollbar_default;
	ui_init_style(&style_Main_list_darts_main_scrollbar_default);

	lv_style_set_radius(&style_Main_list_darts_main_scrollbar_default, 3);
	lv_style_set_bg_opa(&style_Main_list_darts_main_scrollbar_default, 255);
	lv_style_set_bg_color(&style_Main_list_darts_main_scrollbar_default, lv_color_hex(0x252525));
	lv_style_set_bg_grad_dir(&style_Main_list_darts_main_scrollbar_default, LV_GRAD_DIR_NONE);
	lv_obj_add_style(ui->Main_list_darts, &style_Main_list_darts_main_scrollbar_default, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_list_darts_extra_btns_main_default
	static lv_style_t style_Main_list_darts_extra_btns_main_default;
	ui_init_style(&style_Main_list_darts_extra_btns_main_default);

	lv_style_set_pad_top(&style_Main_list_darts_extra_btns_main_default, 15);
	lv_style_set_pad_left(&style_Main_list_darts_extra_btns_main_default, 15);
	lv_style_set_pad_right(&style_Main_list_darts_extra_btns_main_default, 5);
	lv_style_set_pad_bottom(&style_Main_list_darts_extra_btns_main_default, 15);
	lv_style_set_border_width(&style_Main_list_darts_extra_btns_main_default, 0);
	lv_style_set_text_color(&style_Main_list_darts_extra_btns_main_default, lv_color_hex(0x0D3055));
	lv_style_set_text_font(&style_Main_list_darts_extra_btns_main_default, &lv_font_misans_28);
	lv_style_set_text_opa(&style_Main_list_darts_extra_btns_main_default, 255);
	lv_style_set_radius(&style_Main_list_darts_extra_btns_main_default, 0);
	lv_style_set_bg_opa(&style_Main_list_darts_extra_btns_main_default, 0);

	// Write style state: LV_STATE_PRESSED for &style_Main_list_darts_extra_btns_main_pressed
	static lv_style_t style_Main_list_darts_extra_btns_main_pressed;
	ui_init_style(&style_Main_list_darts_extra_btns_main_pressed);

	lv_style_set_pad_top(&style_Main_list_darts_extra_btns_main_pressed, 15);
	lv_style_set_pad_left(&style_Main_list_darts_extra_btns_main_pressed, 15);
	lv_style_set_pad_right(&style_Main_list_darts_extra_btns_main_pressed, 15);
	lv_style_set_pad_bottom(&style_Main_list_darts_extra_btns_main_pressed, 15);
	lv_style_set_border_width(&style_Main_list_darts_extra_btns_main_pressed, 0);
	lv_style_set_radius(&style_Main_list_darts_extra_btns_main_pressed, 20);
	lv_style_set_text_color(&style_Main_list_darts_extra_btns_main_pressed, lv_color_hex(0x0D3055));
	lv_style_set_text_font(&style_Main_list_darts_extra_btns_main_pressed, &lv_font_misans_28);
	lv_style_set_text_opa(&style_Main_list_darts_extra_btns_main_pressed, 255);
	lv_style_set_bg_opa(&style_Main_list_darts_extra_btns_main_pressed, 255);
	lv_style_set_bg_color(&style_Main_list_darts_extra_btns_main_pressed, lv_color_hex(0xc1c1c1));
	lv_style_set_bg_grad_dir(&style_Main_list_darts_extra_btns_main_pressed, LV_GRAD_DIR_NONE);

	// Write style state: LV_STATE_FOCUSED for &style_Main_list_darts_extra_btns_main_focused
	static lv_style_t style_Main_list_darts_extra_btns_main_focused;
	ui_init_style(&style_Main_list_darts_extra_btns_main_focused);

	lv_style_set_pad_top(&style_Main_list_darts_extra_btns_main_focused, 15);
	lv_style_set_pad_left(&style_Main_list_darts_extra_btns_main_focused, 15);
	lv_style_set_pad_right(&style_Main_list_darts_extra_btns_main_focused, 15);
	lv_style_set_pad_bottom(&style_Main_list_darts_extra_btns_main_focused, 15);
	lv_style_set_border_width(&style_Main_list_darts_extra_btns_main_focused, 0);
	lv_style_set_radius(&style_Main_list_darts_extra_btns_main_focused, 20);
	lv_style_set_text_color(&style_Main_list_darts_extra_btns_main_focused, lv_color_hex(0xcadfff));
	lv_style_set_text_font(&style_Main_list_darts_extra_btns_main_focused, &lv_font_misans_28);
	lv_style_set_text_opa(&style_Main_list_darts_extra_btns_main_focused, 255);
	lv_style_set_bg_opa(&style_Main_list_darts_extra_btns_main_focused, 255);
	lv_style_set_bg_color(&style_Main_list_darts_extra_btns_main_focused, lv_color_hex(0x3b3b3b));
	lv_style_set_bg_grad_dir(&style_Main_list_darts_extra_btns_main_focused, LV_GRAD_DIR_NONE);

	// Write style state: LV_STATE_DEFAULT for &style_Main_list_darts_extra_texts_main_default
	static lv_style_t style_Main_list_darts_extra_texts_main_default;
	ui_init_style(&style_Main_list_darts_extra_texts_main_default);

	lv_style_set_pad_top(&style_Main_list_darts_extra_texts_main_default, 5);
	lv_style_set_pad_left(&style_Main_list_darts_extra_texts_main_default, 0);
	lv_style_set_pad_right(&style_Main_list_darts_extra_texts_main_default, 0);
	lv_style_set_pad_bottom(&style_Main_list_darts_extra_texts_main_default, 0);
	lv_style_set_border_width(&style_Main_list_darts_extra_texts_main_default, 0);
	lv_style_set_text_color(&style_Main_list_darts_extra_texts_main_default, lv_color_hex(0x0D3055));
	lv_style_set_text_font(&style_Main_list_darts_extra_texts_main_default, &lv_font_misans_20);
	lv_style_set_text_opa(&style_Main_list_darts_extra_texts_main_default, 255);
	lv_style_set_radius(&style_Main_list_darts_extra_texts_main_default, 3);
	lv_style_set_bg_opa(&style_Main_list_darts_extra_texts_main_default, 0);

	// Write codes Main_label_35
	ui->Main_label_35 = lv_label_create(ui->Main_MainView_tab_2);
	lv_label_set_text(ui->Main_label_35, "飞镖列表");
	lv_label_set_long_mode(ui->Main_label_35, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_35, 54, 5);
	lv_obj_set_size(ui->Main_label_35, 132, 29);

	// Write style for Main_label_35, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_35, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_35, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_35, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_35, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_35, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_35, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_35, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_35, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_35, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_35, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_35, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_35, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_35, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_35, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_cont_dart_param
	ui->Main_cont_dart_param = lv_obj_create(ui->Main_MainView_tab_2);
	lv_obj_set_pos(ui->Main_cont_dart_param, 340, 53);
	lv_obj_set_size(ui->Main_cont_dart_param, 1319, 418);
	lv_obj_set_scrollbar_mode(ui->Main_cont_dart_param, LV_SCROLLBAR_MODE_OFF);

	// Write style for Main_cont_dart_param, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_cont_dart_param, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_cont_dart_param, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_cont_dart_param, lv_color_hex(0x7d7d7d), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_cont_dart_param, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_cont_dart_param, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_cont_dart_param, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_cont_dart_param, lv_color_hex(0xf1f1f1), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_cont_dart_param, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_cont_dart_param, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_cont_dart_param, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_cont_dart_param, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_cont_dart_param, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_cont_dart_param, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_36
	ui->Main_label_36 = lv_label_create(ui->Main_cont_dart_param);
	lv_label_set_text(ui->Main_label_36, "编号");
	lv_label_set_long_mode(ui->Main_label_36, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_36, 45, 21);
	lv_obj_set_size(ui->Main_label_36, 100, 32);

	// Write style for Main_label_36, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_36, lv_color_hex(0x4c4c4c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_36, &lv_font_misans_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_36, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_36, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_36, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_36, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_ta_dart_param_number
	ui->Main_ta_dart_param_number = lv_textarea_create(ui->Main_cont_dart_param);
	lv_textarea_set_text(ui->Main_ta_dart_param_number, "");
	lv_textarea_set_placeholder_text(ui->Main_ta_dart_param_number, "请输入编号...");
	lv_textarea_set_password_bullet(ui->Main_ta_dart_param_number, "*");
	lv_textarea_set_password_mode(ui->Main_ta_dart_param_number, false);
	lv_textarea_set_one_line(ui->Main_ta_dart_param_number, true);
	lv_textarea_set_accepted_chars(ui->Main_ta_dart_param_number, "1234567890-_.");
	lv_textarea_set_max_length(ui->Main_ta_dart_param_number, 32);
#if LV_USE_KEYBOARD != 0 || LV_USE_ZH_KEYBOARD != 0
	lv_obj_add_event_cb(ui->Main_ta_dart_param_number, ta_event_cb, LV_EVENT_ALL, ui->g_kb_Main);
#endif
	lv_obj_set_pos(ui->Main_ta_dart_param_number, 45, 45);
	lv_obj_set_size(ui->Main_ta_dart_param_number, 265, 53);

	// Write style for Main_ta_dart_param_number, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_ta_dart_param_number, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_ta_dart_param_number, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_ta_dart_param_number, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_ta_dart_param_number, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_ta_dart_param_number, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_ta_dart_param_number, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_ta_dart_param_number, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_ta_dart_param_number, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_ta_dart_param_number, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_ta_dart_param_number, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_ta_dart_param_number, lv_color_hex(0xd0d0d0), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_ta_dart_param_number, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_ta_dart_param_number, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_ta_dart_param_number, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_ta_dart_param_number, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_ta_dart_param_number, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_ta_dart_param_number, 6, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_ta_dart_param_number, Part: LV_PART_MAIN, State: LV_STATE_FOCUSED.
	lv_obj_set_style_text_color(ui->Main_ta_dart_param_number, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_font(ui->Main_ta_dart_param_number, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_opa(ui->Main_ta_dart_param_number, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_opa(ui->Main_ta_dart_param_number, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_color(ui->Main_ta_dart_param_number, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ta_dart_param_number, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_width(ui->Main_ta_dart_param_number, 2, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_opa(ui->Main_ta_dart_param_number, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_color(ui->Main_ta_dart_param_number, lv_color_hex(0xe6e6e6), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_side(ui->Main_ta_dart_param_number, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_shadow_width(ui->Main_ta_dart_param_number, 0, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_top(ui->Main_ta_dart_param_number, 10, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_right(ui->Main_ta_dart_param_number, 25, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_left(ui->Main_ta_dart_param_number, 25, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_radius(ui->Main_ta_dart_param_number, 6, LV_PART_MAIN | LV_STATE_FOCUSED);

	// Write style for Main_ta_dart_param_number, Part: LV_PART_SCROLLBAR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_ta_dart_param_number, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_ta_dart_param_number, lv_color_hex(0x5c5c5c), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_ta_dart_param_number, LV_GRAD_DIR_NONE, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_ta_dart_param_number, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_dart_param_yaw_offset
	ui->Main_spinbox_dart_param_yaw_offset = lv_spinbox_create(ui->Main_cont_dart_param);
	lv_obj_set_pos(ui->Main_spinbox_dart_param_yaw_offset, 112, 137);
	lv_obj_set_width(ui->Main_spinbox_dart_param_yaw_offset, 141);
	lv_obj_set_height(ui->Main_spinbox_dart_param_yaw_offset, 60);
	lv_spinbox_set_digit_format(ui->Main_spinbox_dart_param_yaw_offset, 4, 4);
	lv_spinbox_set_range(ui->Main_spinbox_dart_param_yaw_offset, -9999, 9999);
	lv_coord_t Main_spinbox_dart_param_yaw_offset_h = lv_obj_get_height(ui->Main_spinbox_dart_param_yaw_offset);
	ui->Main_spinbox_dart_param_yaw_offset_btn = lv_btn_create(ui->Main_cont_dart_param);
	lv_obj_set_size(ui->Main_spinbox_dart_param_yaw_offset_btn, Main_spinbox_dart_param_yaw_offset_h, Main_spinbox_dart_param_yaw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_dart_param_yaw_offset_btn, ui->Main_spinbox_dart_param_yaw_offset, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_dart_param_yaw_offset_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_dart_param_yaw_offset_btn, lv_Main_spinbox_dart_param_yaw_offset_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_dart_param_yaw_offset_btn_minus = lv_btn_create(ui->Main_cont_dart_param);
	lv_obj_set_size(ui->Main_spinbox_dart_param_yaw_offset_btn_minus, Main_spinbox_dart_param_yaw_offset_h, Main_spinbox_dart_param_yaw_offset_h);
	lv_obj_align_to(ui->Main_spinbox_dart_param_yaw_offset_btn_minus, ui->Main_spinbox_dart_param_yaw_offset, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_dart_param_yaw_offset_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_dart_param_yaw_offset_btn_minus, lv_Main_spinbox_dart_param_yaw_offset_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_dart_param_yaw_offset, 112, 137);

	// Write style for Main_spinbox_dart_param_yaw_offset, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_dart_param_yaw_offset, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_dart_param_yaw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_dart_param_yaw_offset, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_dart_param_yaw_offset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_dart_param_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_dart_param_yaw_offset, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_dart_param_yaw_offset, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_dart_param_yaw_offset, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_dart_param_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_dart_param_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_dart_param_yaw_offset, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_dart_param_yaw_offset, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_dart_param_yaw_offset, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_dart_param_yaw_offset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_dart_param_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_dart_param_yaw_offset, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_dart_param_yaw_offset, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_dart_param_yaw_offset, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_dart_param_yaw_offset, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_dart_param_yaw_offset, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_dart_param_yaw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_dart_param_yaw_offset, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_dart_param_yaw_offset, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_dart_param_yaw_offset, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default
	static lv_style_t style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_dart_param_yaw_offset_btn, &style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_dart_param_yaw_offset_btn_minus, &style_Main_spinbox_dart_param_yaw_offset_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_39
	ui->Main_label_39 = lv_label_create(ui->Main_cont_dart_param);
	lv_label_set_text(ui->Main_label_39, "Yaw轴角度偏置");
	lv_label_set_long_mode(ui->Main_label_39, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_39, 47, 112);
	lv_obj_set_size(ui->Main_label_39, 158, 20);

	// Write style for Main_label_39, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_39, lv_color_hex(0x4c4c4c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_39, &lv_font_misans_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_39, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_39, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_39, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_39, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_btn_dart_param_save
	ui->Main_btn_dart_param_save = lv_btn_create(ui->Main_cont_dart_param);
	ui->Main_btn_dart_param_save_label = lv_label_create(ui->Main_btn_dart_param_save);
	lv_label_set_text(ui->Main_btn_dart_param_save_label, "保存");
	lv_label_set_long_mode(ui->Main_btn_dart_param_save_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_dart_param_save_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_dart_param_save, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_dart_param_save_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_dart_param_save, 42, 345);
	lv_obj_set_size(ui->Main_btn_dart_param_save, 120, 41);

	// Write style for Main_btn_dart_param_save, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_dart_param_save, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_dart_param_save, lv_color_hex(0x009ea9), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_dart_param_save, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_dart_param_save, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_dart_param_save, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_dart_param_save, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_color(ui->Main_btn_dart_param_save, lv_color_hex(0x0d4b3b), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_opa(ui->Main_btn_dart_param_save, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_spread(ui->Main_btn_dart_param_save, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_ofs_x(ui->Main_btn_dart_param_save, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_ofs_y(ui->Main_btn_dart_param_save, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_dart_param_save, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_dart_param_save, &lv_font_misans_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_dart_param_save, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_dart_param_save, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_btn_dart_param_reload
	ui->Main_btn_dart_param_reload = lv_btn_create(ui->Main_cont_dart_param);
	ui->Main_btn_dart_param_reload_label = lv_label_create(ui->Main_btn_dart_param_reload);
	lv_label_set_text(ui->Main_btn_dart_param_reload_label, "重载");
	lv_label_set_long_mode(ui->Main_btn_dart_param_reload_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_dart_param_reload_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_dart_param_reload, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_dart_param_reload_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_dart_param_reload, 191, 345);
	lv_obj_set_size(ui->Main_btn_dart_param_reload, 120, 41);

	// Write style for Main_btn_dart_param_reload, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_dart_param_reload, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_dart_param_reload, lv_color_hex(0xb66d1b), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_dart_param_reload, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_dart_param_reload, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_dart_param_reload, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_dart_param_reload, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_color(ui->Main_btn_dart_param_reload, lv_color_hex(0x0d4b3b), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_opa(ui->Main_btn_dart_param_reload, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_spread(ui->Main_btn_dart_param_reload, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_ofs_x(ui->Main_btn_dart_param_reload, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_ofs_y(ui->Main_btn_dart_param_reload, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_dart_param_reload, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_dart_param_reload, &lv_font_misans_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_dart_param_reload, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_dart_param_reload, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_chart_velocity_distance_curve
	ui->Main_chart_velocity_distance_curve = lv_chart_create(ui->Main_cont_dart_param);
	lv_chart_set_type(ui->Main_chart_velocity_distance_curve, LV_CHART_TYPE_LINE);
	lv_chart_set_div_line_count(ui->Main_chart_velocity_distance_curve, 3, 5);
	lv_chart_set_point_count(ui->Main_chart_velocity_distance_curve, 5);
	lv_chart_set_range(ui->Main_chart_velocity_distance_curve, LV_CHART_AXIS_PRIMARY_Y, 16, 19);
	lv_chart_set_axis_tick(ui->Main_chart_velocity_distance_curve, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 4, 4, true, 40);
	lv_chart_set_range(ui->Main_chart_velocity_distance_curve, LV_CHART_AXIS_SECONDARY_Y, 0, 100);
	lv_chart_set_axis_tick(ui->Main_chart_velocity_distance_curve, LV_CHART_AXIS_PRIMARY_X, 10, 1, 10, 10, true, 40);
	lv_chart_set_zoom_x(ui->Main_chart_velocity_distance_curve, 256);
	lv_chart_set_zoom_y(ui->Main_chart_velocity_distance_curve, 256);
	lv_obj_set_pos(ui->Main_chart_velocity_distance_curve, 371, 53);
	lv_obj_set_size(ui->Main_chart_velocity_distance_curve, 429, 319);
	lv_obj_set_scrollbar_mode(ui->Main_chart_velocity_distance_curve, LV_SCROLLBAR_MODE_OFF);

	// Write style for Main_chart_velocity_distance_curve, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_chart_velocity_distance_curve, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_chart_velocity_distance_curve, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_chart_velocity_distance_curve, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_chart_velocity_distance_curve, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_chart_velocity_distance_curve, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_chart_velocity_distance_curve, lv_color_hex(0xe8e8e8), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_chart_velocity_distance_curve, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_chart_velocity_distance_curve, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_line_width(ui->Main_chart_velocity_distance_curve, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_line_color(ui->Main_chart_velocity_distance_curve, lv_color_hex(0xe8e8e8), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_line_opa(ui->Main_chart_velocity_distance_curve, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_chart_velocity_distance_curve, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_chart_velocity_distance_curve, Part: LV_PART_TICKS, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_chart_velocity_distance_curve, lv_color_hex(0x151212), LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_chart_velocity_distance_curve, &lv_font_misans_12, LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_chart_velocity_distance_curve, 255, LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_line_width(ui->Main_chart_velocity_distance_curve, 2, LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_line_color(ui->Main_chart_velocity_distance_curve, lv_color_hex(0xe8e8e8), LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_line_opa(ui->Main_chart_velocity_distance_curve, 255, LV_PART_TICKS | LV_STATE_DEFAULT);

	// Write codes Main_label_50
	ui->Main_label_50 = lv_label_create(ui->Main_cont_dart_param);
	lv_label_set_text(ui->Main_label_50, "目标速度调试");
	lv_label_set_long_mode(ui->Main_label_50, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_50, 45, 222);
	lv_obj_set_size(ui->Main_label_50, 135, 29);

	// Write style for Main_label_50, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_50, lv_color_hex(0x4c4c4c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_50, &lv_font_misans_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_50, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_50, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_50, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_50, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_chart_vel_fw_vel_curve
	ui->Main_chart_vel_fw_vel_curve = lv_chart_create(ui->Main_cont_dart_param);
	lv_chart_set_type(ui->Main_chart_vel_fw_vel_curve, LV_CHART_TYPE_LINE);
	lv_chart_set_div_line_count(ui->Main_chart_vel_fw_vel_curve, 3, 5);
	lv_chart_set_point_count(ui->Main_chart_vel_fw_vel_curve, 5);
	lv_chart_set_range(ui->Main_chart_vel_fw_vel_curve, LV_CHART_AXIS_PRIMARY_Y, 16, 19);
	lv_chart_set_axis_tick(ui->Main_chart_vel_fw_vel_curve, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 4, 4, true, 40);
	lv_chart_set_range(ui->Main_chart_vel_fw_vel_curve, LV_CHART_AXIS_SECONDARY_Y, 0, 100);
	lv_chart_set_axis_tick(ui->Main_chart_vel_fw_vel_curve, LV_CHART_AXIS_PRIMARY_X, 10, 1, 10, 10, true, 40);
	lv_chart_set_zoom_x(ui->Main_chart_vel_fw_vel_curve, 256);
	lv_chart_set_zoom_y(ui->Main_chart_vel_fw_vel_curve, 256);
	lv_obj_set_pos(ui->Main_chart_vel_fw_vel_curve, 848, 53);
	lv_obj_set_size(ui->Main_chart_vel_fw_vel_curve, 429, 319);
	lv_obj_set_scrollbar_mode(ui->Main_chart_vel_fw_vel_curve, LV_SCROLLBAR_MODE_OFF);

	// Write style for Main_chart_vel_fw_vel_curve, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_chart_vel_fw_vel_curve, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_chart_vel_fw_vel_curve, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_chart_vel_fw_vel_curve, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_chart_vel_fw_vel_curve, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_chart_vel_fw_vel_curve, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_chart_vel_fw_vel_curve, lv_color_hex(0xe8e8e8), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_chart_vel_fw_vel_curve, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_chart_vel_fw_vel_curve, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_line_width(ui->Main_chart_vel_fw_vel_curve, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_line_color(ui->Main_chart_vel_fw_vel_curve, lv_color_hex(0xe8e8e8), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_line_opa(ui->Main_chart_vel_fw_vel_curve, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_chart_vel_fw_vel_curve, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_chart_vel_fw_vel_curve, Part: LV_PART_TICKS, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_chart_vel_fw_vel_curve, lv_color_hex(0x151212), LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_chart_vel_fw_vel_curve, &lv_font_misans_12, LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_chart_vel_fw_vel_curve, 255, LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_line_width(ui->Main_chart_vel_fw_vel_curve, 2, LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_line_color(ui->Main_chart_vel_fw_vel_curve, lv_color_hex(0xe8e8e8), LV_PART_TICKS | LV_STATE_DEFAULT);
	lv_obj_set_style_line_opa(ui->Main_chart_vel_fw_vel_curve, 255, LV_PART_TICKS | LV_STATE_DEFAULT);

	// Write codes Main_ta_target_initial_speed_debug
	ui->Main_ta_target_initial_speed_debug = lv_textarea_create(ui->Main_cont_dart_param);
	lv_textarea_set_text(ui->Main_ta_target_initial_speed_debug, "");
	lv_textarea_set_placeholder_text(ui->Main_ta_target_initial_speed_debug, "...");
	lv_textarea_set_password_bullet(ui->Main_ta_target_initial_speed_debug, "*");
	lv_textarea_set_password_mode(ui->Main_ta_target_initial_speed_debug, false);
	lv_textarea_set_one_line(ui->Main_ta_target_initial_speed_debug, true);
	lv_textarea_set_accepted_chars(ui->Main_ta_target_initial_speed_debug, "1234567890-_.");
	lv_textarea_set_max_length(ui->Main_ta_target_initial_speed_debug, 32);
#if LV_USE_KEYBOARD != 0 || LV_USE_ZH_KEYBOARD != 0
	lv_obj_add_event_cb(ui->Main_ta_target_initial_speed_debug, ta_event_cb, LV_EVENT_ALL, ui->g_kb_Main);
#endif
	lv_obj_set_pos(ui->Main_ta_target_initial_speed_debug, 45, 245);
	lv_obj_set_size(ui->Main_ta_target_initial_speed_debug, 135, 53);

	// Write style for Main_ta_target_initial_speed_debug, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_ta_target_initial_speed_debug, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_ta_target_initial_speed_debug, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_ta_target_initial_speed_debug, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_ta_target_initial_speed_debug, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_ta_target_initial_speed_debug, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_ta_target_initial_speed_debug, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_ta_target_initial_speed_debug, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_ta_target_initial_speed_debug, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_ta_target_initial_speed_debug, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_ta_target_initial_speed_debug, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_ta_target_initial_speed_debug, lv_color_hex(0xd0d0d0), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_ta_target_initial_speed_debug, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_ta_target_initial_speed_debug, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_ta_target_initial_speed_debug, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_ta_target_initial_speed_debug, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_ta_target_initial_speed_debug, 25, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_ta_target_initial_speed_debug, 6, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_ta_target_initial_speed_debug, Part: LV_PART_MAIN, State: LV_STATE_FOCUSED.
	lv_obj_set_style_text_color(ui->Main_ta_target_initial_speed_debug, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_font(ui->Main_ta_target_initial_speed_debug, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_text_opa(ui->Main_ta_target_initial_speed_debug, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_opa(ui->Main_ta_target_initial_speed_debug, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_color(ui->Main_ta_target_initial_speed_debug, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_bg_grad_dir(ui->Main_ta_target_initial_speed_debug, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_width(ui->Main_ta_target_initial_speed_debug, 2, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_opa(ui->Main_ta_target_initial_speed_debug, 255, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_color(ui->Main_ta_target_initial_speed_debug, lv_color_hex(0xe6e6e6), LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_border_side(ui->Main_ta_target_initial_speed_debug, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_shadow_width(ui->Main_ta_target_initial_speed_debug, 0, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_top(ui->Main_ta_target_initial_speed_debug, 10, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_right(ui->Main_ta_target_initial_speed_debug, 25, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_pad_left(ui->Main_ta_target_initial_speed_debug, 25, LV_PART_MAIN | LV_STATE_FOCUSED);
	lv_obj_set_style_radius(ui->Main_ta_target_initial_speed_debug, 6, LV_PART_MAIN | LV_STATE_FOCUSED);

	// Write style for Main_ta_target_initial_speed_debug, Part: LV_PART_SCROLLBAR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_ta_target_initial_speed_debug, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_ta_target_initial_speed_debug, lv_color_hex(0x5c5c5c), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_ta_target_initial_speed_debug, LV_GRAD_DIR_NONE, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_ta_target_initial_speed_debug, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

	// Write codes Main_label_49
	ui->Main_label_49 = lv_label_create(ui->Main_cont_dart_param);
	lv_label_set_text(ui->Main_label_49, "速度-距离打表");
	lv_label_set_long_mode(ui->Main_label_49, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_49, 854, 26);
	lv_obj_set_size(ui->Main_label_49, 158, 20);

	// Write style for Main_label_49, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_49, lv_color_hex(0x4c4c4c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_49, &lv_font_misans_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_49, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_49, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_49, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_49, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_51
	ui->Main_label_51 = lv_label_create(ui->Main_cont_dart_param);
	lv_label_set_text(ui->Main_label_51, "摩擦轮速度");
	lv_label_set_long_mode(ui->Main_label_51, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_51, 195, 222);
	lv_obj_set_size(ui->Main_label_51, 135, 29);

	// Write style for Main_label_51, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_51, lv_color_hex(0x4c4c4c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_51, &lv_font_misans_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_51, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_51, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_51, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_51, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_target_fw_velocity_result
	ui->Main_label_target_fw_velocity_result = lv_label_create(ui->Main_cont_dart_param);
	lv_label_set_text(ui->Main_label_target_fw_velocity_result, "N/A");
	lv_label_set_long_mode(ui->Main_label_target_fw_velocity_result, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_target_fw_velocity_result, 198, 258);
	lv_obj_set_size(ui->Main_label_target_fw_velocity_result, 156, 29);

	// Write style for Main_label_target_fw_velocity_result, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_target_fw_velocity_result, lv_color_hex(0x4c4c4c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_target_fw_velocity_result, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_target_fw_velocity_result, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_target_fw_velocity_result, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_target_fw_velocity_result, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_target_fw_velocity_result, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_48
	ui->Main_label_48 = lv_label_create(ui->Main_cont_dart_param);
	lv_label_set_text(ui->Main_label_48, "初速度-距离曲线");
	lv_label_set_long_mode(ui->Main_label_48, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_48, 371, 26);
	lv_obj_set_size(ui->Main_label_48, 158, 20);

	// Write style for Main_label_48, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_48, lv_color_hex(0x4c4c4c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_48, &lv_font_misans_18, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_48, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_48, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_48, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_48, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_40
	ui->Main_label_40 = lv_label_create(ui->Main_MainView_tab_2);
	lv_label_set_text(ui->Main_label_40, "参数列表");
	lv_label_set_long_mode(ui->Main_label_40, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_40, 388, 5);
	lv_obj_set_size(ui->Main_label_40, 132, 29);

	// Write style for Main_label_40, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_40, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_40, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_40, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_40, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_40, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_40, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_40, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_40, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_40, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_40, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_40, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_40, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_40, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_40, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes 视觉
	ui->Main_MainView_tab_3 = lv_tabview_add_tab(ui->Main_MainView, "视觉");
	lv_obj_t *Main_MainView_tab_3_label = lv_label_create(ui->Main_MainView_tab_3);
	lv_label_set_text(Main_MainView_tab_3_label, "");

	// Write codes Main_canvas_opencv
	ui->Main_canvas_opencv = lv_canvas_create(ui->Main_MainView_tab_3);
	static lv_color_t buf_Main_canvas_opencv[606 * 485 * 4];
	lv_canvas_set_buffer(ui->Main_canvas_opencv, buf_Main_canvas_opencv, 606, 485, LV_IMG_CF_TRUE_COLOR_ALPHA);
	lv_canvas_fill_bg(ui->Main_canvas_opencv, lv_color_hex(0xffffff), 255);
	lv_obj_set_pos(ui->Main_canvas_opencv, 10, 6);
	lv_obj_set_size(ui->Main_canvas_opencv, 606, 485);
	lv_obj_set_scrollbar_mode(ui->Main_canvas_opencv, LV_SCROLLBAR_MODE_OFF);

	// Write codes Main_label_43
	ui->Main_label_43 = lv_label_create(ui->Main_MainView_tab_3);
	lv_label_set_text(ui->Main_label_43, "Yaw轴主角度");
	lv_label_set_long_mode(ui->Main_label_43, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_43, 646, -7);
	lv_obj_set_size(ui->Main_label_43, 182, 37);

	// Write style for Main_label_43, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_43, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_43, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_43, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_43, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_43, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_43, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_43, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_43, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_43, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_43, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_43, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_43, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_43, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_43, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_42
	ui->Main_label_42 = lv_label_create(ui->Main_MainView_tab_3);
	lv_label_set_text(ui->Main_label_42, "Yaw轴偏移角度");
	lv_label_set_long_mode(ui->Main_label_42, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_42, 645, 106);
	lv_obj_set_size(ui->Main_label_42, 182, 37);

	// Write style for Main_label_42, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_42, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_42, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_42, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_42, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_42, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_42, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_42, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_42, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_42, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_42, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_42, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_42, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_42, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_42, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_yaw_angle_offset_cv
	ui->Main_spinbox_yaw_angle_offset_cv = lv_spinbox_create(ui->Main_MainView_tab_3);
	lv_obj_set_pos(ui->Main_spinbox_yaw_angle_offset_cv, 704, 147);
	lv_obj_set_width(ui->Main_spinbox_yaw_angle_offset_cv, 180);
	lv_obj_set_height(ui->Main_spinbox_yaw_angle_offset_cv, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_yaw_angle_offset_cv, 6, 4);
	lv_spinbox_set_range(ui->Main_spinbox_yaw_angle_offset_cv, -999999, 999999);
	lv_coord_t Main_spinbox_yaw_angle_offset_cv_h = lv_obj_get_height(ui->Main_spinbox_yaw_angle_offset_cv);
	ui->Main_spinbox_yaw_angle_offset_cv_btn = lv_btn_create(ui->Main_MainView_tab_3);
	lv_obj_set_size(ui->Main_spinbox_yaw_angle_offset_cv_btn, Main_spinbox_yaw_angle_offset_cv_h, Main_spinbox_yaw_angle_offset_cv_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_angle_offset_cv_btn, ui->Main_spinbox_yaw_angle_offset_cv, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_angle_offset_cv_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_angle_offset_cv_btn, lv_Main_spinbox_yaw_angle_offset_cv_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_yaw_angle_offset_cv_btn_minus = lv_btn_create(ui->Main_MainView_tab_3);
	lv_obj_set_size(ui->Main_spinbox_yaw_angle_offset_cv_btn_minus, Main_spinbox_yaw_angle_offset_cv_h, Main_spinbox_yaw_angle_offset_cv_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_angle_offset_cv_btn_minus, ui->Main_spinbox_yaw_angle_offset_cv, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_angle_offset_cv_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_angle_offset_cv_btn_minus, lv_Main_spinbox_yaw_angle_offset_cv_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_yaw_angle_offset_cv, 704, 147);

	// Write style for Main_spinbox_yaw_angle_offset_cv, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_angle_offset_cv, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_angle_offset_cv, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_angle_offset_cv, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_yaw_angle_offset_cv, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_yaw_angle_offset_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_yaw_angle_offset_cv, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_yaw_angle_offset_cv, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_yaw_angle_offset_cv, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_yaw_angle_offset_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_yaw_angle_offset_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_yaw_angle_offset_cv, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_angle_offset_cv, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_angle_offset_cv, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_angle_offset_cv, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_yaw_angle_offset_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_yaw_angle_offset_cv, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_yaw_angle_offset_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_yaw_angle_offset_cv, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_angle_offset_cv, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_angle_offset_cv, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_angle_offset_cv, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_angle_offset_cv, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_angle_offset_cv, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_angle_offset_cv, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default
	static lv_style_t style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_yaw_angle_offset_cv_btn, &style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_yaw_angle_offset_cv_btn_minus, &style_Main_spinbox_yaw_angle_offset_cv_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_yaw_angle_cv
	ui->Main_spinbox_yaw_angle_cv = lv_spinbox_create(ui->Main_MainView_tab_3);
	lv_obj_set_pos(ui->Main_spinbox_yaw_angle_cv, 704, 35);
	lv_obj_set_width(ui->Main_spinbox_yaw_angle_cv, 180);
	lv_obj_set_height(ui->Main_spinbox_yaw_angle_cv, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_yaw_angle_cv, 6, 4);
	lv_spinbox_set_range(ui->Main_spinbox_yaw_angle_cv, -999999, 999999);
	lv_coord_t Main_spinbox_yaw_angle_cv_h = lv_obj_get_height(ui->Main_spinbox_yaw_angle_cv);
	ui->Main_spinbox_yaw_angle_cv_btn = lv_btn_create(ui->Main_MainView_tab_3);
	lv_obj_set_size(ui->Main_spinbox_yaw_angle_cv_btn, Main_spinbox_yaw_angle_cv_h, Main_spinbox_yaw_angle_cv_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_angle_cv_btn, ui->Main_spinbox_yaw_angle_cv, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_angle_cv_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_angle_cv_btn, lv_Main_spinbox_yaw_angle_cv_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_yaw_angle_cv_btn_minus = lv_btn_create(ui->Main_MainView_tab_3);
	lv_obj_set_size(ui->Main_spinbox_yaw_angle_cv_btn_minus, Main_spinbox_yaw_angle_cv_h, Main_spinbox_yaw_angle_cv_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_angle_cv_btn_minus, ui->Main_spinbox_yaw_angle_cv, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_angle_cv_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_angle_cv_btn_minus, lv_Main_spinbox_yaw_angle_cv_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_yaw_angle_cv, 704, 35);

	// Write style for Main_spinbox_yaw_angle_cv, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_angle_cv, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_angle_cv, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_angle_cv, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_yaw_angle_cv, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_yaw_angle_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_yaw_angle_cv, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_yaw_angle_cv, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_yaw_angle_cv, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_yaw_angle_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_yaw_angle_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_yaw_angle_cv, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_angle_cv, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_angle_cv, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_angle_cv, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_yaw_angle_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_yaw_angle_cv, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_yaw_angle_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_yaw_angle_cv, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_angle_cv, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_angle_cv, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_angle_cv, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_angle_cv, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_angle_cv, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_angle_cv, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_yaw_angle_cv_extra_btns_main_default
	static lv_style_t style_Main_spinbox_yaw_angle_cv_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_yaw_angle_cv_btn, &style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_yaw_angle_cv_btn_minus, &style_Main_spinbox_yaw_angle_cv_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_44
	ui->Main_label_44 = lv_label_create(ui->Main_MainView_tab_3);
	lv_label_set_text(ui->Main_label_44, "识别位置");
	lv_label_set_long_mode(ui->Main_label_44, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_44, 648, 446);
	lv_obj_set_size(ui->Main_label_44, 157, 32);

	// Write style for Main_label_44, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_44, lv_color_hex(0xf2f2f2), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_44, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_44, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_44, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_44, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_44, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_yaw_location_cv
	ui->Main_label_yaw_location_cv = lv_label_create(ui->Main_MainView_tab_3);
	lv_label_set_text(ui->Main_label_yaw_location_cv, "N/A");
	lv_label_set_long_mode(ui->Main_label_yaw_location_cv, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_yaw_location_cv, 744, 444);
	lv_obj_set_size(ui->Main_label_yaw_location_cv, 251, 45);

	// Write style for Main_label_yaw_location_cv, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_yaw_location_cv, lv_color_hex(0xa2ceee), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_yaw_location_cv, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_yaw_location_cv, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_yaw_location_cv, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_yaw_location_cv, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_yaw_location_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_spinbox_yaw_calibration_x
	ui->Main_spinbox_yaw_calibration_x = lv_spinbox_create(ui->Main_MainView_tab_3);
	lv_obj_set_pos(ui->Main_spinbox_yaw_calibration_x, 705, 259);
	lv_obj_set_width(ui->Main_spinbox_yaw_calibration_x, 180);
	lv_obj_set_height(ui->Main_spinbox_yaw_calibration_x, 55);
	lv_spinbox_set_digit_format(ui->Main_spinbox_yaw_calibration_x, 6, 4);
	lv_spinbox_set_range(ui->Main_spinbox_yaw_calibration_x, -999999, 999999);
	lv_coord_t Main_spinbox_yaw_calibration_x_h = lv_obj_get_height(ui->Main_spinbox_yaw_calibration_x);
	ui->Main_spinbox_yaw_calibration_x_btn = lv_btn_create(ui->Main_MainView_tab_3);
	lv_obj_set_size(ui->Main_spinbox_yaw_calibration_x_btn, Main_spinbox_yaw_calibration_x_h, Main_spinbox_yaw_calibration_x_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_calibration_x_btn, ui->Main_spinbox_yaw_calibration_x, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_calibration_x_btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_calibration_x_btn, lv_Main_spinbox_yaw_calibration_x_increment_event_cb, LV_EVENT_ALL, NULL);
	ui->Main_spinbox_yaw_calibration_x_btn_minus = lv_btn_create(ui->Main_MainView_tab_3);
	lv_obj_set_size(ui->Main_spinbox_yaw_calibration_x_btn_minus, Main_spinbox_yaw_calibration_x_h, Main_spinbox_yaw_calibration_x_h);
	lv_obj_align_to(ui->Main_spinbox_yaw_calibration_x_btn_minus, ui->Main_spinbox_yaw_calibration_x, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(ui->Main_spinbox_yaw_calibration_x_btn_minus, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(ui->Main_spinbox_yaw_calibration_x_btn_minus, lv_Main_spinbox_yaw_calibration_x_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_set_pos(ui->Main_spinbox_yaw_calibration_x, 705, 259);

	// Write style for Main_spinbox_yaw_calibration_x, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_calibration_x, 238, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_calibration_x, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_calibration_x, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_spinbox_yaw_calibration_x, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Main_spinbox_yaw_calibration_x, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Main_spinbox_yaw_calibration_x, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_side(ui->Main_spinbox_yaw_calibration_x, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_spinbox_yaw_calibration_x, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_spinbox_yaw_calibration_x, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_spinbox_yaw_calibration_x, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_spinbox_yaw_calibration_x, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_calibration_x, lv_color_hex(0x2c2c2c), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_calibration_x, &lv_font_misans_35, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_calibration_x, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_spinbox_yaw_calibration_x, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_spinbox_yaw_calibration_x, 7, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_spinbox_yaw_calibration_x, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_spinbox_yaw_calibration_x, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->Main_spinbox_yaw_calibration_x, lv_color_hex(0xcbeaed), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_spinbox_yaw_calibration_x, &lv_font_misans_35, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_spinbox_yaw_calibration_x, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_spinbox_yaw_calibration_x, 255, LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_spinbox_yaw_calibration_x, lv_color_hex(0x098D6B), LV_PART_CURSOR | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_spinbox_yaw_calibration_x, LV_GRAD_DIR_NONE, LV_PART_CURSOR | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_spinbox_yaw_calibration_x_extra_btns_main_default
	static lv_style_t style_Main_spinbox_yaw_calibration_x_extra_btns_main_default;
	ui_init_style(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default);

	lv_style_set_text_color(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, 255);
	lv_style_set_bg_opa(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, 183);
	lv_style_set_bg_color(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, lv_color_hex(0x00a1b5));
	lv_style_set_bg_grad_dir(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, 0);
	lv_style_set_radius(&style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, 10);
	lv_obj_add_style(ui->Main_spinbox_yaw_calibration_x_btn, &style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_add_style(ui->Main_spinbox_yaw_calibration_x_btn_minus, &style_Main_spinbox_yaw_calibration_x_extra_btns_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_46
	ui->Main_label_46 = lv_label_create(ui->Main_MainView_tab_3);
	lv_label_set_text(ui->Main_label_46, "目标像素点X轴");
	lv_label_set_long_mode(ui->Main_label_46, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_46, 642, 219);
	lv_obj_set_size(ui->Main_label_46, 182, 37);

	// Write style for Main_label_46, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_46, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_46, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_46, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_46, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_46, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_46, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_46, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_46, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_46, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_46, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_46, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_46, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_46, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_46, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_47
	ui->Main_label_47 = lv_label_create(ui->Main_MainView_tab_3);
	lv_label_set_text(ui->Main_label_47, "Auto");
	lv_label_set_long_mode(ui->Main_label_47, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_47, 652, 330);
	lv_obj_set_size(ui->Main_label_47, 62, 29);

	// Write style for Main_label_47, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_47, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_47, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_47, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_47, &lv_font_misans_20, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_47, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_47, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_47, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_47, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_47, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_47, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_47, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_47, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_47, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_47, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_sw_auto_yaw_calibration_cv
	ui->Main_sw_auto_yaw_calibration_cv = lv_switch_create(ui->Main_MainView_tab_3);
	lv_obj_set_pos(ui->Main_sw_auto_yaw_calibration_cv, 718, 334);
	lv_obj_set_size(ui->Main_sw_auto_yaw_calibration_cv, 64, 30);

	// Write style for Main_sw_auto_yaw_calibration_cv, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_yaw_calibration_cv, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_yaw_calibration_cv, lv_color_hex(0x41485a), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_yaw_calibration_cv, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_sw_auto_yaw_calibration_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_sw_auto_yaw_calibration_cv, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_sw_auto_yaw_calibration_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style for Main_sw_auto_yaw_calibration_cv, Part: LV_PART_INDICATOR, State: LV_STATE_CHECKED.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_yaw_calibration_cv, 68, LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_yaw_calibration_cv, lv_color_hex(0x00a1b5), LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_yaw_calibration_cv, LV_GRAD_DIR_NONE, LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_border_width(ui->Main_sw_auto_yaw_calibration_cv, 0, LV_PART_INDICATOR | LV_STATE_CHECKED);

	// Write style for Main_sw_auto_yaw_calibration_cv, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_sw_auto_yaw_calibration_cv, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_sw_auto_yaw_calibration_cv, lv_color_hex(0x00a1b5), LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_sw_auto_yaw_calibration_cv, LV_GRAD_DIR_NONE, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_sw_auto_yaw_calibration_cv, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_sw_auto_yaw_calibration_cv, 10, LV_PART_KNOB | LV_STATE_DEFAULT);

	// Write codes Main_btn_yaw_calibration_cv
	ui->Main_btn_yaw_calibration_cv = lv_btn_create(ui->Main_MainView_tab_3);
	ui->Main_btn_yaw_calibration_cv_label = lv_label_create(ui->Main_btn_yaw_calibration_cv);
	lv_label_set_text(ui->Main_btn_yaw_calibration_cv_label, "标定Yaw");
	lv_label_set_long_mode(ui->Main_btn_yaw_calibration_cv_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Main_btn_yaw_calibration_cv_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Main_btn_yaw_calibration_cv, 0, LV_STATE_DEFAULT);
	lv_obj_set_width(ui->Main_btn_yaw_calibration_cv_label, LV_PCT(100));
	lv_obj_set_pos(ui->Main_btn_yaw_calibration_cv, 646, 374);
	lv_obj_set_size(ui->Main_btn_yaw_calibration_cv, 152, 60);

	// Write style for Main_btn_yaw_calibration_cv, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_btn_yaw_calibration_cv, 183, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_btn_yaw_calibration_cv, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_btn_yaw_calibration_cv, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_btn_yaw_calibration_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_btn_yaw_calibration_cv, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_btn_yaw_calibration_cv, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_btn_yaw_calibration_cv, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_btn_yaw_calibration_cv, &lv_font_misans_24, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_btn_yaw_calibration_cv, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_btn_yaw_calibration_cv, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_state
	ui->Main_label_state = lv_label_create(ui->Main);
	lv_label_set_text(ui->Main_label_state, "Unknown");
	lv_label_set_long_mode(ui->Main_label_state, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_state, 307, 15);
	lv_obj_set_size(ui->Main_label_state, 176, 43);

	// Write style for Main_label_state, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_state, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_state, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_state, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_state, &lv_font_misans_28, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_state, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_state, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_state, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_state, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_state, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_state, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_state, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_state, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_state, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_state, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_3
	ui->Main_label_3 = lv_label_create(ui->Main);
	lv_label_set_text(ui->Main_label_3, "IP");
	lv_label_set_long_mode(ui->Main_label_3, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_3, 671, 15);
	lv_obj_set_size(ui->Main_label_3, 31, 43);

	// Write style for Main_label_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_3, lv_color_hex(0xdcdcdc), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_3, &lv_font_misans_28, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_3, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_3, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_ip
	ui->Main_label_ip = lv_label_create(ui->Main);
	lv_label_set_text(ui->Main_label_ip, "Unavailable");
	lv_label_set_long_mode(ui->Main_label_ip, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_ip, 731, 15);
	lv_obj_set_size(ui->Main_label_ip, 266, 43);

	// Write style for Main_label_ip, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_ip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_ip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_ip, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_ip, &lv_font_misans_28, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_ip, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_ip, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_ip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_ip, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_ip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_ip, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_ip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_ip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_ip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_ip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_title
	ui->Main_label_title = lv_label_create(ui->Main);
	lv_label_set_text(ui->Main_label_title, "3SE飞镖中控");
	lv_label_set_long_mode(ui->Main_label_title, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_title, 110, 15);
	lv_obj_set_size(ui->Main_label_title, 232, 43);

	// Write style for Main_label_title, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_title, lv_color_hex(0xffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_title, &lv_font_misans_28, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_title, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_title, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_title, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_title, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_title, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_voltage
	ui->Main_label_voltage = lv_label_create(ui->Main);
	lv_label_set_text(ui->Main_label_voltage, "N/A");
	lv_label_set_long_mode(ui->Main_label_voltage, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_voltage, 550, 15);
	lv_obj_set_size(ui->Main_label_voltage, 96, 43);

	// Write style for Main_label_voltage, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_voltage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_voltage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_voltage, lv_color_hex(0x00a1b5), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_voltage, &lv_font_misans_28, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_voltage, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_voltage, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_voltage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_voltage, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_voltage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_voltage, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_voltage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_voltage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_voltage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_voltage, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_label_37
	ui->Main_label_37 = lv_label_create(ui->Main);
	lv_label_set_text(ui->Main_label_37, "请输入编号");
	lv_label_set_long_mode(ui->Main_label_37, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Main_label_37, 1134, 366);
	lv_obj_set_size(ui->Main_label_37, 100, 32);
	lv_obj_add_flag(ui->Main_label_37, LV_OBJ_FLAG_HIDDEN);

	// Write style for Main_label_37, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Main_label_37, lv_color_hex(0x0d0d0d), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Main_label_37, &lv_font_misans_26, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Main_label_37, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Main_label_37, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Main_label_37, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_label_37, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write codes Main_msgbox
	static const char *Main_msgbox_btns[] = {""};
	ui->Main_msgbox = lv_msgbox_create(ui->Main, "提示警告", "Yaw轴弹鼓摩擦轮丝杆C板电机裁判遥控器离线", Main_msgbox_btns, false);
	lv_obj_set_size(lv_msgbox_get_btns(ui->Main_msgbox), 0, 0);
	lv_obj_set_pos(ui->Main_msgbox, 24, 24);
	lv_obj_set_size(ui->Main_msgbox, 349, 80);
	lv_obj_add_flag(ui->Main_msgbox, LV_OBJ_FLAG_HIDDEN);

	// Write style for Main_msgbox, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Main_msgbox, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Main_msgbox, lv_color_hex(0x967000), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Main_msgbox, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Main_msgbox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Main_msgbox, 13, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Main_msgbox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_msgbox_extra_title_main_default
	static lv_style_t style_Main_msgbox_extra_title_main_default;
	ui_init_style(&style_Main_msgbox_extra_title_main_default);

	lv_style_set_text_color(&style_Main_msgbox_extra_title_main_default, lv_color_hex(0xf9f3ec));
	lv_style_set_text_font(&style_Main_msgbox_extra_title_main_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_msgbox_extra_title_main_default, 255);
	lv_style_set_text_letter_space(&style_Main_msgbox_extra_title_main_default, 0);
	lv_style_set_text_line_space(&style_Main_msgbox_extra_title_main_default, 0);
	lv_obj_add_style(lv_msgbox_get_title(ui->Main_msgbox), &style_Main_msgbox_extra_title_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_msgbox_extra_content_main_default
	static lv_style_t style_Main_msgbox_extra_content_main_default;
	ui_init_style(&style_Main_msgbox_extra_content_main_default);

	lv_style_set_text_color(&style_Main_msgbox_extra_content_main_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_msgbox_extra_content_main_default, &lv_font_misans_24);
	lv_style_set_text_opa(&style_Main_msgbox_extra_content_main_default, 255);
	lv_style_set_text_letter_space(&style_Main_msgbox_extra_content_main_default, 0);
	lv_style_set_text_line_space(&style_Main_msgbox_extra_content_main_default, 0);
	lv_obj_add_style(lv_msgbox_get_text(ui->Main_msgbox), &style_Main_msgbox_extra_content_main_default, LV_PART_MAIN | LV_STATE_DEFAULT);

	// Write style state: LV_STATE_DEFAULT for &style_Main_msgbox_extra_btns_items_default
	static lv_style_t style_Main_msgbox_extra_btns_items_default;
	ui_init_style(&style_Main_msgbox_extra_btns_items_default);

	lv_style_set_bg_opa(&style_Main_msgbox_extra_btns_items_default, 255);
	lv_style_set_bg_color(&style_Main_msgbox_extra_btns_items_default, lv_color_hex(0x5a6173));
	lv_style_set_bg_grad_dir(&style_Main_msgbox_extra_btns_items_default, LV_GRAD_DIR_NONE);
	lv_style_set_border_width(&style_Main_msgbox_extra_btns_items_default, 0);
	lv_style_set_radius(&style_Main_msgbox_extra_btns_items_default, 10);
	lv_style_set_text_color(&style_Main_msgbox_extra_btns_items_default, lv_color_hex(0xffffff));
	lv_style_set_text_font(&style_Main_msgbox_extra_btns_items_default, &lv_font_misans_12);
	lv_style_set_text_opa(&style_Main_msgbox_extra_btns_items_default, 255);
	lv_obj_add_style(lv_msgbox_get_btns(ui->Main_msgbox), &style_Main_msgbox_extra_btns_items_default, LV_PART_ITEMS | LV_STATE_DEFAULT);

	// The custom code of Main.

	// Update current screen layout.
	lv_obj_update_layout(ui->Main);
}
