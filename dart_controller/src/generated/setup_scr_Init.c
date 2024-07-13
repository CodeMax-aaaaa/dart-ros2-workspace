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



void setup_scr_Init(lv_ui *ui)
{
	//Write codes Init
	ui->Init = lv_obj_create(NULL);
	ui->g_kb_Init = lv_keyboard_create(ui->Init);
	lv_obj_add_event_cb(ui->g_kb_Init, kb_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_add_flag(ui->g_kb_Init, LV_OBJ_FLAG_HIDDEN);
	lv_obj_set_style_text_font(ui->g_kb_Init, &lv_font_misans_20, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_size(ui->Init, 1024, 600);
	lv_obj_set_scrollbar_mode(ui->Init, LV_SCROLLBAR_MODE_OFF);

	//Write style for Init, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Init, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes Init_img_1
	ui->Init_img_1 = lv_img_create(ui->Init);
	lv_obj_add_flag(ui->Init_img_1, LV_OBJ_FLAG_CLICKABLE);
	lv_img_set_src(ui->Init_img_1, &_logo_3SE_alpha_1024x600);
	lv_img_set_pivot(ui->Init_img_1, 50,50);
	lv_img_set_angle(ui->Init_img_1, 0);
	lv_obj_set_pos(ui->Init_img_1, 0, 0);
	lv_obj_set_size(ui->Init_img_1, 1024, 600);

	//Write style for Init_img_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_img_opa(ui->Init_img_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes Init_label_2
	ui->Init_label_2 = lv_label_create(ui->Init);
	lv_label_set_text(ui->Init_label_2, "正在初始化..");
	lv_label_set_long_mode(ui->Init_label_2, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Init_label_2, 56, 115);
	lv_obj_set_size(ui->Init_label_2, 223, 49);

	//Write style for Init_label_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Init_label_2, lv_color_hex(0xededed), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Init_label_2, &lv_font_misans_32, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Init_label_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Init_label_2, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Init_label_2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Init_label_2, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes Init_bar_1
	ui->Init_bar_1 = lv_bar_create(ui->Init);
	lv_obj_set_style_anim_time(ui->Init_bar_1, 200, 0);
	lv_bar_set_mode(ui->Init_bar_1, LV_BAR_MODE_NORMAL);
	lv_bar_set_range(ui->Init_bar_1, 0, 100);
	lv_bar_set_value(ui->Init_bar_1, 50, LV_ANIM_ON);
	lv_obj_set_pos(ui->Init_bar_1, 75, 166);
	lv_obj_set_size(ui->Init_bar_1, 271, 5);

	//Write style for Init_bar_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Init_bar_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Init_bar_1, lv_color_hex(0x393c41), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Init_bar_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Init_bar_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Init_bar_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write style for Init_bar_1, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Init_bar_1, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Init_bar_1, lv_color_hex(0x00a1b5), LV_PART_INDICATOR|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_grad_dir(ui->Init_bar_1, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Init_bar_1, 10, LV_PART_INDICATOR|LV_STATE_DEFAULT);

	//Write codes Init_labelInitLog
	ui->Init_labelInitLog = lv_label_create(ui->Init);
	lv_label_set_text(ui->Init_labelInitLog, "Initializing communication utilities...");
	lv_label_set_long_mode(ui->Init_labelInitLog, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Init_labelInitLog, 75, 173);
	lv_obj_set_size(ui->Init_labelInitLog, 456, 304);

	//Write style for Init_labelInitLog, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Init_labelInitLog, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Init_labelInitLog, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Init_labelInitLog, lv_color_hex(0xd8d8d8), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Init_labelInitLog, &lv_font_misans_18, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Init_labelInitLog, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Init_labelInitLog, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Init_labelInitLog, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Init_labelInitLog, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Init_labelInitLog, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Init_labelInitLog, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Init_labelInitLog, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Init_labelInitLog, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Init_labelInitLog, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Init_labelInitLog, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes Init_label_3
	ui->Init_label_3 = lv_label_create(ui->Init);
	lv_label_set_text(ui->Init_label_3, "3SE飞镖中控");
	lv_label_set_long_mode(ui->Init_label_3, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Init_label_3, 110, 15);
	lv_obj_set_size(ui->Init_label_3, 232, 43);

	//Write style for Init_label_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Init_label_3, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Init_label_3, &lv_font_misans_28, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->Init_label_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Init_label_3, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Init_label_3, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Init_label_3, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write style for Init_label_3, Part: LV_PART_MAIN, State: LV_STATE_DISABLED.
	lv_obj_set_style_border_width(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_radius(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_text_color(ui->Init_label_3, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_text_font(ui->Init_label_3, &lv_font_misans_16, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_text_opa(ui->Init_label_3, 255, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_text_letter_space(ui->Init_label_3, 2, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_text_line_space(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_text_align(ui->Init_label_3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_bg_opa(ui->Init_label_3, 255, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_bg_color(ui->Init_label_3, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_bg_grad_dir(ui->Init_label_3, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_pad_top(ui->Init_label_3, 8, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_pad_right(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_pad_bottom(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_pad_left(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DISABLED);
	lv_obj_set_style_shadow_width(ui->Init_label_3, 0, LV_PART_MAIN|LV_STATE_DISABLED);

	//The custom code of Init.
	

	//Update current screen layout.
	lv_obj_update_layout(ui->Init);

}
