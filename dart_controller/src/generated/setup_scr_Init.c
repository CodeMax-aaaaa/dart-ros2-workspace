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
	lv_obj_set_style_bg_img_src(ui->Init, &_logo_3SE_1024x600, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_img_opa(ui->Init, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

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

	//Write codes Init_label_2
	ui->Init_label_2 = lv_label_create(ui->Init);
	lv_label_set_text(ui->Init_label_2, "镖神保佑\n此把必中");
	lv_label_set_long_mode(ui->Init_label_2, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Init_label_2, 8, 150);
	lv_obj_set_size(ui->Init_label_2, 653, 273);

	//Write style for Init_label_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Init_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Init_label_2, lv_color_hex(0xdadada), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Init_label_2, &lv_font_FZShaoLGFTJW_128, LV_PART_MAIN|LV_STATE_DEFAULT);
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

	//The custom code of Init.
	

	//Update current screen layout.
	lv_obj_update_layout(ui->Init);

}
