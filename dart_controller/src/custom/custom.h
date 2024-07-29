// SPDX-License-Identifier: MIT
// Copyright 2020 NXP

/*
 * custom.h
 *
 *  Created on: July 29, 2020
 *      Author: nxf53801
 */

#ifndef __CUSTOM_H_
#define __CUSTOM_H_
#ifdef __cplusplus
#include <string>
void loadDartInfo(std::string dart_name);
extern "C"
{
#endif

#include "gui_guider.h"

    void custom_init(lv_ui *ui);

    void print_cb(const char *buf);

    // void textarea_event_cb(lv_event_t * e);

    // void color_changer_event_cb(lv_event_t * e);

    // void color_event_cb(lv_event_t * e);

    // void shop_chart_event_cb(lv_event_t * e);

    // void arc_MT_anim_cb(void * var, int32_t v);

    // void chart_event_cb(lv_event_t * e);

    // void meter_sessions_timer_cb(lv_timer_t * timer);

    // void meterNS_anim_cb(void * var, int32_t v);

    void set_switch_state(lv_obj_t *sw, bool state);

    void dart_list_event_cb(lv_event_t *e);

    extern bool callback_spinbox_disabled;
    extern lv_obj_t *obj_dart_list_seleted;
    void ta_debug_velocity_cb(lv_event_t *e);

#ifdef __cplusplus
}
#endif
#endif /* EVENT_CB_H_ */
