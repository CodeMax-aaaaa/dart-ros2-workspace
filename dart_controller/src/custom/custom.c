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
// bool msgbox_queue_empty();
lv_anim_ready_cb_t ready_cb(lv_anim_t *a);
lv_anim_ready_cb_t dequeue_ready_cb(lv_anim_t *a);
// static void send_msg(const char *title, const char *msg, msg_emerg_t emerg);
// static void timer_cb(lv_timer_t *timer);

// /**********************
//  *  STATIC PROTOTYPES
//  **********************/

// /**********************
//  *  STATIC VARIABLES
//  **********************/
// // 该队列后续应该使用om重写
// // 动态分配内存
// struct msgbox_queue_node
// {
//     char *title;
//     char *msg;
//     msg_emerg_t emerg;
//     struct msgbox_queue_node *next;
// };

// struct msgbox_queue
// {
//     struct msgbox_queue_node *head;
//     struct msgbox_queue_node *tail;
// };

// struct msgbox_queue msgbox_queue;
// // msgbox维护
// lv_timer_t *msgbox_timer;

// bool msgbox_queue_empty()
// {
//     return msgbox_queue.head == NULL;
// }

// // ready_cb
// lv_anim_ready_cb_t ready_cb(lv_anim_t *a)
// {
//     lv_timer_enable(msgbox_timer);
// }

// static void timer_cb(lv_timer_t *timer)
// {
//     // ui_anim
//     // Animation for moving the message box to the center of the screen
//     lv_timer_del(timer);
//     ui_move_animation(guider_ui.Main_msgbox, 200, 0, 840, 300, &lv_anim_path_ease_in, 0, 0, 0, 0, NULL, dequeue_ready_cb, NULL);
// }

// // timer cb
// lv_anim_ready_cb_t dequeue_ready_cb(lv_anim_t *a)
// {
//     // dequeue
//     struct msgbox_queue_node *node = msgbox_queue.head;
//     msgbox_queue.head = msgbox_queue.head->next;
//     if (msgbox_queue.tail == node)
//     {
//         msgbox_queue.tail = NULL;
//     }
//     free(node->title);
//     free(node->msg);
//     free(node);
//     // if not empty, show the next msgbox
//     if (!msgbox_queue_empty())
//     {
//         lv_label_set_text(lv_msgbox_get_title(guider_ui.Main_msgbox), msgbox_queue.head->title);
//         lv_label_set_text(lv_msgbox_get_text(guider_ui.Main_msgbox), msgbox_queue.head->msg);
//         // emerg
//         switch (msgbox_queue.head->emerg)
//         {
//         case MSG_EMERG_INFO:
//             lv_obj_set_style_bg_color(guider_ui.Main_msgbox, lv_color_hex(0x002140), LV_PART_MAIN | LV_STATE_DEFAULT);
//             break;
//         case MSG_EMERG_WARN:
//             lv_obj_set_style_bg_color(guider_ui.Main_msgbox, lv_color_hex(0xB66D1B), LV_PART_MAIN | LV_STATE_DEFAULT);
//             break;
//         case MSG_EMERG_ERROR:
//             lv_obj_set_style_bg_color(guider_ui.Main_msgbox, lv_color_hex(0x861078), LV_PART_MAIN | LV_STATE_DEFAULT);
//             break;
//         case MSG_EMERG_CRITICAL:
//             lv_obj_set_style_bg_color(guider_ui.Main_msgbox, lv_color_hex(0x8B0015), LV_PART_MAIN | LV_STATE_DEFAULT);
//             break;
//         default:
//             break;
//         }
//         if(msgbox_queue.head->next!=NULL) // 有两个以上的消息
//         {
//             // 定时器
//             msgbox_timer = lv_timer_create(timer_cb, 1000, NULL);
//         }
//         else
//         {
//             // 定时器
//             msgbox_timer = lv_timer_create(timer_cb, 1500, NULL);
//         }
//         // Animation for moving the message box to the center of the screen
//         ui_move_animation(guider_ui.Main_msgbox, 200, 0, 330, 300, &lv_anim_path_ease_in, 0, 0, 0, 0, NULL, ready_cb, NULL);
//     }
//     else
//     {
//         lv_obj_add_flag(guider_ui.Main_msgbox, LV_OBJ_FLAG_HIDDEN);
//     }
// }

// static void send_msg(const char *title, const char *msg, msg_emerg_t emerg)
// {
//     // 展示msgbox， 3s后自动关闭
//     // Write codes Main_msgbox
//     // lv_timer
//     // if empty, show the next msgbox
//     if (msgbox_queue_empty())
//     {
//         // enqueue
//         struct msgbox_queue_node *node = (struct msgbox_queue_node *)malloc(sizeof(struct msgbox_queue_node));
//         node->title = malloc(strlen(title) + 1);
//         memcpy(node->title, title, strlen(title) + 1);
//         node->msg = malloc(strlen(msg) + 1);
//         memcpy(node->msg, msg, strlen(msg) + 1);
//         node->emerg = emerg;
//         node->next = NULL;
//         msgbox_queue.tail = node;
//         msgbox_queue.head = node;

//         lv_label_set_text(lv_msgbox_get_title(guider_ui.Main_msgbox), msgbox_queue.head->title);
//         lv_label_set_text(lv_msgbox_get_text(guider_ui.Main_msgbox), msgbox_queue.head->msg);
//         // emerg
//         switch (msgbox_queue.head->emerg)
//         {
//         case MSG_EMERG_INFO:
//             lv_obj_set_style_bg_color(guider_ui.Main_msgbox, lv_color_hex(0x002140), LV_PART_MAIN | LV_STATE_DEFAULT);
//             break;
//         case MSG_EMERG_WARN:
//             lv_obj_set_style_bg_color(guider_ui.Main_msgbox, lv_color_hex(0x967000), LV_PART_MAIN | LV_STATE_DEFAULT);
//             break;
//         case MSG_EMERG_ERROR:
//             lv_obj_set_style_bg_color(guider_ui.Main_msgbox, lv_color_hex(0x861078), LV_PART_MAIN | LV_STATE_DEFAULT);
//             break;
//         case MSG_EMERG_CRITICAL:
//             lv_obj_set_style_bg_color(guider_ui.Main_msgbox, lv_color_hex(0x8B0015), LV_PART_MAIN | LV_STATE_DEFAULT);
//             break;
//         default:
//             break;
//         }
//         lv_obj_clear_flag(guider_ui.Main_msgbox, LV_OBJ_FLAG_HIDDEN);

//         // 定时器
//         msgbox_timer = lv_timer_create(timer_cb, 1500, NULL);
//         // Animation for moving the message box to the center of the screen
//         ui_move_animation(guider_ui.Main_msgbox, 200, 0, 330, 300, &lv_anim_path_ease_in, 0, 0, 0, 0, NULL, ready_cb, NULL);
//     }
//     else
//     {
//         // enqueue
//         struct msgbox_queue_node *node = (struct msgbox_queue_node *)malloc(sizeof(struct msgbox_queue_node));
//         node->title = malloc(strlen(title) + 1);
//         strcpy(node->title, title);
//         node->msg = malloc(strlen(msg) + 1);
//         strcpy(node->msg, msg);
//         node->emerg = emerg;
//         node->next = NULL;
//         msgbox_queue.tail->next = node;
//         msgbox_queue.tail = node;
//     }
// }
// // 按键回调函数
// static void btnAllOn_cb(lv_event_t *e)
// {
//     // Write codes btnAllOn
//     send_msg("提示", "总开命令已发送", MSG_EMERG_INFO);
// }

// static void btnAllOff_cb(lv_event_t *e)
// {
//     // Write codes btnAllOff
//     send_msg("提示", "总关命令已发送", MSG_EMERG_INFO);
// }

// static void swPDEnable_cb(lv_event_t *e)
// {
//     // Write codes swPDEnable
//     if (lv_obj_has_state(guider_ui.Main_swPDEnable, LV_STATE_CHECKED))
//     {
//         send_msg("提示", "无线充电已打开", MSG_EMERG_INFO);
//     }
//     else
//     {
//         send_msg("提示", "无线充电已关闭", MSG_EMERG_INFO);
//     }
// }

void custom_init(lv_ui *ui)
{
    /* Add your codes here */
    // msgbox_queue.head = NULL;
    // msgbox_queue.tail = NULL;
    lv_obj_clear_flag(guider_ui.Main, LV_OBJ_FLAG_SCROLLABLE);
    // lv_obj_add_event_cb(guider_ui.Main_btnAllOn, btnAllOn_cb, LV_EVENT_CLICKED, NULL);           // 绑定回调函数
    // lv_obj_add_event_cb(guider_ui.Main_btnAllOff, btnAllOff_cb, LV_EVENT_CLICKED, NULL);         // 绑定回调函数
    // lv_obj_add_event_cb(guider_ui.Main_swPDEnable, swPDEnable_cb, LV_EVENT_VALUE_CHANGED, NULL); // 绑定回调函数
    lv_obj_clear_flag(guider_ui.Main_msgbox, LV_OBJ_FLAG_SCROLLABLE);
}