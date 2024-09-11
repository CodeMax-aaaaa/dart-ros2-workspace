//
// Created by cheny on 24-9-10.
//
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/time.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <buzzer.h>
#include "buzzer_examples.h"
#include "dartmcu_node.h"

#include "sound_effect.h"
#include "tim.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)
#define BUZZER_NOTE(x) x, (sizeof(x) / sizeof(x[0]))
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_autopilot_disconnect));}}

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;


rcl_allocator_t allocator;
rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
rcl_publisher_t publisher;
rcl_node_t node;
rclc_support_t support;
rcl_timer_t timer;

rclc_executor_t executor;
std_msgs__msg__Int64 msg;
bool micro_ros_init_successful;

SoundEffectManager soundEffectManager;

bool create_entities();

void set_ros_transport();

void destroy_entities();

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);

void subscription_callback(const void *msgin);

void microros_node_task(void) {
    soundEffectManager.Init(&htim12, &htim6, TIM_CHANNEL_1, HAL_RCC_GetPCLK2Freq());

    soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_reconnect));

    set_ros_transport();
    state = WAITING_AGENT;

    while (1) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE
                                                                                            : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                } else if (state == AGENT_CONNECTED) {
                    soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_plug_in), false, false);
                }
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED
                                                                                             : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));
                } else if (state == AGENT_DISCONNECTED) {
                    soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_remove), false, false);
                }
                break;
            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }

    }
};

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        // 时间戳同步
        rmw_uros_sync_session(100);
        msg.data = rmw_uros_epoch_millis();
        rcl_publish(&publisher, &msg, NULL);
    }
}

bool create_entities() {

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "dart_mcu", "", &support));

    // create publisher
    rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
            "controller/time_since_epoch");
    // subscribe to /buzzer/cmd_note and /buzzer/cmd_sound_effect
    // create subscriber
    RCSOFTCHECK(rclc_subscription_init_best_effort(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "buzzer/cmd_sound_effect"));
    // create timer,
    const unsigned int timer_timeout = 10000;
    RCCHECK(rclc_timer_init_default2(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback, true));

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                               &subscription_callback,
                                               ON_NEW_DATA));
    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);
    rcl_subscription_fini(&subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    // 重新初始化USB设备
    // 断联
    USB_DEVICE_Stop();
    HAL_Delay(200);
    USB_DEVICE_Start();
}

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *) msgin;
    switch (msg->data) {
        case song_list::Eautopilot_disconnect:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_autopilot_disconnect));
            break;
        case song_list::Elaoda:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_laoda));
            break;
        case song_list::Ewinxp:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
            break;
        case song_list::Ereconnect:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_reconnect));
            break;
        case song_list::Estartup:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_startup));
            break;
        case song_list::Eplug_in:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_plug_in));
            break;
        case song_list::Eremove:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_remove));
            break;
        default:
            soundEffectManager.stopCurrentSoundEffect();
            break;
    }
}