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

#include <std_msgs/msg/int64.h>
#include <buzzer.h>
#include "buzzer_examples.h"
#include "dartmcu_node.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)\

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

bool create_entities();

void destroy_entities();


void microros_node_task(void) {
    state = WAITING_AGENT;
    // micro-ROS configuration
    rmw_uros_set_custom_transport(
            true,
            (void *) NULL,
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read);

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
                };
                break;
            case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED
                                                                                         : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));
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
        rcl_publish(&publisher, &msg, NULL);
        msg.data = rmw_uros_epoch_millis();
    }
}

TaskHandle_t send_msg_task_handle = NULL;
bool send_msg_task_spin = false;

void send_msg_task(void *pvParameters) {
    std_msgs__msg__Int64 msg;
    msg.data = 0;
    while (send_msg_task_spin) {
        rcl_publish(&publisher, &msg, NULL);
        vTaskDelay(9000 / portTICK_PERIOD_MS);
    }
}

bool create_entities() {

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "cubemx_node", "", &support));

    // create publisher
    rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
            "cubemx_publisher");

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default2(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback, true));



    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    send_msg_task_spin = true;
    xTaskCreate(send_msg_task, "send_msg_task", 200, NULL, 1, &send_msg_task_handle);

    const size_t songSize = sizeof(buzzer_plug_in) / sizeof(buzzer_plug_in[0]);
    for (size_t j = 0; j < 1; j++) {
        for (size_t i = 0; i < songSize; i++) {
            Buzzer_Note(&hbuzzer, buzzer_plug_in[i].pitch);
            HAL_Delay(buzzer_plug_in[i].duration);
        }
    }
    Buzzer_NoNote(&hbuzzer);

    return true;
}

void destroy_entities() {
    const size_t songSize = sizeof(buzzer_remove) / sizeof(buzzer_remove[0]);
    for (size_t j = 0; j < 1; j++) {
        for (size_t i = 0; i < songSize; i++) {
            Buzzer_Note(&hbuzzer, buzzer_remove[i].pitch);
            HAL_Delay(buzzer_remove[i].duration);
        }
    }
    Buzzer_NoNote(&hbuzzer);

    send_msg_task_spin = false;

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    // 重新初始化USB设备
    // 断联
    USB_DEVICE_Stop();
    HAL_Delay(200);
    USB_DEVICE_Start();
}
