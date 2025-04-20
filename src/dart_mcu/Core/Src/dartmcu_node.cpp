//
// Created by cheny on 24-9-10.
//
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usb_device.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/time.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <stdlib.h>
#include <string.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <buzzer.h>
#include "buzzer_examples.h"
#include "dartmcu_node.h"

#include "sound_effect.h"
#include "tim.h"

#include "led.h"
#include "velocimeter.h"
#include "servo.h"

#include "state_machine.h"

#include "motor_controller.h"

#include <dart_config.h>

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

rcl_allocator_t allocator;
rcl_subscription_t subscriber_buzzer = rcl_get_zero_initialized_subscription();
rcl_subscription_t subscriber_servo = rcl_get_zero_initialized_subscription();
rcl_subscription_t subscriber_parameter = rcl_get_zero_initialized_subscription();
rcl_publisher_t publisher;
rcl_publisher_t publisher_can;
rcl_publisher_t publisher_dart_velocity_meter;
rcl_node_t node;
rclc_support_t support;
rcl_timer_t timer, timer2;

rclc_executor_t executor;
std_msgs__msg__Int64 msgInt64;
std_msgs__msg__String msgString;

struct {
    double velocity;
    bool is_valid;
} velocity_meter_result;

TaskHandle_t velocity_meter_result_task_handle;

void publish_velocity_meter_result(void *arg) {
    double velocity = 0;
    while (1) {
        // 等待任务通知
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        velocity = velocity_meter_result.velocity;
        std_msgs__msg__Float64 msg_velocity;
        msg_velocity.data = velocity;
        rcl_publish(&publisher_dart_velocity_meter, &msg_velocity, NULL);
    }
}

void microros_node_task(void) {
    soundEffectManager.begin(&htim12, &htim6, TIM_CHANNEL_1, HAL_RCC_GetPCLK2Freq());
    LED::led_flow.begin();
    trigger_servo[0].begin(&htim4, TIM_CHANNEL_1, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 180, 10000, 100,
                           CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_0);
    trigger_servo[1].begin(&htim5, TIM_CHANNEL_3, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 180, 10000, 100,
                           CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_1);
    trigger_servo[2].begin(&htim4, TIM_CHANNEL_3, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_LOAD_SERVO_UP_ANGLE_0);
    trigger_servo[3].begin(&htim4, TIM_CHANNEL_4, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_LOAD_SERVO_UP_ANGLE_0);
    trigger_servo[4].begin(&htim5, TIM_CHANNEL_1, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_LOAD_SERVO_UP_ANGLE_1);
    trigger_servo[5].begin(&htim5, TIM_CHANNEL_2, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_LOAD_SERVO_UP_ANGLE_1);
    trigger_servo[6].begin(&htim5, TIM_CHANNEL_4, HAL_RCC_GetPCLK2Freq(), 500, 2500, 0, 270, 10000, 100,
                           CONFIG_SLIDE_SERVO_CUT_ANGLE);


    meter::velocity_meter.begin(&htim8, TIM_CHANNEL_1, &htim8, TIM_CHANNEL_2, 65536, [=](float velocity) {
        velocity_meter_result.velocity = velocity;
        velocity_meter_result.is_valid = true;
        xTaskNotifyFromISR(velocity_meter_result_task_handle, 0, eNoAction, NULL);
    }, 0.1, 0.000001);

    xTaskCreate(publish_velocity_meter_result, "publish_velocity_meter_result", 64, NULL, 5,
                &velocity_meter_result_task_handle);

    xTaskCreate(state_machine::fsm_thread, "fsm_thread", 256, NULL, 1, NULL);

    xTaskCreate(motor_controller::pid_control_task, "pid_control_task", 128, NULL, 6, NULL);
    set_ros_transport();
    state = WAITING_AGENT;

    while (1) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_AVAILABLE
                                                                                            : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                } else if (state == AGENT_CONNECTED) {
                    soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_plug_in), false, false);
                    LED::setLED(
                            LED::LED_GREEN, true);
                    LED::led_flow.flow_state_ = LED::LED_Flow_State::FLOW_NORMAL;
                }
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(3000,
                                   state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED
                                                                                       : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500));
                } else if (state == AGENT_DISCONNECTED) {
                    soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_remove), false, false);
                    LED::setLED(LED::LED_GREEN, false);
                    LED::led_flow.flow_state_ = LED::LED_Flow_State::FLOW_NONE;
                }
                break;
            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }
        vTaskDelay(1);
    }
};

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != nullptr) {
        // 时间戳同步
        rmw_uros_sync_session(100);
        // 写入日志msgString
        sprintf(msgString.data.data, "Time #%d", rmw_uros_epoch_millis());
        msgString.data.size = strlen(msgString.data.data);
        rcl_publish(&publisher, &msgString, nullptr);
    }
}

std_msgs__msg__Int32MultiArray can_controller_data_msg;

std_msgs__msg__Int32MultiArray controller_parameter_setting_msg;

void timer2_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != nullptr) {
        // 先发送Pitch电机的数据，1.电机角度 2.电机速度 3.控制器输入 4.控制器输出
        int32_t *data = can_controller_data_msg.data.data;
        data[0] = motor::MotorLoad[0].current_angle_;
        data[1] = motor::MotorLoad[0].current_velocity_;
        data[2] = motor_controller::MotorLoadController[0].current_angle_with_rounds_;
        data[3] = motor::MotorLoad[0].target_current_;
        // 再发送Yaw电机的数据，1.电机角度 2.电机速度 3.控制器输入 4.控制器输出
        data[4] = motor::MotorLoad[1].current_angle_;
        data[5] = motor::MotorLoad[1].current_velocity_;
        data[6] = motor_controller::MotorLoadController[1].current_angle_with_rounds_;
        data[7] = motor::MotorLoad[1].target_current_;
        // 最后发送扳机丝杆电机的数据，1.电机角度 2.电机速度 3.控制器输入 4.控制器输出
        data[8] = motor::MotorTriggerLS.current_angle_;
        data[9] = motor::MotorTriggerLS.current_velocity_;
        data[10] = motor_controller::MotorTriggerLSController.target_velocity_;
        data[11] = motor::MotorTriggerLS.target_current_;

        rcl_publish(&publisher_can, &can_controller_data_msg, nullptr);
    }
}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "dart_mcu", "", &support));

    // create publisher
    rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/dart_launcher_mcu/log");

    rclc_publisher_init_default(
            &publisher_dart_velocity_meter,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
            "/dart_launcher_mcu/dart_velocity_meter");

    rclc_publisher_init_default(
            &publisher_can,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "/dart_launcher_mcu/can_controller_data");

    // subscribe to /buzzer/cmd_note and /buzzer/cmd_sound_effect
    // create subscriber
    RCSOFTCHECK(rclc_subscription_init_best_effort(
            &subscriber_buzzer,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "buzzer/cmd_sound_effect"));

    RCSOFTCHECK(rclc_subscription_init_best_effort(
            &subscriber_servo,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "controller/cmd_servo_test"));

    RCSOFTCHECK(rclc_subscription_init_best_effort(
            &subscriber_parameter,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "controller/cmd"));

    // create timer,
    const unsigned int timer1_timeout = 10000;
    RCCHECK(rclc_timer_init_default2(
            &timer,
            &support,
            RCL_MS_TO_NS(timer1_timeout),
            timer_callback, true));

    const unsigned int timer2_timeout = 10;
    RCCHECK(rclc_timer_init_default2(
            &timer2,
            &support,
            RCL_MS_TO_NS(timer2_timeout),
            timer2_callback, true));

    // 初始化消息
    msgString.data.data = (char *) malloc(200);
    msgString.data.size = 0;
    msgString.data.capacity = 200;


    can_controller_data_msg.data.capacity = 12;
    can_controller_data_msg.data.size = 12;

    // 分配内存给 data 数组
    can_controller_data_msg.data.data = (int32_t *) malloc(can_controller_data_msg.data.capacity * sizeof(int32_t));

    // 初始化 layout.dim 数组
    can_controller_data_msg.layout.dim.capacity = 1;
    can_controller_data_msg.layout.dim.size = 1;

    // 为 dim 分配内存
    can_controller_data_msg.layout.dim.data =
            (std_msgs__msg__MultiArrayDimension *) malloc(sizeof(std_msgs__msg__MultiArrayDimension));

    // 初始化维度信息
    static char label[] = "motor_data";
    can_controller_data_msg.layout.dim.data[0].label.data = label;
    can_controller_data_msg.layout.dim.data[0].label.size = strlen(label);
    can_controller_data_msg.layout.dim.data[0].label.capacity = strlen(label);
    can_controller_data_msg.layout.dim.data[0].size = 4;   // 长度为4
    can_controller_data_msg.layout.dim.data[0].stride = 1;  // 步长设置为1

    controller_parameter_setting_msg.data.capacity = 3;
    controller_parameter_setting_msg.data.size = 3;

    controller_parameter_setting_msg.data.data =
            (int32_t *) malloc(controller_parameter_setting_msg.data.capacity * sizeof(int32_t));

    controller_parameter_setting_msg.layout.dim.capacity = 1;
    controller_parameter_setting_msg.layout.dim.size = 1;
    controller_parameter_setting_msg.layout.dim.data =
            (std_msgs__msg__MultiArrayDimension *) malloc(sizeof(std_msgs__msg__MultiArrayDimension));

    static char label2[] = "controller_parameter";
    controller_parameter_setting_msg.layout.dim.data[0].label.data = label2;
    controller_parameter_setting_msg.layout.dim.data[0].label.size = strlen(label2);
    controller_parameter_setting_msg.layout.dim.data[0].label.capacity = strlen(label2);
    controller_parameter_setting_msg.layout.dim.data[0].size = 3;   // 长度为3
    controller_parameter_setting_msg.layout.dim.data[0].stride = 1;  // 步长设置为1


    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_timer(&executor, &timer2));

    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &subscriber_buzzer, &msgInt64,
                                               &subscription_buzzer_callback,
                                               ON_NEW_DATA));
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &subscriber_servo, &msgInt64,
                                               &subscription_servo_callback,
                                               ON_NEW_DATA));
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &subscriber_parameter, &controller_parameter_setting_msg,
                                               &subscription_parameter_setting_callback,
                                               ON_NEW_DATA));

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher, &node);
    rcl_publisher_fini(&publisher_can, &node);
    rcl_publisher_fini(&publisher_dart_velocity_meter, &node);
    rcl_timer_fini(&timer);
    rcl_subscription_fini(&subscriber_buzzer, &node);
    rcl_subscription_fini(&subscriber_servo, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    // 重新初始化USB设备
    // 断联
    USB_DEVICE_Stop();
    HAL_Delay(200);
    USB_DEVICE_Start();
}

void subscription_buzzer_callback(const void *msgin) {
    const auto *msg = (const std_msgs__msg__Int32 *) msgin;
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
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_dji_startup));
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
        case song_list::Eprotect:
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_protect));
            break;
        default:
            soundEffectManager.stopCurrentSoundEffect();
            break;
    }
}

#include "dart_config.h"
#include "state_machine.h"

// 将堵转电机移动到初始位置
state_machine::UpsideState state_machine::upside_state = state_machine::UpsideState::Idle;

void subscription_servo_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *) msgin;

    if (msgin != NULL) {
        // Limit the angle
        int angle = msg->data;
        if (angle == 0) {
            trigger_servo[0].setAngle(CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_0);
            trigger_servo[1].setAngle(CONFIG_TRIGGER_SERVO_TRIGGER_ANGLE_1);
        } else if (angle == 1) {
            trigger_servo[0].setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_0);
            trigger_servo[1].setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE_1);
        } else if (angle == 2) {
            state_machine::upside_state = state_machine::UpsideState::MovingDown;
        } else if (angle == 3) {
            state_machine::upside_state = state_machine::UpsideState::MovingUp;
        } else if (angle == 4) {

        }

        trigger_servo[0].enable();
        trigger_servo[1].enable();
//    trigger_servo[0].setAngle(angle);
//    trigger_servo[1].setAngle(angle);
    }
}

void subscription_parameter_setting_callback(const void *msgin) {
    const auto *msg = (const std_msgs__msg__Int32MultiArray *) msgin;
    if (msgin != NULL) {
        int32_t *data = msg->data.data;
        motor_controller::MotorYawLSController.target_angle_with_rounds_ = data[0];
        motor_controller::MotorPitchLSController.target_angle_with_rounds_ = data[1];
        motor_controller::MotorTriggerLSController.target_angle_with_rounds_ = data[2];
    }
}