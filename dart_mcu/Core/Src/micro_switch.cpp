//
// Created by cheny on 24-9-18.
//

#include "micro_switch.h"

E_Lead_Screw_Switch_State yaw_switch_state;
E_Lead_Screw_Switch_State pitch_switch_state;
E_Lead_Screw_Switch_State trigger_switch_state;

#define read_switch_state_debounce_time(GPIO_Port, GPIO_Pin, switch_state) \
    if(HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin) == GPIO_PIN_SET) { \
        if(HAL_GetTick() - switch_state##_last_time > 10) { \
            switch_state = Triggered; \
        } \
    } else { \
        switch_state = Untriggered; \
    }

// 消抖读取
void micro_switch_read() {
    static uint32_t yaw_switch_state_last_time = 0;
    static uint32_t pitch_switch_state_last_time = 0;
    static uint32_t trigger_switch_state_last_time = 0;

    read_switch_state_debounce_time(Yaw_Switch_GPIO_Port, Yaw_Switch_Pin, yaw_switch_state);
    read_switch_state_debounce_time(Pitch_Switch_GPIO_Port, Pitch_Switch_Pin, pitch_switch_state);
    read_switch_state_debounce_time(Trigger_Switch_GPIO_Port, Trigger_Switch_Pin, trigger_switch_state);
}