//
// Created by cheny on 24-9-15.
//

#include "velocimeter.h"

namespace meter {

    velocimeter velocity_meter;

    void velocimeter::begin(TIM_HandleTypeDef *htim_begin, uint32_t channel_begin, TIM_HandleTypeDef *htim_end,
                            uint32_t channel_end,
                            uint32_t timer_period, std::function<void(float)> onVelocityUpdate,
                            double distanceBetweenTwoPulse,
                            double seconds_per_tick) {
        this->htim_begin = htim_begin;
        this->timer_period = timer_period;
        this->onVelocityUpdate = onVelocityUpdate;
        this->distanceBetweenTwoPulse = distanceBetweenTwoPulse;
        this->seconds_per_tick = seconds_per_tick;
        state = READY;
        refresh_counter_timer = 0;
        update_count_begin = 0;
        update_count_end = 0;
        begin_time = 0;
        end_time = 0;

        HAL_TIM_Base_Start_IT(htim_begin);
        HAL_TIM_IC_Start_IT(htim_begin, channel_begin);
        HAL_TIM_IC_Start_IT(htim_end, channel_end);
    }

    void velocimeter::onUpdate(TIM_HandleTypeDef *htim) {
        if (htim == htim_begin) {
            refresh_counter_timer++;
        }
    }

    void velocimeter::onCaptureBegin(uint32_t count) {
        if (state == READY) {
            state = MEASURING;
            update_count_begin = refresh_counter_timer;
            begin_time = count;
        }
    }

    void velocimeter::onCaptureEnd(uint32_t count) {
        if (state == MEASURING) {
            update_count_end = refresh_counter_timer;
            end_time = count;
            // 计算速度
            uint64_t ticks_diff = (update_count_end - update_count_begin) * timer_period + (end_time - begin_time);
            float velocity = distanceBetweenTwoPulse / (ticks_diff * seconds_per_tick);
            onVelocityUpdate(velocity);

            state = READY;
        }
    }
}

