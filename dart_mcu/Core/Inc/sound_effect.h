//
// Created by cheny on 24-9-10.
//

#ifndef DART_MCU_SOUND_EFFECT_H
#define DART_MCU_SOUND_EFFECT_H

#include "main.h"
#include "buzzer.h"
#include "buzzer_tones.h"
#include <vector>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcutils/time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

// 智能指针
#include <memory>

// 可以触发音效事件/进入警报模式
enum SoundEffectState {
    READY,
    READY_FOR_CIRCULATING,
    PLAYING,
    CIRCULATING,
    PLAYED
};

typedef struct soundEffect_t {
    // 音效数组
    note_t *notes;
    // 音效数组大小
    size_t notes_size;
    // 播放进度
    size_t progress;
    // 行为
    uint8_t state;
} soundEffect_t;


//HAL_RCC_GetPCLK2Freq();
class SoundEffectManager {
private:
    Buzzer_HandleTypeDef hbuzzer;
    std::vector<std::shared_ptr<soundEffect_t>> soundEffects_queue;
    std::shared_ptr<soundEffect_t> currentSoundEffect;
    TIM_HandleTypeDef* timer_beep;

public:
    SoundEffectManager() = default;

    void Init(TIM_HandleTypeDef *timer_pwm,
              TIM_HandleTypeDef *timer_beep_,
              __IO uint32_t channel,
              uint32_t timerClockFreqHz) {
        Buzzer_InitTypeDef buzzerConfig;
        buzzerConfig.channel = channel;
        buzzerConfig.timer = timer_pwm;
        buzzerConfig.timerClockFreqHz = timerClockFreqHz;
        timer_beep = timer_beep_;
        Buzzer_Init(&hbuzzer, &buzzerConfig);
        Buzzer_Start(&hbuzzer);
//        xTaskCreate(SoundEffectManager::soundEffectTask, "soundEffectTask", 100, this, 1, NULL);
    }

    std::shared_ptr<soundEffect_t>
    addSoundEffect(note_t *notes_, size_t notes_size_, bool emergency = false, bool circulating = false) {
        std::shared_ptr<soundEffect_t> soundEffect_ptr = std::make_shared<soundEffect_t>(soundEffect_t());
        soundEffect_ptr->notes = notes_;
        soundEffect_ptr->notes_size = notes_size_;
        soundEffect_ptr->progress = 0;
        soundEffect_ptr->state = circulating ? SoundEffectState::READY_FOR_CIRCULATING : SoundEffectState::READY;
        if (emergency) {
            soundEffects_queue.insert(soundEffects_queue.begin(), soundEffect_ptr);
            // 如果定时器没开启，开始播放音效
            if (currentSoundEffect == nullptr) {
                Start_SoundEffect();
            }
            return soundEffects_queue.front();
        } else {
            soundEffects_queue.push_back(soundEffect_ptr);
            // 如果定时器没开启，开始播放音效
            if (currentSoundEffect == nullptr) {
                Start_SoundEffect();
            }
            return soundEffects_queue.back();
        }
    }

    // 开始播放音效
    void Start_SoundEffect() {
        // 如果队列不为空，加载下一个音效
        if (!this->soundEffects_queue.empty()) {
            this->currentSoundEffect = this->soundEffects_queue.front();
            this->currentSoundEffect->state =
                    this->currentSoundEffect->state == SoundEffectState::READY_FOR_CIRCULATING
                    ? SoundEffectState::CIRCULATING : SoundEffectState::PLAYING;
        } else {
            HAL_TIM_Base_Stop(&htim6);
        }
        if (this->currentSoundEffect != nullptr) {
            // 设定第一个音符的持续时间
            uint32_t duration = this->currentSoundEffect->notes[this->currentSoundEffect->progress].duration;
            // 设置定时器中断时间为音符的持续时间
            __HAL_TIM_SET_AUTORELOAD(&htim6, duration * 10 - 1);
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Start_IT(&htim6);
        }
    }

//    static void soundEffectTask(void *pvParameters) {
//        SoundEffectManager *manager = (SoundEffectManager *) pvParameters;
//        while (1) {
//            if (manager->currentSoundEffect != nullptr &&
//                ((manager->currentSoundEffect->state == SoundEffectState::PLAYING) ||
//                 (manager->currentSoundEffect->state == SoundEffectState::CIRCULATING))) {
//                if (manager->currentSoundEffect->progress < manager->currentSoundEffect->notes_size) {
//                    Buzzer_Note(&manager->hbuzzer,
//                                manager->currentSoundEffect->notes[manager->currentSoundEffect->progress].pitch);
//                    vTaskDelay(manager->currentSoundEffect->notes[manager->currentSoundEffect->progress].duration);
//                    manager->currentSoundEffect->progress++;
//                } else if (manager->currentSoundEffect->state == SoundEffectState::CIRCULATING) {
//                    manager->currentSoundEffect->progress = 0;
//                } else {
//                    manager->currentSoundEffect->state = SoundEffectState::PLAYED;
//                    Buzzer_NoNote(&manager->hbuzzer);
//                    manager->currentSoundEffect = nullptr;
//                    // pop
//                    manager->soundEffects_queue.erase(manager->soundEffects_queue.begin());
//                    vTaskDelay(200);
//                }
//
//            } else if (!manager->soundEffects_queue.empty()) {
//                manager->currentSoundEffect = manager->soundEffects_queue.front();
//                manager->currentSoundEffect->state =
//                        manager->currentSoundEffect->state == SoundEffectState::READY_FOR_CIRCULATING
//                        ? SoundEffectState::CIRCULATING : SoundEffectState::PLAYING;
//            } else
//                vTaskDelay(50);
//        }
//    }

    static void timer_callback(void *pvParameters) {
        SoundEffectManager *g_manager = (SoundEffectManager *) pvParameters;
        // 当前音效处理
        if (g_manager->currentSoundEffect != nullptr) {
            if (g_manager->currentSoundEffect->progress < g_manager->currentSoundEffect->notes_size && (
                    g_manager->currentSoundEffect->state == SoundEffectState::PLAYING ||
                    g_manager->currentSoundEffect->state == SoundEffectState::CIRCULATING)) {
                // 播放当前音符
                Buzzer_Note(&g_manager->hbuzzer,
                            g_manager->currentSoundEffect->notes[g_manager->currentSoundEffect->progress].pitch);

                // 获取当前音符的持续时间，设置下一次中断时间
                uint32_t duration = g_manager->currentSoundEffect->notes[g_manager->currentSoundEffect->progress].duration;
                g_manager->currentSoundEffect->progress++;

                // 重新加载定时器的计数值用于下一次音符的播放
                        __HAL_TIM_SetCounter(g_manager->timer_beep, 0);
                __HAL_TIM_SET_AUTORELOAD(g_manager->timer_beep, duration * 10 - 1); // 设置定时器中断时间为音符的持续时间

            } else {
                // 检查是否循环播放
                if (g_manager->currentSoundEffect->state == SoundEffectState::CIRCULATING) {
                    g_manager->currentSoundEffect->progress = 0; // 循环播放从头开始
                } else {
                    // 如果非循环播放，结束音效
                    g_manager->currentSoundEffect->state = SoundEffectState::PLAYED;
                    Buzzer_NoNote(&g_manager->hbuzzer);
                    g_manager->currentSoundEffect = nullptr;

                    // 从队列中移除已播放的音效
                    g_manager->soundEffects_queue.erase(g_manager->soundEffects_queue.begin());

                    // 如果队列不为空，加载下一个音效
                    if (!g_manager->soundEffects_queue.empty()) {
                        g_manager->currentSoundEffect = g_manager->soundEffects_queue.front();
                        g_manager->currentSoundEffect->state =
                                g_manager->currentSoundEffect->state == SoundEffectState::READY_FOR_CIRCULATING
                                ? SoundEffectState::CIRCULATING : SoundEffectState::PLAYING;
                    } else {
                        HAL_TIM_Base_Stop(g_manager->timer_beep);
                    }
                }
            }
        }

    }

    void stopCurrentSoundEffect() {
        if (currentSoundEffect != nullptr) {
            currentSoundEffect->state = SoundEffectState::PLAYED;
        }
    }

    void clearSoundEffects() {
        soundEffects_queue.clear();
        currentSoundEffect = nullptr;
        Buzzer_NoNote(&hbuzzer);
        HAL_TIM_Base_Stop(timer_beep);
    }
};

extern SoundEffectManager soundEffectManager;


#endif //DART_MCU_SOUND_EFFECT_H
