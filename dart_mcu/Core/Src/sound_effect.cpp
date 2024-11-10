//
// Created by cheny on 24-9-10.
//
#include "main.h"
#include "buzzer.h"
#include "buzzer_tones.h"
#include "sound_effect.h"
#include <vector>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcutils/time.h>
#include "FreeRTOS.h"
#include "task.h"

SoundEffectManager soundEffectManager;

void SoundEffectManager::begin(TIM_HandleTypeDef *timer_pwm,
                               TIM_HandleTypeDef *timer_beep_,
                               __IO uint32_t pwm_channel,
                               uint32_t timerClockFreqHz) {
    Buzzer_InitTypeDef buzzerConfig;
    buzzerConfig.channel = pwm_channel;
    buzzerConfig.timer = timer_pwm;
    buzzerConfig.timerClockFreqHz = timerClockFreqHz;
    timer_beep = timer_beep_;
    Buzzer_Init(&hbuzzer, &buzzerConfig);
    Buzzer_Start(&hbuzzer);
}

std::shared_ptr<soundEffect_t>
SoundEffectManager::addSoundEffect(note_t *notes_, size_t notes_size_, bool emergency, bool circulating) {
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

void SoundEffectManager::Start_SoundEffect() {
    // 如果队列不为空，加载下一个音效
    if (!this->soundEffects_queue.empty()) {
        this->currentSoundEffect = this->soundEffects_queue.front();
        this->currentSoundEffect->state =
                this->currentSoundEffect->state == SoundEffectState::READY_FOR_CIRCULATING
                ? SoundEffectState::CIRCULATING : SoundEffectState::PLAYING;
    } else {
        HAL_TIM_Base_Stop(timer_beep);
    }
    if (this->currentSoundEffect != nullptr) {
        // 设定第一个音符的持续时间
        uint32_t duration = this->currentSoundEffect->notes[this->currentSoundEffect->progress].duration;
        // 设置定时器中断时间为音符的持续时间
        __HAL_TIM_SET_AUTORELOAD(timer_beep, duration * 10 - 1);
        __HAL_TIM_SET_COUNTER(timer_beep, 0);
        HAL_TIM_Base_Start_IT(timer_beep);
    }
}

void SoundEffectManager::timer_callback(void *pvParameters) {
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
            __HAL_TIM_SET_AUTORELOAD(g_manager->timer_beep, (duration * 10 - 1)); // 设置定时器中断时间为音符的持续时间

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

void SoundEffectManager::stopCurrentSoundEffect() {
    if (currentSoundEffect != nullptr) {
        currentSoundEffect->state = SoundEffectState::PLAYED;
    }
}

void SoundEffectManager::clearSoundEffects() {
    soundEffects_queue.clear();
    currentSoundEffect = nullptr;
    Buzzer_NoNote(&hbuzzer);
    HAL_TIM_Base_Stop(timer_beep);
}