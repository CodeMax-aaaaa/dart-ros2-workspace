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

// 可以触发音效事件/进入警报模式
typedef struct SoundEffect {
    // 音效数组
    note_t* notes;
} SoundEffect_t;