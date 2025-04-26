//
// 序列化和反序列化飞镖架参数
// Created by cheny on 25-4-25.
//
#include "dart_launcher_param.h"

#include <string.h>
#include <stdint.h>

#include <stdint.h>

// 拆 uint64_t → hi/lo int32_t
static inline void split_u64(uint64_t u, int32_t *hi, int32_t *lo) {
    *hi = (int32_t)(u >> 32);
    *lo = (int32_t)(u & 0xFFFFFFFFu);
}

// 重组 hi/lo → uint64_t
static inline uint64_t combine_u64(int32_t hi, int32_t lo) {
    return (((uint64_t)(uint32_t)hi) << 32) | (uint32_t)lo;
}

// 拆 double → hi/lo int32_t（通过 uint64_t 位模式）
static inline void split_double(double d, int32_t *hi, int32_t *lo) {
    union { double d; uint64_t u; } conv;
    conv.d = d;
    split_u64(conv.u, hi, lo);
}

// 重组 hi/lo → double
static inline double combine_double(int32_t hi, int32_t lo) {
    union { double d; uint64_t u; } conv;
    conv.u = combine_u64(hi, lo);
    return conv.d;
}

void serializeParams(const DartLauncherParams *p, int32_t buf[DL_PARAMS_BUF_LEN]) {
    int idx = 0;
    // 主控 4 个 int32
    buf[idx++] = p->primary_yaw;
    buf[idx++] = p->primary_yaw_offset;
    buf[idx++] = p->primary_force;
    buf[idx++] = p->primary_force_offset;
    // 副控数组各 4 个
    for (int i = 0; i < 4; i++) buf[idx++] = p->auxiliary_yaw_offsets[i];
    for (int i = 0; i < 4; i++) buf[idx++] = p->auxiliary_force_offsets[i];
    // 2 个 uint8 → int32
    buf[idx++] = p->dart_launch_process_offset_begin;
    buf[idx++] = p->dart_launch_process_offset_end;
    // uint64_t 拆高低 32bit
    split_u64(p->last_param_update_time, &buf[idx], &buf[idx+1]); idx += 2;
    // 此时 idx == DL_PARAMS_BUF_LEN
}

void deserializeParams(const int32_t buf[DL_PARAMS_BUF_LEN], DartLauncherParams *p) {
    int idx = 0;
    p->primary_yaw                  = buf[idx++];
    p->primary_yaw_offset           = buf[idx++];
    p->primary_force                = buf[idx++];
    p->primary_force_offset         = buf[idx++];
    for (int i = 0; i < 4; i++) p->auxiliary_yaw_offsets[i]   = buf[idx++];
    for (int i = 0; i < 4; i++) p->auxiliary_force_offsets[i] = buf[idx++];
    p->dart_launch_process_offset_begin = (uint8_t)buf[idx++];
    p->dart_launch_process_offset_end   = (uint8_t)buf[idx++];
    p->last_param_update_time = combine_u64(buf[idx], buf[idx+1]); idx += 2;
}

void serializeStatus(const DartLauncherStatus *s, int32_t buf[DL_STATUS_BUF_LEN]) {
    int idx = 0;
    // 将 6 个 bool 打包到一个 int32（bit0…bit5）
    buf[idx++] =
            (s->motor_yaw_online       ? 1<<0 : 0) |  // Yaw 在线
            (s->motor_loader_online[0] ? 1<<1 : 0) |  // Loader[0]
            (s->motor_loader_online[1] ? 1<<2 : 0) |  // Loader[1]
            (s->motor_trigger_online   ? 1<<3 : 0) |  // Trigger 在线
            (s->judge_online           ? 1<<4 : 0) |  // 裁判系统
            (s->rc_online              ? 1<<5 : 0);   // 遥控器&#8203;:contentReference[oaicite:5]{index=5}
    // 状态字节
    buf[idx++] = s->dart_state;
    buf[idx++] = s->dart_launch_process;
    // 两个角度
    buf[idx++] = s->motor_yaw_angle;
    buf[idx++] = s->motor_trigger_angle;
    // 电流与角度数组
    buf[idx++] = s->motor_loader_current[0];
    buf[idx++] = s->motor_loader_current[1];
    buf[idx++] = s->motor_loader_angle[0];
    buf[idx++] = s->motor_loader_angle[1];
    // double 拆高低 32bit
    split_double(s->last_launch_speed, &buf[idx], &buf[idx+1]); idx += 2;  // :contentReference[oaicite:6]{index=6}
    // uint64_t 拆高低 32bit
    split_u64(s->last_launch_time, &buf[idx], &buf[idx+1]); idx += 2;        // :contentReference[oaicite:7]{index=7}
}

void deserializeStatus(const int32_t buf[DL_STATUS_BUF_LEN], DartLauncherStatus *s) {
    int bits, idx = 0;
    bits = buf[idx++];
    s->motor_yaw_online        =  bits & (1<<0);
    s->motor_loader_online[0]  =  bits & (1<<1);
    s->motor_loader_online[1]  =  bits & (1<<2);
    s->motor_trigger_online    =  bits & (1<<3);
    s->judge_online            =  bits & (1<<4);
    s->rc_online               =  bits & (1<<5);
    s->dart_state              = (uint8_t)buf[idx++];
    s->dart_launch_process     = (uint8_t)buf[idx++];
    s->motor_yaw_angle         = buf[idx++];
    s->motor_trigger_angle     = buf[idx++];
    s->motor_loader_current[0] = buf[idx++];
    s->motor_loader_current[1] = buf[idx++];
    s->motor_loader_angle[0]   = buf[idx++];
    s->motor_loader_angle[1]   = buf[idx++];
    s->last_launch_speed       = combine_double(buf[idx], buf[idx+1]); idx += 2;
    s->last_launch_time        = combine_u64(buf[idx], buf[idx+1]);       idx += 2;
}