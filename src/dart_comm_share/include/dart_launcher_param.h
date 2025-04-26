//
// Created by cheny on 25-4-19.
//

#ifndef DART_ROS2_WORKSPACE_DART_LAUNCHER_PARAM_H
#define DART_ROS2_WORKSPACE_DART_LAUNCHER_PARAM_H

#include "stdint.h"

#define DL_PARAMS_BUF_LEN 16

#define DL_STATUS_BUF_LEN 13

// 飞镖架对外暴露控制变量
/*
classDiagram
    class DartLauncherParams{
        +int32 primary_yaw, 发射目标主偏航角
        +int32 primary_yaw_offset, 发射目标主偏航角偏移量, 备视觉用
        +int32 primary_force, 发射扳机主位置
        +int32 primary_force_offset, 发射扳机主位置偏移量, 备视觉用
        +int32[4] auxiliary_yaw_offsets, 发射目标副偏航角偏移量, 按程序发射时使用
        +int32[4] auxiliary_force_offsets, 发射扳机副位置偏移量, 按程序发射时使用
        +uint64 last_param_update_time, 上次参数更新时间, 用于同步, 单位ms 高位优先
    }
 */
typedef struct DartLauncherParams {
    int32_t primary_yaw;
    int32_t primary_yaw_offset;
    int32_t primary_force;
    int32_t primary_force_offset;
    int32_t auxiliary_yaw_offsets[4];
    int32_t auxiliary_force_offsets[4];
    uint8_t dart_launch_process_offset_begin;
    uint8_t dart_launch_process_offset_end;
    uint64_t last_param_update_time;
} DartLauncherParams;

/*
 * 拉簧发射架状态回传数据如下：
```mermaid
classDiagram
    class DartLauncherStatus{
        +bool motor_yaw_online, 电机Yaw在线
        +bool[2] motor_loader_online, 电机Load[0, 1]在线
        +bool motor_trigger_online, 电机Trigger在线
        +bool judge_online, 裁判系统在线
        +bool rc_online, 遥控器在线
        +uint8 dart_state, 发射架状态, 100 Boot, 101 Protect, 102 Remote, 103-106 Match Enter Wait Launch Reload, 255 Undefined
        +uint8 dart_launch_process, 发射流程(1/4)
        +int32 motor_yaw_angle, 电机Yaw总角度
        +int32 motor_trigger_angle, 电机Trigger总角度
        +int32[2] motor_loader_current, 电机Load[0, 1]相电流
        +int32[2] motor_loader_angle, 电机Load[0, 1]总角度
        +double last_launch_speed, 上次发射检测速度
        +uint64_t last_launch_time, 上次发射检测数据时间UNIX戳，高位优先
    }
```
 */
typedef struct DartLauncherStatus {
    bool motor_yaw_online;
    bool motor_loader_online[2];
    bool motor_trigger_online;
    bool judge_online;
    bool rc_online;
    uint8_t dart_state;
    uint8_t dart_launch_process;
    int32_t motor_yaw_angle;
    int32_t motor_trigger_angle;
    int32_t motor_loader_current[2];
    int32_t motor_loader_angle[2];
    double last_launch_speed;
    uint64_t last_launch_time;
} DartLauncherStatus;

// 序列化 反序列化函数
void serializeParams(const DartLauncherParams *p, int32_t buf[DL_PARAMS_BUF_LEN]);
void deserializeParams(const int32_t buf[DL_PARAMS_BUF_LEN], DartLauncherParams *p);
void serializeStatus(const DartLauncherStatus *p, int32_t buf[DL_STATUS_BUF_LEN]);
void deserializeStatus(const int32_t buf[DL_STATUS_BUF_LEN], DartLauncherStatus *p);

#endif //DART_ROS2_WORKSPACE_DART_LAUNCHER_PARAM_H
