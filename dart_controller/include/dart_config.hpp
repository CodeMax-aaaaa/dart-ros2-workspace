/**
 * @file dart_config.hpp
 * @brief DART Configuration Header
 */
#ifndef DART_CONFIG_H
#define DART_CONFIG_H

#define DART_CAN_INTERFACE "can0"
#define YAW_MAX_ANGLE 350000.0
#define MAX_YAW_CALIBRATION_DELTA_X 5000.0

#include "rclcpp/rclcpp.hpp"
#include <info/msg/dart_param.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

// 数据定义
// C板上传状态数据
//  一 0X701 系统状态1 &发射参数 1 【电机和遥控器在线位 8位 [1字节]+
//  模式位 1字节+堵转过程状态1字节+发射过程状态1字节+摩擦轮Target4字节】
//  一 0X702 发射参数2【摩擦轮 TargetOffset4 字节 +Yaw角度 4字节】
//  一 0X703 发射参数3 【Yaw 轴角度 Offset4 字节+ Yaw轴真实角度 4字节】
//  一 0X704 发射参数4 【第一槽位 Yaw 角度Offset + 第一槽位摩擦轮Offset】
//  一 0X705 发射参数5 【第二槽位 Yaw 角度Offset + 第二槽位摩擦轮Offset】
//  一 0X706 发射参数6 【第三槽位 Yaw 角度Offset + 第三槽位摩擦轮Offset】
//  一 0X707 发射参数7 【第四槽位 Yaw 角度Offset + 第四槽位摩擦轮Offset】
//  一 0X708 发射参数8 【摩擦轮速度比】 double*10000
//  一 0X709 裁判日志【ext_dart_client_cmd.dart_launch_opening_status +
//  ext_game_status.game_progress +
//  ext_dart_info.dart_remaining_time +
//  ext_dart_client_cmd.latest_launch_cmd_time +
//  ext_game_status.stage_remain_time + 在线判断位】

namespace DartConfig
{
    void declareParameters(rclcpp::Node &node);
    void loadParametersfromMsg(rclcpp::Node &node, const info::msg::DartParam::SharedPtr msg);
};

#endif // DART_CONFIG_H