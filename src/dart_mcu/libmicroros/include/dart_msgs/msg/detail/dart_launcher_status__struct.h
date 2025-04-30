// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dart_msgs:msg/DartLauncherStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dart_msgs/msg/dart_launcher_status.h"


#ifndef DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__STRUCT_H_
#define DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/DartLauncherStatus in the package dart_msgs.
typedef struct dart_msgs__msg__DartLauncherStatus
{
  std_msgs__msg__Header header;
  bool motor_ls_online;
  bool motor_y_online;
  bool motor_dm_online;
  bool judge_online;
  bool motor_fw_online[4];
  bool rc_online;
  bool dart_launcher_online;
  /// boot = 100, protect = 101, remote = 102, match = 103-106 Enter Wait Launch Reload, undefined=255
  uint8_t dart_state;
  bool motor_y_resetting;
  bool motor_dm_resetting;
  bool motor_ls_resetting;
  uint8_t dart_launch_process;
  int32_t motor_y_angle;
  int32_t motor_fw_velocity[4];
  double motor_fw_current[4];
  double bus_voltage[4];
} dart_msgs__msg__DartLauncherStatus;

// Struct for a sequence of dart_msgs__msg__DartLauncherStatus.
typedef struct dart_msgs__msg__DartLauncherStatus__Sequence
{
  dart_msgs__msg__DartLauncherStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dart_msgs__msg__DartLauncherStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__STRUCT_H_
