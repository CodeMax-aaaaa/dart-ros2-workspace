// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dart_msgs:msg/DartParam.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dart_msgs/msg/dart_param.h"


#ifndef DART_MSGS__MSG__DETAIL__DART_PARAM__STRUCT_H_
#define DART_MSGS__MSG__DETAIL__DART_PARAM__STRUCT_H_

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

/// Struct defined in msg/DartParam in the package dart_msgs.
typedef struct dart_msgs__msg__DartParam
{
  std_msgs__msg__Header header;
  int32_t primary_yaw;
  int32_t primary_yaw_offset;
  int32_t primary_force;
  int32_t primary_force_offset;
  int32_t auxiliary_yaw_offsets[4];
  int32_t auxiliary_force_offsets[4];
  uint8_t dart_launch_process_offset_begin;
  uint8_t dart_launch_process_offset_end;
  uint64_t last_param_update_time;
} dart_msgs__msg__DartParam;

// Struct for a sequence of dart_msgs__msg__DartParam.
typedef struct dart_msgs__msg__DartParam__Sequence
{
  dart_msgs__msg__DartParam * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dart_msgs__msg__DartParam__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DART_MSGS__MSG__DETAIL__DART_PARAM__STRUCT_H_
