// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dart_msgs:msg/Judge.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dart_msgs/msg/judge.h"


#ifndef DART_MSGS__MSG__DETAIL__JUDGE__STRUCT_H_
#define DART_MSGS__MSG__DETAIL__JUDGE__STRUCT_H_

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

/// Struct defined in msg/Judge in the package dart_msgs.
typedef struct dart_msgs__msg__Judge
{
  std_msgs__msg__Header header;
  uint8_t dart_launch_opening_status;
  uint8_t game_progress;
  uint8_t dart_remaining_time;
  uint16_t latest_launch_cmd_time;
  uint16_t stage_remain_time;
  bool judge_online;
} dart_msgs__msg__Judge;

// Struct for a sequence of dart_msgs__msg__Judge.
typedef struct dart_msgs__msg__Judge__Sequence
{
  dart_msgs__msg__Judge * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dart_msgs__msg__Judge__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DART_MSGS__MSG__DETAIL__JUDGE__STRUCT_H_
