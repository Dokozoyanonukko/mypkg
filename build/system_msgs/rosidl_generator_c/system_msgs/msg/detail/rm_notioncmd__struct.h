// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from system_msgs:msg/RmNotioncmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_notioncmd.h"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__STRUCT_H_
#define SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/RmNotioncmd in the package system_msgs.
/**
  * 通知コマンド
 */
typedef struct system_msgs__msg__RmNotioncmd
{
  /// 0: OFF, 1: ON
  uint8_t command;
} system_msgs__msg__RmNotioncmd;

// Struct for a sequence of system_msgs__msg__RmNotioncmd.
typedef struct system_msgs__msg__RmNotioncmd__Sequence
{
  system_msgs__msg__RmNotioncmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} system_msgs__msg__RmNotioncmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__STRUCT_H_
