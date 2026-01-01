// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from system_msgs:msg/RmEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_event.h"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__STRUCT_H_
#define SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/RmEvent in the package system_msgs.
/**
  * 服薬イベント
 */
typedef struct system_msgs__msg__RmEvent
{
  /// 0:NONE, 1:MEDICATION_TIME
  uint8_t event;
  /// 発生時刻
  builtin_interfaces__msg__Time timestamp;
} system_msgs__msg__RmEvent;

// Struct for a sequence of system_msgs__msg__RmEvent.
typedef struct system_msgs__msg__RmEvent__Sequence
{
  system_msgs__msg__RmEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} system_msgs__msg__RmEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__STRUCT_H_
