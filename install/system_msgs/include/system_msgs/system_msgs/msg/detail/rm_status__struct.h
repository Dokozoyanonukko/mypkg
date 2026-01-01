// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from system_msgs:msg/RmStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_status.h"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__STRUCT_H_
#define SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/RmStatus in the package system_msgs.
/**
  * 通知結果
 */
typedef struct system_msgs__msg__RmStatus
{
  /// 0: SUCCESS, 1: FAILURE
  uint8_t status;
} system_msgs__msg__RmStatus;

// Struct for a sequence of system_msgs__msg__RmStatus.
typedef struct system_msgs__msg__RmStatus__Sequence
{
  system_msgs__msg__RmStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} system_msgs__msg__RmStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__STRUCT_H_
