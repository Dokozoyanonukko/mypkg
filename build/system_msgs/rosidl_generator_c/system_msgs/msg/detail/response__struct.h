// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from system_msgs:msg/Response.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/response.h"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RESPONSE__STRUCT_H_
#define SYSTEM_MSGS__MSG__DETAIL__RESPONSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Response in the package system_msgs.
/**
  * ユーザ応答
 */
typedef struct system_msgs__msg__Response
{
  /// 0:NO_RESPONSE, 1: TAKEN
  uint8_t response;
} system_msgs__msg__Response;

// Struct for a sequence of system_msgs__msg__Response.
typedef struct system_msgs__msg__Response__Sequence
{
  system_msgs__msg__Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} system_msgs__msg__Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_MSGS__MSG__DETAIL__RESPONSE__STRUCT_H_
