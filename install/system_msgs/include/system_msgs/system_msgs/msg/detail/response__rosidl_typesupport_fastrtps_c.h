// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from system_msgs:msg/Response.idl
// generated code does not contain a copyright notice
#ifndef SYSTEM_MSGS__MSG__DETAIL__RESPONSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define SYSTEM_MSGS__MSG__DETAIL__RESPONSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "system_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "system_msgs/msg/detail/response__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_system_msgs
bool cdr_serialize_system_msgs__msg__Response(
  const system_msgs__msg__Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_system_msgs
bool cdr_deserialize_system_msgs__msg__Response(
  eprosima::fastcdr::Cdr &,
  system_msgs__msg__Response * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_system_msgs
size_t get_serialized_size_system_msgs__msg__Response(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_system_msgs
size_t max_serialized_size_system_msgs__msg__Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_system_msgs
bool cdr_serialize_key_system_msgs__msg__Response(
  const system_msgs__msg__Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_system_msgs
size_t get_serialized_size_key_system_msgs__msg__Response(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_system_msgs
size_t max_serialized_size_key_system_msgs__msg__Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_system_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, system_msgs, msg, Response)();

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_MSGS__MSG__DETAIL__RESPONSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
