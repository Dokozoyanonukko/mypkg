// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from system_msgs:msg/RmEvent.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "system_msgs/msg/detail/rm_event__rosidl_typesupport_introspection_c.h"
#include "system_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "system_msgs/msg/detail/rm_event__functions.h"
#include "system_msgs/msg/detail/rm_event__struct.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  system_msgs__msg__RmEvent__init(message_memory);
}

void system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_fini_function(void * message_memory)
{
  system_msgs__msg__RmEvent__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_member_array[2] = {
  {
    "event",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(system_msgs__msg__RmEvent, event),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(system_msgs__msg__RmEvent, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_members = {
  "system_msgs__msg",  // message namespace
  "RmEvent",  // message name
  2,  // number of fields
  sizeof(system_msgs__msg__RmEvent),
  false,  // has_any_key_member_
  system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_member_array,  // message members
  system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_init_function,  // function to initialize message memory (memory has to be allocated)
  system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_type_support_handle = {
  0,
  &system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_members,
  get_message_typesupport_handle_function,
  &system_msgs__msg__RmEvent__get_type_hash,
  &system_msgs__msg__RmEvent__get_type_description,
  &system_msgs__msg__RmEvent__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_system_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, system_msgs, msg, RmEvent)() {
  system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_type_support_handle.typesupport_identifier) {
    system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &system_msgs__msg__RmEvent__rosidl_typesupport_introspection_c__RmEvent_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
