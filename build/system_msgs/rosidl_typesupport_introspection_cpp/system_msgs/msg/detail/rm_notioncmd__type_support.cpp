// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from system_msgs:msg/RmNotioncmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "system_msgs/msg/detail/rm_notioncmd__functions.h"
#include "system_msgs/msg/detail/rm_notioncmd__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace system_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RmNotioncmd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) system_msgs::msg::RmNotioncmd(_init);
}

void RmNotioncmd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<system_msgs::msg::RmNotioncmd *>(message_memory);
  typed_message->~RmNotioncmd();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RmNotioncmd_message_member_array[1] = {
  {
    "command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(system_msgs::msg::RmNotioncmd, command),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RmNotioncmd_message_members = {
  "system_msgs::msg",  // message namespace
  "RmNotioncmd",  // message name
  1,  // number of fields
  sizeof(system_msgs::msg::RmNotioncmd),
  false,  // has_any_key_member_
  RmNotioncmd_message_member_array,  // message members
  RmNotioncmd_init_function,  // function to initialize message memory (memory has to be allocated)
  RmNotioncmd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RmNotioncmd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RmNotioncmd_message_members,
  get_message_typesupport_handle_function,
  &system_msgs__msg__RmNotioncmd__get_type_hash,
  &system_msgs__msg__RmNotioncmd__get_type_description,
  &system_msgs__msg__RmNotioncmd__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace system_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<system_msgs::msg::RmNotioncmd>()
{
  return &::system_msgs::msg::rosidl_typesupport_introspection_cpp::RmNotioncmd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, system_msgs, msg, RmNotioncmd)() {
  return &::system_msgs::msg::rosidl_typesupport_introspection_cpp::RmNotioncmd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
