// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from system_msgs:msg/RmNotioncmd.idl
// generated code does not contain a copyright notice

#include "system_msgs/msg/detail/rm_notioncmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_type_hash_t *
system_msgs__msg__RmNotioncmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xba, 0x07, 0x1d, 0x1f, 0x56, 0xc5, 0x4d, 0x6c,
      0xab, 0xdf, 0x06, 0x92, 0x1c, 0x34, 0xc7, 0xe4,
      0x1a, 0x49, 0x48, 0xfd, 0x0e, 0x29, 0x0c, 0x40,
      0x08, 0x28, 0xd4, 0xa9, 0xd5, 0xd8, 0x9a, 0x11,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char system_msgs__msg__RmNotioncmd__TYPE_NAME[] = "system_msgs/msg/RmNotioncmd";

// Define type names, field names, and default values
static char system_msgs__msg__RmNotioncmd__FIELD_NAME__command[] = "command";

static rosidl_runtime_c__type_description__Field system_msgs__msg__RmNotioncmd__FIELDS[] = {
  {
    {system_msgs__msg__RmNotioncmd__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
system_msgs__msg__RmNotioncmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {system_msgs__msg__RmNotioncmd__TYPE_NAME, 27, 27},
      {system_msgs__msg__RmNotioncmd__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#\\xe9\\x80\\x9a\\xe7\\x9f\\xa5\\xe3\\x82\\xb3\\xe3\\x83\\x9e\\xe3\\x83\\xb3\\xe3\\x83\\x89\n"
  "uint8 command # 0: OFF, 1: ON";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
system_msgs__msg__RmNotioncmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {system_msgs__msg__RmNotioncmd__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 38, 38},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
system_msgs__msg__RmNotioncmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *system_msgs__msg__RmNotioncmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
