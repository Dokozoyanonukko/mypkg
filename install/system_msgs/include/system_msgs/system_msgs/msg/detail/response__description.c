// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from system_msgs:msg/Response.idl
// generated code does not contain a copyright notice

#include "system_msgs/msg/detail/response__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_type_hash_t *
system_msgs__msg__Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x58, 0xe1, 0xfb, 0x1e, 0x90, 0x2f, 0xa7, 0x90,
      0x3d, 0x50, 0xb5, 0xd9, 0x0a, 0x3f, 0x5c, 0x67,
      0x76, 0x98, 0x9d, 0x56, 0x42, 0x7d, 0x30, 0x30,
      0x2a, 0x2c, 0xbf, 0x1c, 0x92, 0xeb, 0x4e, 0xab,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char system_msgs__msg__Response__TYPE_NAME[] = "system_msgs/msg/Response";

// Define type names, field names, and default values
static char system_msgs__msg__Response__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field system_msgs__msg__Response__FIELDS[] = {
  {
    {system_msgs__msg__Response__FIELD_NAME__response, 8, 8},
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
system_msgs__msg__Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {system_msgs__msg__Response__TYPE_NAME, 24, 24},
      {system_msgs__msg__Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe3\\x83\\xa6\\xe3\\x83\\xbc\\xe3\\x82\\xb6\\xe5\\xbf\\x9c\\xe7\\xad\\x94\n"
  "uint8 response # 0:NO_RESPONSE, 1: TAKEN";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
system_msgs__msg__Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {system_msgs__msg__Response__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 49, 49},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
system_msgs__msg__Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *system_msgs__msg__Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
