// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from system_msgs:msg/RmStatus.idl
// generated code does not contain a copyright notice

#include "system_msgs/msg/detail/rm_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_type_hash_t *
system_msgs__msg__RmStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe7, 0xe9, 0xf5, 0x0b, 0x90, 0x1a, 0x50, 0x65,
      0xf2, 0x97, 0xb4, 0x72, 0xce, 0x2d, 0xbc, 0x10,
      0xb7, 0x2d, 0xcc, 0xb2, 0xad, 0x9a, 0x5c, 0x5e,
      0x74, 0x31, 0xd9, 0x72, 0xec, 0xcd, 0x36, 0x4e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char system_msgs__msg__RmStatus__TYPE_NAME[] = "system_msgs/msg/RmStatus";

// Define type names, field names, and default values
static char system_msgs__msg__RmStatus__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field system_msgs__msg__RmStatus__FIELDS[] = {
  {
    {system_msgs__msg__RmStatus__FIELD_NAME__status, 6, 6},
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
system_msgs__msg__RmStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {system_msgs__msg__RmStatus__TYPE_NAME, 24, 24},
      {system_msgs__msg__RmStatus__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe9\\x80\\x9a\\xe7\\x9f\\xa5\\xe7\\xb5\\x90\\xe6\\x9e\\x9c\n"
  "uint8 status # 0: SUCCESS, 1: FAILURE";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
system_msgs__msg__RmStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {system_msgs__msg__RmStatus__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 45, 45},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
system_msgs__msg__RmStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *system_msgs__msg__RmStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
