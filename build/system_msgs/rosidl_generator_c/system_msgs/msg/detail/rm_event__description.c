// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from system_msgs:msg/RmEvent.idl
// generated code does not contain a copyright notice

#include "system_msgs/msg/detail/rm_event__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_type_hash_t *
system_msgs__msg__RmEvent__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3a, 0xd8, 0x05, 0x00, 0x2d, 0xc5, 0x40, 0x0f,
      0x19, 0xde, 0xb8, 0xdd, 0xab, 0xd4, 0x88, 0x20,
      0x72, 0xbf, 0xd3, 0x56, 0x0e, 0x56, 0x56, 0xce,
      0x5c, 0xb5, 0x4d, 0x3c, 0x9f, 0x88, 0xc6, 0x43,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char system_msgs__msg__RmEvent__TYPE_NAME[] = "system_msgs/msg/RmEvent";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char system_msgs__msg__RmEvent__FIELD_NAME__event[] = "event";
static char system_msgs__msg__RmEvent__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field system_msgs__msg__RmEvent__FIELDS[] = {
  {
    {system_msgs__msg__RmEvent__FIELD_NAME__event, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {system_msgs__msg__RmEvent__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription system_msgs__msg__RmEvent__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
system_msgs__msg__RmEvent__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {system_msgs__msg__RmEvent__TYPE_NAME, 23, 23},
      {system_msgs__msg__RmEvent__FIELDS, 2, 2},
    },
    {system_msgs__msg__RmEvent__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#\\xe6\\x9c\\x8d\\xe8\\x96\\xac\\xe3\\x82\\xa4\\xe3\\x83\\x99\\xe3\\x83\\xb3\\xe3\\x83\\x88\n"
  "uint8 event # 0:NONE, 1:MEDICATION_TIME\n"
  "builtin_interfaces/Time timestamp  # \\xe7\\x99\\xba\\xe7\\x94\\x9f\\xe6\\x99\\x82\\xe5\\x88\\xbb";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
system_msgs__msg__RmEvent__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {system_msgs__msg__RmEvent__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 90, 90},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
system_msgs__msg__RmEvent__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *system_msgs__msg__RmEvent__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
