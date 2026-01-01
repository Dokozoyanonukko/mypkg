// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from system_msgs:msg/RmEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_event.h"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__FUNCTIONS_H_
#define SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "system_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "system_msgs/msg/detail/rm_event__struct.h"

/// Initialize msg/RmEvent message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * system_msgs__msg__RmEvent
 * )) before or use
 * system_msgs__msg__RmEvent__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmEvent__init(system_msgs__msg__RmEvent * msg);

/// Finalize msg/RmEvent message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
void
system_msgs__msg__RmEvent__fini(system_msgs__msg__RmEvent * msg);

/// Create msg/RmEvent message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * system_msgs__msg__RmEvent__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
system_msgs__msg__RmEvent *
system_msgs__msg__RmEvent__create(void);

/// Destroy msg/RmEvent message.
/**
 * It calls
 * system_msgs__msg__RmEvent__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
void
system_msgs__msg__RmEvent__destroy(system_msgs__msg__RmEvent * msg);

/// Check for msg/RmEvent message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmEvent__are_equal(const system_msgs__msg__RmEvent * lhs, const system_msgs__msg__RmEvent * rhs);

/// Copy a msg/RmEvent message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmEvent__copy(
  const system_msgs__msg__RmEvent * input,
  system_msgs__msg__RmEvent * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_type_hash_t *
system_msgs__msg__RmEvent__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_runtime_c__type_description__TypeDescription *
system_msgs__msg__RmEvent__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_runtime_c__type_description__TypeSource *
system_msgs__msg__RmEvent__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
system_msgs__msg__RmEvent__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/RmEvent messages.
/**
 * It allocates the memory for the number of elements and calls
 * system_msgs__msg__RmEvent__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmEvent__Sequence__init(system_msgs__msg__RmEvent__Sequence * array, size_t size);

/// Finalize array of msg/RmEvent messages.
/**
 * It calls
 * system_msgs__msg__RmEvent__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
void
system_msgs__msg__RmEvent__Sequence__fini(system_msgs__msg__RmEvent__Sequence * array);

/// Create array of msg/RmEvent messages.
/**
 * It allocates the memory for the array and calls
 * system_msgs__msg__RmEvent__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
system_msgs__msg__RmEvent__Sequence *
system_msgs__msg__RmEvent__Sequence__create(size_t size);

/// Destroy array of msg/RmEvent messages.
/**
 * It calls
 * system_msgs__msg__RmEvent__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
void
system_msgs__msg__RmEvent__Sequence__destroy(system_msgs__msg__RmEvent__Sequence * array);

/// Check for msg/RmEvent message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmEvent__Sequence__are_equal(const system_msgs__msg__RmEvent__Sequence * lhs, const system_msgs__msg__RmEvent__Sequence * rhs);

/// Copy an array of msg/RmEvent messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmEvent__Sequence__copy(
  const system_msgs__msg__RmEvent__Sequence * input,
  system_msgs__msg__RmEvent__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__FUNCTIONS_H_
