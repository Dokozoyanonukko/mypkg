// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from system_msgs:msg/RmStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_status.h"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__FUNCTIONS_H_
#define SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__FUNCTIONS_H_

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

#include "system_msgs/msg/detail/rm_status__struct.h"

/// Initialize msg/RmStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * system_msgs__msg__RmStatus
 * )) before or use
 * system_msgs__msg__RmStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmStatus__init(system_msgs__msg__RmStatus * msg);

/// Finalize msg/RmStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
void
system_msgs__msg__RmStatus__fini(system_msgs__msg__RmStatus * msg);

/// Create msg/RmStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * system_msgs__msg__RmStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
system_msgs__msg__RmStatus *
system_msgs__msg__RmStatus__create(void);

/// Destroy msg/RmStatus message.
/**
 * It calls
 * system_msgs__msg__RmStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
void
system_msgs__msg__RmStatus__destroy(system_msgs__msg__RmStatus * msg);

/// Check for msg/RmStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmStatus__are_equal(const system_msgs__msg__RmStatus * lhs, const system_msgs__msg__RmStatus * rhs);

/// Copy a msg/RmStatus message.
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
system_msgs__msg__RmStatus__copy(
  const system_msgs__msg__RmStatus * input,
  system_msgs__msg__RmStatus * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_type_hash_t *
system_msgs__msg__RmStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_runtime_c__type_description__TypeDescription *
system_msgs__msg__RmStatus__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_runtime_c__type_description__TypeSource *
system_msgs__msg__RmStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
system_msgs__msg__RmStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/RmStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * system_msgs__msg__RmStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmStatus__Sequence__init(system_msgs__msg__RmStatus__Sequence * array, size_t size);

/// Finalize array of msg/RmStatus messages.
/**
 * It calls
 * system_msgs__msg__RmStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
void
system_msgs__msg__RmStatus__Sequence__fini(system_msgs__msg__RmStatus__Sequence * array);

/// Create array of msg/RmStatus messages.
/**
 * It allocates the memory for the array and calls
 * system_msgs__msg__RmStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
system_msgs__msg__RmStatus__Sequence *
system_msgs__msg__RmStatus__Sequence__create(size_t size);

/// Destroy array of msg/RmStatus messages.
/**
 * It calls
 * system_msgs__msg__RmStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
void
system_msgs__msg__RmStatus__Sequence__destroy(system_msgs__msg__RmStatus__Sequence * array);

/// Check for msg/RmStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_system_msgs
bool
system_msgs__msg__RmStatus__Sequence__are_equal(const system_msgs__msg__RmStatus__Sequence * lhs, const system_msgs__msg__RmStatus__Sequence * rhs);

/// Copy an array of msg/RmStatus messages.
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
system_msgs__msg__RmStatus__Sequence__copy(
  const system_msgs__msg__RmStatus__Sequence * input,
  system_msgs__msg__RmStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__FUNCTIONS_H_
