// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from system_msgs:msg/RmEvent.idl
// generated code does not contain a copyright notice
#include "system_msgs/msg/detail/rm_event__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
system_msgs__msg__RmEvent__init(system_msgs__msg__RmEvent * msg)
{
  if (!msg) {
    return false;
  }
  // event
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    system_msgs__msg__RmEvent__fini(msg);
    return false;
  }
  return true;
}

void
system_msgs__msg__RmEvent__fini(system_msgs__msg__RmEvent * msg)
{
  if (!msg) {
    return;
  }
  // event
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
}

bool
system_msgs__msg__RmEvent__are_equal(const system_msgs__msg__RmEvent * lhs, const system_msgs__msg__RmEvent * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // event
  if (lhs->event != rhs->event) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  return true;
}

bool
system_msgs__msg__RmEvent__copy(
  const system_msgs__msg__RmEvent * input,
  system_msgs__msg__RmEvent * output)
{
  if (!input || !output) {
    return false;
  }
  // event
  output->event = input->event;
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  return true;
}

system_msgs__msg__RmEvent *
system_msgs__msg__RmEvent__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  system_msgs__msg__RmEvent * msg = (system_msgs__msg__RmEvent *)allocator.allocate(sizeof(system_msgs__msg__RmEvent), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(system_msgs__msg__RmEvent));
  bool success = system_msgs__msg__RmEvent__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
system_msgs__msg__RmEvent__destroy(system_msgs__msg__RmEvent * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    system_msgs__msg__RmEvent__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
system_msgs__msg__RmEvent__Sequence__init(system_msgs__msg__RmEvent__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  system_msgs__msg__RmEvent * data = NULL;

  if (size) {
    data = (system_msgs__msg__RmEvent *)allocator.zero_allocate(size, sizeof(system_msgs__msg__RmEvent), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = system_msgs__msg__RmEvent__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        system_msgs__msg__RmEvent__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
system_msgs__msg__RmEvent__Sequence__fini(system_msgs__msg__RmEvent__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      system_msgs__msg__RmEvent__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

system_msgs__msg__RmEvent__Sequence *
system_msgs__msg__RmEvent__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  system_msgs__msg__RmEvent__Sequence * array = (system_msgs__msg__RmEvent__Sequence *)allocator.allocate(sizeof(system_msgs__msg__RmEvent__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = system_msgs__msg__RmEvent__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
system_msgs__msg__RmEvent__Sequence__destroy(system_msgs__msg__RmEvent__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    system_msgs__msg__RmEvent__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
system_msgs__msg__RmEvent__Sequence__are_equal(const system_msgs__msg__RmEvent__Sequence * lhs, const system_msgs__msg__RmEvent__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!system_msgs__msg__RmEvent__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
system_msgs__msg__RmEvent__Sequence__copy(
  const system_msgs__msg__RmEvent__Sequence * input,
  system_msgs__msg__RmEvent__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(system_msgs__msg__RmEvent);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    system_msgs__msg__RmEvent * data =
      (system_msgs__msg__RmEvent *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!system_msgs__msg__RmEvent__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          system_msgs__msg__RmEvent__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!system_msgs__msg__RmEvent__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
