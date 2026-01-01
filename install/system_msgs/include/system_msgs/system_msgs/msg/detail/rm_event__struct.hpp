// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from system_msgs:msg/RmEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_event.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__STRUCT_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__system_msgs__msg__RmEvent __attribute__((deprecated))
#else
# define DEPRECATED__system_msgs__msg__RmEvent __declspec(deprecated)
#endif

namespace system_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RmEvent_
{
  using Type = RmEvent_<ContainerAllocator>;

  explicit RmEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event = 0;
    }
  }

  explicit RmEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event = 0;
    }
  }

  // field types and members
  using _event_type =
    uint8_t;
  _event_type event;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__event(
    const uint8_t & _arg)
  {
    this->event = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    system_msgs::msg::RmEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const system_msgs::msg::RmEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<system_msgs::msg::RmEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<system_msgs::msg::RmEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      system_msgs::msg::RmEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<system_msgs::msg::RmEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      system_msgs::msg::RmEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<system_msgs::msg::RmEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<system_msgs::msg::RmEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<system_msgs::msg::RmEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__system_msgs__msg__RmEvent
    std::shared_ptr<system_msgs::msg::RmEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__system_msgs__msg__RmEvent
    std::shared_ptr<system_msgs::msg::RmEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RmEvent_ & other) const
  {
    if (this->event != other.event) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const RmEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RmEvent_

// alias to use template instance with default allocator
using RmEvent =
  system_msgs::msg::RmEvent_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace system_msgs

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__STRUCT_HPP_
