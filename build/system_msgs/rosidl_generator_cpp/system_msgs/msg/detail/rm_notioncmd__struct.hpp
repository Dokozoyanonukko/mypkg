// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from system_msgs:msg/RmNotioncmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_notioncmd.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__STRUCT_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__system_msgs__msg__RmNotioncmd __attribute__((deprecated))
#else
# define DEPRECATED__system_msgs__msg__RmNotioncmd __declspec(deprecated)
#endif

namespace system_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RmNotioncmd_
{
  using Type = RmNotioncmd_<ContainerAllocator>;

  explicit RmNotioncmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  explicit RmNotioncmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  // field types and members
  using _command_type =
    uint8_t;
  _command_type command;

  // setters for named parameter idiom
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    system_msgs::msg::RmNotioncmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const system_msgs::msg::RmNotioncmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<system_msgs::msg::RmNotioncmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<system_msgs::msg::RmNotioncmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      system_msgs::msg::RmNotioncmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<system_msgs::msg::RmNotioncmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      system_msgs::msg::RmNotioncmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<system_msgs::msg::RmNotioncmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<system_msgs::msg::RmNotioncmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<system_msgs::msg::RmNotioncmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__system_msgs__msg__RmNotioncmd
    std::shared_ptr<system_msgs::msg::RmNotioncmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__system_msgs__msg__RmNotioncmd
    std::shared_ptr<system_msgs::msg::RmNotioncmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RmNotioncmd_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const RmNotioncmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RmNotioncmd_

// alias to use template instance with default allocator
using RmNotioncmd =
  system_msgs::msg::RmNotioncmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace system_msgs

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__STRUCT_HPP_
