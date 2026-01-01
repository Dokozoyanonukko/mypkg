// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from system_msgs:msg/Response.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/response.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RESPONSE__STRUCT_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RESPONSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__system_msgs__msg__Response __attribute__((deprecated))
#else
# define DEPRECATED__system_msgs__msg__Response __declspec(deprecated)
#endif

namespace system_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Response_
{
  using Type = Response_<ContainerAllocator>;

  explicit Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = 0;
    }
  }

  explicit Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = 0;
    }
  }

  // field types and members
  using _response_type =
    uint8_t;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const uint8_t & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    system_msgs::msg::Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const system_msgs::msg::Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<system_msgs::msg::Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<system_msgs::msg::Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      system_msgs::msg::Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<system_msgs::msg::Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      system_msgs::msg::Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<system_msgs::msg::Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<system_msgs::msg::Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<system_msgs::msg::Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__system_msgs__msg__Response
    std::shared_ptr<system_msgs::msg::Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__system_msgs__msg__Response
    std::shared_ptr<system_msgs::msg::Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Response_

// alias to use template instance with default allocator
using Response =
  system_msgs::msg::Response_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace system_msgs

#endif  // SYSTEM_MSGS__MSG__DETAIL__RESPONSE__STRUCT_HPP_
