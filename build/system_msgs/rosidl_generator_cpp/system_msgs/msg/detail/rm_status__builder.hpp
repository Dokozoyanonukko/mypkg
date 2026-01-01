// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from system_msgs:msg/RmStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_status.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__BUILDER_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "system_msgs/msg/detail/rm_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace system_msgs
{

namespace msg
{

namespace builder
{

class Init_RmStatus_status
{
public:
  Init_RmStatus_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::system_msgs::msg::RmStatus status(::system_msgs::msg::RmStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::system_msgs::msg::RmStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::system_msgs::msg::RmStatus>()
{
  return system_msgs::msg::builder::Init_RmStatus_status();
}

}  // namespace system_msgs

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_STATUS__BUILDER_HPP_
