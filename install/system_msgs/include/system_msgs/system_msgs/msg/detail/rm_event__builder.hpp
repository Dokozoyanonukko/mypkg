// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from system_msgs:msg/RmEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_event.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__BUILDER_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "system_msgs/msg/detail/rm_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace system_msgs
{

namespace msg
{

namespace builder
{

class Init_RmEvent_timestamp
{
public:
  explicit Init_RmEvent_timestamp(::system_msgs::msg::RmEvent & msg)
  : msg_(msg)
  {}
  ::system_msgs::msg::RmEvent timestamp(::system_msgs::msg::RmEvent::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::system_msgs::msg::RmEvent msg_;
};

class Init_RmEvent_event
{
public:
  Init_RmEvent_event()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RmEvent_timestamp event(::system_msgs::msg::RmEvent::_event_type arg)
  {
    msg_.event = std::move(arg);
    return Init_RmEvent_timestamp(msg_);
  }

private:
  ::system_msgs::msg::RmEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::system_msgs::msg::RmEvent>()
{
  return system_msgs::msg::builder::Init_RmEvent_event();
}

}  // namespace system_msgs

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__BUILDER_HPP_
