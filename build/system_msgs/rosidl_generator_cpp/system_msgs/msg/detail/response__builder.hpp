// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from system_msgs:msg/Response.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/response.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RESPONSE__BUILDER_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "system_msgs/msg/detail/response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace system_msgs
{

namespace msg
{

namespace builder
{

class Init_Response_response
{
public:
  Init_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::system_msgs::msg::Response response(::system_msgs::msg::Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::system_msgs::msg::Response msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::system_msgs::msg::Response>()
{
  return system_msgs::msg::builder::Init_Response_response();
}

}  // namespace system_msgs

#endif  // SYSTEM_MSGS__MSG__DETAIL__RESPONSE__BUILDER_HPP_
