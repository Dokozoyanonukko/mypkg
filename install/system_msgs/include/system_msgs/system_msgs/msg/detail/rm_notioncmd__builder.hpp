// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from system_msgs:msg/RmNotioncmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_notioncmd.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__BUILDER_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "system_msgs/msg/detail/rm_notioncmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace system_msgs
{

namespace msg
{

namespace builder
{

class Init_RmNotioncmd_command
{
public:
  Init_RmNotioncmd_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::system_msgs::msg::RmNotioncmd command(::system_msgs::msg::RmNotioncmd::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::system_msgs::msg::RmNotioncmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::system_msgs::msg::RmNotioncmd>()
{
  return system_msgs::msg::builder::Init_RmNotioncmd_command();
}

}  // namespace system_msgs

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__BUILDER_HPP_
