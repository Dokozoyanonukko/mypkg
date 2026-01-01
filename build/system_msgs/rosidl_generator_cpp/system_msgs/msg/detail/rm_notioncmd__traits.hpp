// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from system_msgs:msg/RmNotioncmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_notioncmd.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__TRAITS_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "system_msgs/msg/detail/rm_notioncmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace system_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RmNotioncmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RmNotioncmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RmNotioncmd & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace system_msgs

namespace rosidl_generator_traits
{

[[deprecated("use system_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const system_msgs::msg::RmNotioncmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  system_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use system_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const system_msgs::msg::RmNotioncmd & msg)
{
  return system_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<system_msgs::msg::RmNotioncmd>()
{
  return "system_msgs::msg::RmNotioncmd";
}

template<>
inline const char * name<system_msgs::msg::RmNotioncmd>()
{
  return "system_msgs/msg/RmNotioncmd";
}

template<>
struct has_fixed_size<system_msgs::msg::RmNotioncmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<system_msgs::msg::RmNotioncmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<system_msgs::msg::RmNotioncmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_NOTIONCMD__TRAITS_HPP_
