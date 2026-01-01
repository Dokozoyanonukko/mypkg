// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from system_msgs:msg/RmEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/rm_event.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__TRAITS_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "system_msgs/msg/detail/rm_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace system_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RmEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: event
  {
    out << "event: ";
    rosidl_generator_traits::value_to_yaml(msg.event, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RmEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: event
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "event: ";
    rosidl_generator_traits::value_to_yaml(msg.event, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RmEvent & msg, bool use_flow_style = false)
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
  const system_msgs::msg::RmEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  system_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use system_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const system_msgs::msg::RmEvent & msg)
{
  return system_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<system_msgs::msg::RmEvent>()
{
  return "system_msgs::msg::RmEvent";
}

template<>
inline const char * name<system_msgs::msg::RmEvent>()
{
  return "system_msgs/msg/RmEvent";
}

template<>
struct has_fixed_size<system_msgs::msg::RmEvent>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<system_msgs::msg::RmEvent>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<system_msgs::msg::RmEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYSTEM_MSGS__MSG__DETAIL__RM_EVENT__TRAITS_HPP_
