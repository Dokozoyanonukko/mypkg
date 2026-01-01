// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from system_msgs:msg/Response.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "system_msgs/msg/response.hpp"


#ifndef SYSTEM_MSGS__MSG__DETAIL__RESPONSE__TRAITS_HPP_
#define SYSTEM_MSGS__MSG__DETAIL__RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "system_msgs/msg/detail/response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace system_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Response & msg, bool use_flow_style = false)
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
  const system_msgs::msg::Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  system_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use system_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const system_msgs::msg::Response & msg)
{
  return system_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<system_msgs::msg::Response>()
{
  return "system_msgs::msg::Response";
}

template<>
inline const char * name<system_msgs::msg::Response>()
{
  return "system_msgs/msg/Response";
}

template<>
struct has_fixed_size<system_msgs::msg::Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<system_msgs::msg::Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<system_msgs::msg::Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SYSTEM_MSGS__MSG__DETAIL__RESPONSE__TRAITS_HPP_
