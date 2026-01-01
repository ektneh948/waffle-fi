// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wifi_interface:msg/WifiFused.idl
// generated code does not contain a copyright notice

#ifndef WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__TRAITS_HPP_
#define WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wifi_interface/msg/detail/wifi_fused__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace wifi_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const WifiFused & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: rssi
  {
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << ", ";
  }

  // member: ssid
  {
    out << "ssid: ";
    rosidl_generator_traits::value_to_yaml(msg.ssid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WifiFused & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << "\n";
  }

  // member: ssid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ssid: ";
    rosidl_generator_traits::value_to_yaml(msg.ssid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WifiFused & msg, bool use_flow_style = false)
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

}  // namespace wifi_interface

namespace rosidl_generator_traits
{

[[deprecated("use wifi_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wifi_interface::msg::WifiFused & msg,
  std::ostream & out, size_t indentation = 0)
{
  wifi_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wifi_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const wifi_interface::msg::WifiFused & msg)
{
  return wifi_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wifi_interface::msg::WifiFused>()
{
  return "wifi_interface::msg::WifiFused";
}

template<>
inline const char * name<wifi_interface::msg::WifiFused>()
{
  return "wifi_interface/msg/WifiFused";
}

template<>
struct has_fixed_size<wifi_interface::msg::WifiFused>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wifi_interface::msg::WifiFused>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wifi_interface::msg::WifiFused>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__TRAITS_HPP_
