// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wifi_interface:msg/WifiRssi.idl
// generated code does not contain a copyright notice

#ifndef WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__TRAITS_HPP_
#define WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wifi_interface/msg/detail/wifi_rssi__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace wifi_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const WifiRssi & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
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
  const WifiRssi & msg,
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

inline std::string to_yaml(const WifiRssi & msg, bool use_flow_style = false)
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
  const wifi_interface::msg::WifiRssi & msg,
  std::ostream & out, size_t indentation = 0)
{
  wifi_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wifi_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const wifi_interface::msg::WifiRssi & msg)
{
  return wifi_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wifi_interface::msg::WifiRssi>()
{
  return "wifi_interface::msg::WifiRssi";
}

template<>
inline const char * name<wifi_interface::msg::WifiRssi>()
{
  return "wifi_interface/msg/WifiRssi";
}

template<>
struct has_fixed_size<wifi_interface::msg::WifiRssi>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wifi_interface::msg::WifiRssi>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wifi_interface::msg::WifiRssi>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__TRAITS_HPP_
