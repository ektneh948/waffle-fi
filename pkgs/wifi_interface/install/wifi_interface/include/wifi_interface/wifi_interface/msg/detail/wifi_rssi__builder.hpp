// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wifi_interface:msg/WifiRssi.idl
// generated code does not contain a copyright notice

#ifndef WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__BUILDER_HPP_
#define WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wifi_interface/msg/detail/wifi_rssi__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wifi_interface
{

namespace msg
{

namespace builder
{

class Init_WifiRssi_ssid
{
public:
  explicit Init_WifiRssi_ssid(::wifi_interface::msg::WifiRssi & msg)
  : msg_(msg)
  {}
  ::wifi_interface::msg::WifiRssi ssid(::wifi_interface::msg::WifiRssi::_ssid_type arg)
  {
    msg_.ssid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wifi_interface::msg::WifiRssi msg_;
};

class Init_WifiRssi_rssi
{
public:
  explicit Init_WifiRssi_rssi(::wifi_interface::msg::WifiRssi & msg)
  : msg_(msg)
  {}
  Init_WifiRssi_ssid rssi(::wifi_interface::msg::WifiRssi::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return Init_WifiRssi_ssid(msg_);
  }

private:
  ::wifi_interface::msg::WifiRssi msg_;
};

class Init_WifiRssi_header
{
public:
  Init_WifiRssi_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WifiRssi_rssi header(::wifi_interface::msg::WifiRssi::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WifiRssi_rssi(msg_);
  }

private:
  ::wifi_interface::msg::WifiRssi msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wifi_interface::msg::WifiRssi>()
{
  return wifi_interface::msg::builder::Init_WifiRssi_header();
}

}  // namespace wifi_interface

#endif  // WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__BUILDER_HPP_
