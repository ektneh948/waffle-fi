// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wifi_interface:msg/WifiFused.idl
// generated code does not contain a copyright notice

#ifndef WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__BUILDER_HPP_
#define WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wifi_interface/msg/detail/wifi_fused__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wifi_interface
{

namespace msg
{

namespace builder
{

class Init_WifiFused_ssid
{
public:
  explicit Init_WifiFused_ssid(::wifi_interface::msg::WifiFused & msg)
  : msg_(msg)
  {}
  ::wifi_interface::msg::WifiFused ssid(::wifi_interface::msg::WifiFused::_ssid_type arg)
  {
    msg_.ssid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wifi_interface::msg::WifiFused msg_;
};

class Init_WifiFused_rssi
{
public:
  explicit Init_WifiFused_rssi(::wifi_interface::msg::WifiFused & msg)
  : msg_(msg)
  {}
  Init_WifiFused_ssid rssi(::wifi_interface::msg::WifiFused::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return Init_WifiFused_ssid(msg_);
  }

private:
  ::wifi_interface::msg::WifiFused msg_;
};

class Init_WifiFused_y
{
public:
  explicit Init_WifiFused_y(::wifi_interface::msg::WifiFused & msg)
  : msg_(msg)
  {}
  Init_WifiFused_rssi y(::wifi_interface::msg::WifiFused::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_WifiFused_rssi(msg_);
  }

private:
  ::wifi_interface::msg::WifiFused msg_;
};

class Init_WifiFused_x
{
public:
  explicit Init_WifiFused_x(::wifi_interface::msg::WifiFused & msg)
  : msg_(msg)
  {}
  Init_WifiFused_y x(::wifi_interface::msg::WifiFused::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_WifiFused_y(msg_);
  }

private:
  ::wifi_interface::msg::WifiFused msg_;
};

class Init_WifiFused_header
{
public:
  Init_WifiFused_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WifiFused_x header(::wifi_interface::msg::WifiFused::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WifiFused_x(msg_);
  }

private:
  ::wifi_interface::msg::WifiFused msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wifi_interface::msg::WifiFused>()
{
  return wifi_interface::msg::builder::Init_WifiFused_header();
}

}  // namespace wifi_interface

#endif  // WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__BUILDER_HPP_
