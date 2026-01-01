// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wifi_interface:msg/WifiFused.idl
// generated code does not contain a copyright notice

#ifndef WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__STRUCT_HPP_
#define WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wifi_interface__msg__WifiFused __attribute__((deprecated))
#else
# define DEPRECATED__wifi_interface__msg__WifiFused __declspec(deprecated)
#endif

namespace wifi_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WifiFused_
{
  using Type = WifiFused_<ContainerAllocator>;

  explicit WifiFused_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->rssi = 0l;
      this->ssid = "";
    }
  }

  explicit WifiFused_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    ssid(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->rssi = 0l;
      this->ssid = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _rssi_type =
    int32_t;
  _rssi_type rssi;
  using _ssid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ssid_type ssid;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__rssi(
    const int32_t & _arg)
  {
    this->rssi = _arg;
    return *this;
  }
  Type & set__ssid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ssid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wifi_interface::msg::WifiFused_<ContainerAllocator> *;
  using ConstRawPtr =
    const wifi_interface::msg::WifiFused_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wifi_interface::msg::WifiFused_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wifi_interface::msg::WifiFused_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wifi_interface::msg::WifiFused_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wifi_interface::msg::WifiFused_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wifi_interface::msg::WifiFused_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wifi_interface::msg::WifiFused_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wifi_interface::msg::WifiFused_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wifi_interface::msg::WifiFused_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wifi_interface__msg__WifiFused
    std::shared_ptr<wifi_interface::msg::WifiFused_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wifi_interface__msg__WifiFused
    std::shared_ptr<wifi_interface::msg::WifiFused_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WifiFused_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->rssi != other.rssi) {
      return false;
    }
    if (this->ssid != other.ssid) {
      return false;
    }
    return true;
  }
  bool operator!=(const WifiFused_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WifiFused_

// alias to use template instance with default allocator
using WifiFused =
  wifi_interface::msg::WifiFused_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wifi_interface

#endif  // WIFI_INTERFACE__MSG__DETAIL__WIFI_FUSED__STRUCT_HPP_
