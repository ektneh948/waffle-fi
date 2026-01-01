// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wifi_interface:msg/WifiRssi.idl
// generated code does not contain a copyright notice

#ifndef WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__STRUCT_H_
#define WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'ssid'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/WifiRssi in the package wifi_interface.
typedef struct wifi_interface__msg__WifiRssi
{
  std_msgs__msg__Header header;
  int32_t rssi;
  rosidl_runtime_c__String ssid;
} wifi_interface__msg__WifiRssi;

// Struct for a sequence of wifi_interface__msg__WifiRssi.
typedef struct wifi_interface__msg__WifiRssi__Sequence
{
  wifi_interface__msg__WifiRssi * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wifi_interface__msg__WifiRssi__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__STRUCT_H_
