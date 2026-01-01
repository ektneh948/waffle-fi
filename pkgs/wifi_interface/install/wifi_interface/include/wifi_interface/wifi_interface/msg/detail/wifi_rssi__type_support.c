// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wifi_interface:msg/WifiRssi.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wifi_interface/msg/detail/wifi_rssi__rosidl_typesupport_introspection_c.h"
#include "wifi_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wifi_interface/msg/detail/wifi_rssi__functions.h"
#include "wifi_interface/msg/detail/wifi_rssi__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `ssid`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wifi_interface__msg__WifiRssi__init(message_memory);
}

void wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_fini_function(void * message_memory)
{
  wifi_interface__msg__WifiRssi__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wifi_interface__msg__WifiRssi, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rssi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wifi_interface__msg__WifiRssi, rssi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ssid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wifi_interface__msg__WifiRssi, ssid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_members = {
  "wifi_interface__msg",  // message namespace
  "WifiRssi",  // message name
  3,  // number of fields
  sizeof(wifi_interface__msg__WifiRssi),
  wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_member_array,  // message members
  wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_init_function,  // function to initialize message memory (memory has to be allocated)
  wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_type_support_handle = {
  0,
  &wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wifi_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wifi_interface, msg, WifiRssi)() {
  wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_type_support_handle.typesupport_identifier) {
    wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wifi_interface__msg__WifiRssi__rosidl_typesupport_introspection_c__WifiRssi_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
