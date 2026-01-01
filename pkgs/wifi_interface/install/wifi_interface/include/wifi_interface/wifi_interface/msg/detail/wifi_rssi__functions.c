// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wifi_interface:msg/WifiRssi.idl
// generated code does not contain a copyright notice
#include "wifi_interface/msg/detail/wifi_rssi__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `ssid`
#include "rosidl_runtime_c/string_functions.h"

bool
wifi_interface__msg__WifiRssi__init(wifi_interface__msg__WifiRssi * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wifi_interface__msg__WifiRssi__fini(msg);
    return false;
  }
  // rssi
  // ssid
  if (!rosidl_runtime_c__String__init(&msg->ssid)) {
    wifi_interface__msg__WifiRssi__fini(msg);
    return false;
  }
  return true;
}

void
wifi_interface__msg__WifiRssi__fini(wifi_interface__msg__WifiRssi * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rssi
  // ssid
  rosidl_runtime_c__String__fini(&msg->ssid);
}

bool
wifi_interface__msg__WifiRssi__are_equal(const wifi_interface__msg__WifiRssi * lhs, const wifi_interface__msg__WifiRssi * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // rssi
  if (lhs->rssi != rhs->rssi) {
    return false;
  }
  // ssid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->ssid), &(rhs->ssid)))
  {
    return false;
  }
  return true;
}

bool
wifi_interface__msg__WifiRssi__copy(
  const wifi_interface__msg__WifiRssi * input,
  wifi_interface__msg__WifiRssi * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // rssi
  output->rssi = input->rssi;
  // ssid
  if (!rosidl_runtime_c__String__copy(
      &(input->ssid), &(output->ssid)))
  {
    return false;
  }
  return true;
}

wifi_interface__msg__WifiRssi *
wifi_interface__msg__WifiRssi__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wifi_interface__msg__WifiRssi * msg = (wifi_interface__msg__WifiRssi *)allocator.allocate(sizeof(wifi_interface__msg__WifiRssi), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wifi_interface__msg__WifiRssi));
  bool success = wifi_interface__msg__WifiRssi__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wifi_interface__msg__WifiRssi__destroy(wifi_interface__msg__WifiRssi * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wifi_interface__msg__WifiRssi__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wifi_interface__msg__WifiRssi__Sequence__init(wifi_interface__msg__WifiRssi__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wifi_interface__msg__WifiRssi * data = NULL;

  if (size) {
    data = (wifi_interface__msg__WifiRssi *)allocator.zero_allocate(size, sizeof(wifi_interface__msg__WifiRssi), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wifi_interface__msg__WifiRssi__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wifi_interface__msg__WifiRssi__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
wifi_interface__msg__WifiRssi__Sequence__fini(wifi_interface__msg__WifiRssi__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      wifi_interface__msg__WifiRssi__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

wifi_interface__msg__WifiRssi__Sequence *
wifi_interface__msg__WifiRssi__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wifi_interface__msg__WifiRssi__Sequence * array = (wifi_interface__msg__WifiRssi__Sequence *)allocator.allocate(sizeof(wifi_interface__msg__WifiRssi__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wifi_interface__msg__WifiRssi__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wifi_interface__msg__WifiRssi__Sequence__destroy(wifi_interface__msg__WifiRssi__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wifi_interface__msg__WifiRssi__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wifi_interface__msg__WifiRssi__Sequence__are_equal(const wifi_interface__msg__WifiRssi__Sequence * lhs, const wifi_interface__msg__WifiRssi__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wifi_interface__msg__WifiRssi__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wifi_interface__msg__WifiRssi__Sequence__copy(
  const wifi_interface__msg__WifiRssi__Sequence * input,
  wifi_interface__msg__WifiRssi__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wifi_interface__msg__WifiRssi);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wifi_interface__msg__WifiRssi * data =
      (wifi_interface__msg__WifiRssi *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wifi_interface__msg__WifiRssi__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wifi_interface__msg__WifiRssi__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wifi_interface__msg__WifiRssi__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
