// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from wifi_interface:msg/WifiRssi.idl
// generated code does not contain a copyright notice

#ifndef WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__FUNCTIONS_H_
#define WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "wifi_interface/msg/rosidl_generator_c__visibility_control.h"

#include "wifi_interface/msg/detail/wifi_rssi__struct.h"

/// Initialize msg/WifiRssi message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * wifi_interface__msg__WifiRssi
 * )) before or use
 * wifi_interface__msg__WifiRssi__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
bool
wifi_interface__msg__WifiRssi__init(wifi_interface__msg__WifiRssi * msg);

/// Finalize msg/WifiRssi message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
void
wifi_interface__msg__WifiRssi__fini(wifi_interface__msg__WifiRssi * msg);

/// Create msg/WifiRssi message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * wifi_interface__msg__WifiRssi__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
wifi_interface__msg__WifiRssi *
wifi_interface__msg__WifiRssi__create();

/// Destroy msg/WifiRssi message.
/**
 * It calls
 * wifi_interface__msg__WifiRssi__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
void
wifi_interface__msg__WifiRssi__destroy(wifi_interface__msg__WifiRssi * msg);

/// Check for msg/WifiRssi message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
bool
wifi_interface__msg__WifiRssi__are_equal(const wifi_interface__msg__WifiRssi * lhs, const wifi_interface__msg__WifiRssi * rhs);

/// Copy a msg/WifiRssi message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
bool
wifi_interface__msg__WifiRssi__copy(
  const wifi_interface__msg__WifiRssi * input,
  wifi_interface__msg__WifiRssi * output);

/// Initialize array of msg/WifiRssi messages.
/**
 * It allocates the memory for the number of elements and calls
 * wifi_interface__msg__WifiRssi__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
bool
wifi_interface__msg__WifiRssi__Sequence__init(wifi_interface__msg__WifiRssi__Sequence * array, size_t size);

/// Finalize array of msg/WifiRssi messages.
/**
 * It calls
 * wifi_interface__msg__WifiRssi__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
void
wifi_interface__msg__WifiRssi__Sequence__fini(wifi_interface__msg__WifiRssi__Sequence * array);

/// Create array of msg/WifiRssi messages.
/**
 * It allocates the memory for the array and calls
 * wifi_interface__msg__WifiRssi__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
wifi_interface__msg__WifiRssi__Sequence *
wifi_interface__msg__WifiRssi__Sequence__create(size_t size);

/// Destroy array of msg/WifiRssi messages.
/**
 * It calls
 * wifi_interface__msg__WifiRssi__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
void
wifi_interface__msg__WifiRssi__Sequence__destroy(wifi_interface__msg__WifiRssi__Sequence * array);

/// Check for msg/WifiRssi message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
bool
wifi_interface__msg__WifiRssi__Sequence__are_equal(const wifi_interface__msg__WifiRssi__Sequence * lhs, const wifi_interface__msg__WifiRssi__Sequence * rhs);

/// Copy an array of msg/WifiRssi messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_wifi_interface
bool
wifi_interface__msg__WifiRssi__Sequence__copy(
  const wifi_interface__msg__WifiRssi__Sequence * input,
  wifi_interface__msg__WifiRssi__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WIFI_INTERFACE__MSG__DETAIL__WIFI_RSSI__FUNCTIONS_H_
