# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:wifi_interface__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:wifi_interface__rosidl_typesupport_fastrtps_c;__rosidl_typesupport_introspection_c:wifi_interface__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:wifi_interface__rosidl_typesupport_c;__rosidl_generator_cpp:wifi_interface__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:wifi_interface__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_cpp:wifi_interface__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:wifi_interface__rosidl_typesupport_cpp;__rosidl_generator_py:wifi_interface__rosidl_generator_py")

# populate wifi_interface_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "wifi_interface::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'wifi_interface' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND wifi_interface_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
