# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_qt_robot_control_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED qt_robot_control_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(qt_robot_control_FOUND FALSE)
  elseif(NOT qt_robot_control_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(qt_robot_control_FOUND FALSE)
  endif()
  return()
endif()
set(_qt_robot_control_CONFIG_INCLUDED TRUE)

# output package information
if(NOT qt_robot_control_FIND_QUIETLY)
  message(STATUS "Found qt_robot_control: 1.0.0 (${qt_robot_control_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'qt_robot_control' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${qt_robot_control_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(qt_robot_control_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${qt_robot_control_DIR}/${_extra}")
endforeach()
