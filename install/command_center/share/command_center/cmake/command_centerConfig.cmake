# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_command_center_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED command_center_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(command_center_FOUND FALSE)
  elseif(NOT command_center_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(command_center_FOUND FALSE)
  endif()
  return()
endif()
set(_command_center_CONFIG_INCLUDED TRUE)

# output package information
if(NOT command_center_FIND_QUIETLY)
  message(STATUS "Found command_center: 0.0.0 (${command_center_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'command_center' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${command_center_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(command_center_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${command_center_DIR}/${_extra}")
endforeach()
