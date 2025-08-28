# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vel-to-ser_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vel-to-ser_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vel-to-ser_FOUND FALSE)
  elseif(NOT vel-to-ser_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vel-to-ser_FOUND FALSE)
  endif()
  return()
endif()
set(_vel-to-ser_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vel-to-ser_FIND_QUIETLY)
  message(STATUS "Found vel-to-ser: 0.0.0 (${vel-to-ser_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vel-to-ser' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${vel-to-ser_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vel-to-ser_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${vel-to-ser_DIR}/${_extra}")
endforeach()
