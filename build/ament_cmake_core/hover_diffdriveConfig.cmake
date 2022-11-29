# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hover_diffdrive_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hover_diffdrive_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hover_diffdrive_FOUND FALSE)
  elseif(NOT hover_diffdrive_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hover_diffdrive_FOUND FALSE)
  endif()
  return()
endif()
set(_hover_diffdrive_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hover_diffdrive_FIND_QUIETLY)
  message(STATUS "Found hover_diffdrive: 0.0.0 (${hover_diffdrive_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hover_diffdrive' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${hover_diffdrive_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hover_diffdrive_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${hover_diffdrive_DIR}/${_extra}")
endforeach()
