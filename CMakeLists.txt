# CMakeLists.txt for HikCamera

target_include_directories(xr
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/hikSDK/include
)

foreach(_prop COMPILE_DEFINITIONS INTERFACE_COMPILE_DEFINITIONS)
  get_target_property(_defs xr ${_prop})
  if(_defs)
    list(FILTER _defs EXCLUDE REGEX "^LIBXR_PRINT_INTEGER_ENABLE_64BIT=")
    set_property(TARGET xr PROPERTY ${_prop} ${_defs})
  endif()
endforeach()
target_compile_definitions(xr PUBLIC LIBXR_PRINT_INTEGER_ENABLE_64BIT=1)

get_filename_component(_MODULE_DIRNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
set(_DEPS_TARGET "${_MODULE_DIRNAME}_deps")
add_library(${_DEPS_TARGET} INTERFACE)

set(_HIK_LIBDIR_AMD64 ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/amd64)
set(_HIK_LIBDIR_ARM64 ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/arm64)

target_link_directories(${_DEPS_TARGET} INTERFACE
  $<$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},x86_64>:${_HIK_LIBDIR_AMD64}>
  $<$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},aarch64>:${_HIK_LIBDIR_ARM64}>
)

target_link_libraries(${_DEPS_TARGET} INTERFACE
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
)

set_property(GLOBAL APPEND PROPERTY XR_MODULE_DEPS ${_DEPS_TARGET})
