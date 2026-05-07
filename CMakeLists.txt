# CMakeLists.txt for HikCamera

set(_HIK_SYSTEM_ROOT /opt/MVS)
set(_HIK_USE_SYSTEM_SDK OFF)
if(EXISTS ${_HIK_SYSTEM_ROOT}/include/MvCameraControl.h AND
   EXISTS ${_HIK_SYSTEM_ROOT}/lib/64/libMvCameraControl.so)
  set(_HIK_USE_SYSTEM_SDK ON)
endif()

if(_HIK_USE_SYSTEM_SDK)
  set(_HIK_INCLUDE_DIR ${_HIK_SYSTEM_ROOT}/include)
  set(_HIK_LIBDIR_AMD64 ${_HIK_SYSTEM_ROOT}/lib/64)
  set(_HIK_LIBDIR_ARM64 ${_HIK_SYSTEM_ROOT}/lib/64)
  message(STATUS "HikCamera: using system MVS SDK from ${_HIK_SYSTEM_ROOT}")
else()
  set(_HIK_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/hikSDK/include)
  set(_HIK_LIBDIR_AMD64 ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/amd64)
  set(_HIK_LIBDIR_ARM64 ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/arm64)
  message(STATUS "HikCamera: using bundled MVS SDK")
endif()

target_include_directories(xr
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${_HIK_INCLUDE_DIR}
)

get_filename_component(_MODULE_DIRNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
set(_DEPS_TARGET "${_MODULE_DIRNAME}_deps")
add_library(${_DEPS_TARGET} INTERFACE)

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
