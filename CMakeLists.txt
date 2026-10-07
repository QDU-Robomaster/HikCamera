# HikCamera 模块 CMake 配置

# SDK：已安装官方 MVS 时用 /opt/MVS，否则用仓库自带的 MVS 5.0.2。
# SDK: the official MVS under /opt/MVS when installed, else the bundled MVS 5.0.2.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(_HIK_SYSTEM_LIBDIR /opt/MVS/lib/64)
  set(_HIK_BUNDLED_LIBDIR ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(_HIK_SYSTEM_LIBDIR /opt/MVS/lib/aarch64)
  set(_HIK_BUNDLED_LIBDIR ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/arm64)
else()
  message(FATAL_ERROR "HikCamera: unsupported processor ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(EXISTS /opt/MVS/include/MvCameraControl.h
   AND EXISTS ${_HIK_SYSTEM_LIBDIR}/libMvCameraControl.so)
  set(_HIK_INCLUDE_DIR /opt/MVS/include)
  set(_HIK_LIBDIR ${_HIK_SYSTEM_LIBDIR})
else()
  set(_HIK_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/hikSDK/include)
  set(_HIK_LIBDIR ${_HIK_BUNDLED_LIBDIR})
endif()
message(STATUS "HikCamera: MVS SDK from ${_HIK_LIBDIR}")

target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${_HIK_INCLUDE_DIR})

if(NOT TARGET HikMVS::MvCameraControl)
  add_library(HikMVS::MvCameraControl SHARED IMPORTED GLOBAL)
  set_target_properties(
    HikMVS::MvCameraControl PROPERTIES IMPORTED_LOCATION
                                       ${_HIK_LIBDIR}/libMvCameraControl.so)
endif()

get_filename_component(_MODULE_DIRNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
set(_DEPS_TARGET "${_MODULE_DIRNAME}_deps")
add_library(${_DEPS_TARGET} INTERFACE)
target_link_libraries(${_DEPS_TARGET} INTERFACE HikMVS::MvCameraControl)
set_property(GLOBAL APPEND PROPERTY XR_MODULE_DEPS ${_DEPS_TARGET})

if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tests
                   ${CMAKE_CURRENT_BINARY_DIR}/HikCamera_tests)
endif()
