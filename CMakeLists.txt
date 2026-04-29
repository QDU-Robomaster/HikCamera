# CMakeLists.txt for HikCamera

target_include_directories(xr
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/hikSDK/include
)

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
