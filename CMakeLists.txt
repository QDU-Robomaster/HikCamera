# CMakeLists.txt for HikCamera

# Add module to include path
target_include_directories(xr
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/hikSDK/include
  ${CMAKE_CURRENT_LIST_DIR}
)

# Auto-include source files
file(GLOB MODULE_HIKCAMERA_SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/*.cxx"
  "${CMAKE_CURRENT_LIST_DIR}/*.cc"
  "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

target_sources(xr PRIVATE ${MODULE_HIKCAMERA_SRC})

# INTERFACE deps shell
get_filename_component(_MODULE_DIRNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
set(_DEPS_TARGET "${_MODULE_DIRNAME}_deps")
add_library(${_DEPS_TARGET} INTERFACE)

set(_HIK_LIBDIR_AMD64 ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/amd64)
set(_HIK_LIBDIR_ARM64 ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/arm64)

target_link_directories(${_DEPS_TARGET} INTERFACE
  $<$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},x86_64>:${_HIK_LIBDIR_AMD64}>
  $<$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},aarch64>:${_HIK_LIBDIR_ARM64}>
)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui)

target_link_libraries(${_DEPS_TARGET} INTERFACE
  ${OpenCV_LIBS}
  # HikSDK
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/amd64/ DESTINATION lib)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/hikSDK/lib/arm64/ DESTINATION lib)
else()
  message(FATAL_ERROR "Unsupported host system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Register to global
set_property(GLOBAL APPEND PROPERTY XR_MODULE_DEPS ${_DEPS_TARGET})
