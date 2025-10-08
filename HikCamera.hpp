#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - info:
      width: 1440
      height: 1080
      step: 4320
      timestamp: 0
      encoding: CameraBase::Encoding::RGB8
      camera_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      distortion_model: CameraBase::DistortionModel::PLUMB_BOB
      distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - runtime:
      gain: 32.0
      exposure_time: 500.0
template_args: []
required_hardware: []
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

#include "CameraBase.hpp"
#include "MvCameraControl.h"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "message.hpp"

// STL
#include <atomic>
#include <memory>
#include <opencv2/core/mat.hpp>

class HikCamera : public LibXR::Application

{
  static constexpr int MAX_W = 4096;
  static constexpr int MAX_H = 3072;
  static constexpr int CH = 3;
  static constexpr size_t BUF_BYTES = static_cast<size_t>(MAX_W) * MAX_H * CH;

 public:
  struct RuntimeParam
  {
    float gain = 32.0f;
    float exposure_time = 500.0f;  // microseconds
  };

 public:
  // Main constructor (definition in .cpp)
  explicit HikCamera(LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
                     const CameraBase::CameraInfo info, const RuntimeParam runtime);

  ~HikCamera();

  void SetRuntimeParam(const RuntimeParam& p);

  void OnMonitor() override {}

 private:
  void UpdateParameters();

  static void ThreadFun(HikCamera* self);

  // Runtime state
  std::unique_ptr<std::array<uint8_t, BUF_BYTES>> frame_buf_{};  // large RGB buffer

  // Parameters
  CameraBase::CameraInfo info_;
  RuntimeParam runtime_{};

  // Topics
  LibXR::Topic frame_topic_ = LibXR::Topic("image_raw", sizeof(cv::Mat));
  LibXR::Topic info_topic_ = LibXR::Topic("camera_info", sizeof(CameraBase::CameraInfo));

  // Camera handle & info
  void* camera_handle_{};
  MV_IMAGE_BASIC_INFO img_info_{};
  MV_CC_PIXEL_CONVERT_PARAM convert_param_{};

  int fail_count_ = 0;
  std::atomic<bool> running_{false};
  LibXR::Thread capture_thread_{};
};
