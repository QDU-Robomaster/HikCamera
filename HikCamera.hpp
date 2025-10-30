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
      autocap: true
template_args: []
required_hardware: []
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

// Hik SDK
#include "MvCameraControl.h"

// OpenCV
#include <opencv2/core.hpp>

// LibXR
#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "thread.hpp"

// STL / 3rd
#include <atomic>
#include <opencv2/core/mat.hpp>

class HikCamera : public LibXR::Application, public CameraBase
{
 public:
  struct RuntimeParam
  {
    float gain = 32.f;
    float exposure_time = 500.f;  // microseconds
    bool autocap = true;          // false -> external trigger, true -> free-run
  };

  static constexpr int CH = 3;  // RGB8 channels

  HikCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
            CameraBase::CameraInfo info, RuntimeParam runtime);
  ~HikCamera() override;

  // CameraBase overrides (follow your template)
  void SetExposure(double exposure) override;
  void SetGain(double gain) override;
  void SetRuntimeParam(const RuntimeParam& p);

 private:
  // ======== lifecycle helpers (preserve original semantics) ========
  bool CaptureStart();  // enumerate, open, set features
  bool StartGrabbing();
  void CaptureStop();  // stop, reset, close, destroy

  // ======== threads ========
  static void CapThreadFun(HikCamera* self);
  static void GuardThreadFun(HikCamera* self);
  // ======== utils ========
  void UpdateParameters();
  void OnMonitor() override {};
  // ======== state ========
  std::atomic<bool> camera_state_{false};  // true while capture thread should run
  std::atomic<bool> stopped_flag_{false};  // set by capture on exit

  // thread
  LibXR::Semaphore stop_event_{0};

  LibXR::Thread capture_thread_;
  LibXR::Thread guard_thread_;
  LibXR::Mutex state_mtx_;

  bool cap_thread_created_{false};
  bool guard_thread_created_{false};

  // device
  void* camera_handle_{};  // MV_CC_DEVICE_HANDLE (void*) in SDK C API
  MV_IMAGE_BASIC_INFO img_info_{};

  // Parameters
  CameraBase::CameraInfo info_{};
  RuntimeParam runtime_{};

  // Topics
  LibXR::Topic frame_topic_ = LibXR::Topic("image_raw", sizeof(cv::Mat));
  LibXR::Topic info_topic_ = LibXR::Topic("camera_info", sizeof(CameraBase::CameraInfo));

  int fail_count_ = 0;
};