#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cfg:
      gain: 32.0
      exposure_ms: 500.0
      autocap: true
template_args: []
required_hardware: []
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on
#include <atomic>
#include <condition_variable>
#include <thread>

#include "CameraBase.hpp"
#include "MvCameraControl.h"
#include "app_framework.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "timebase.hpp"

class HikCamera : public LibXR::Application, public CameraBase
{
 public:
  struct ImageData
  {
    cv::Mat image;
    LibXR::MicrosecondTimestamp time;
  };

  struct Config
  {
    double exposure_ms = 500.0;
    double gain = 32.0;
    bool autocap = true;
  };

  enum class Hik : uint8_t
  {
    RUNNING,
    STOPPED
  };

  struct Protect
  {
    std::mutex mux;
    std::condition_variable HikIsquit;
    std::thread protectthread;
  };
  HikCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Config cfg);

  bool Read(ImageData& imgdata);

  ~HikCamera();
  void OnMonitor() override {}

 private:
  LibXR::Topic image_topic_;

  void* handle_;

  Config cfg_;
  Protect guard_;

  std::atomic<Hik> hik_state_;
  std::thread hik_sdk_thread_;

  void ProtectRunning();

  void CaptureInit();

  void CaptureStop();

  void SetFloatValue(const std::string& name, double value);
  void SetEnumValue(const std::string& name, unsigned int value);
};
