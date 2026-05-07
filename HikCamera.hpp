#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Hikrobot USB 相机采集模块，按 CameraBase 图像槽位合约发布图像
constructor_args:
  - runtime:
      camera_name: "camera"
      image_topic_name: "camera_image"
      imu_topic_name: "camera_imu"
      gain: 16.0
      exposure_time: 2000.0
      external_trigger: true
      acquisition_frame_rate: 249.0
      grab_timeout_ms: 100
      image_node_num: 3
      rotate_180: false
      recording: {}
template_args:
  - Info:
      width: 1440
      height: 1080
      step: 4320
      encoding: CameraTypes::Encoding::BGR8
      camera_matrix: [2328.6857198980888, 0.0, 733.35646250924742, 0.0, 2328.6701077899961, 540.61872869227727, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [-0.091821039187099038, 0.46399073468302049, 0.0026098786426372819, 0.0009819586010405485, -0.47512788503104569]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [2328.6857198980888, 0.0, 733.35646250924742, 0.0, 0.0, 2328.6701077899961, 540.61872869227727, 0.0, 0.0, 0.0, 1.0, 0.0]
required_hardware:
  - Hikrobot USB camera
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "MvCameraControl.h"

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "thread.hpp"

/**
 * @brief Hikrobot USB 相机采集模块。
 *
 * 本模块只发布图像：像素写入 `CameraBase` 图像槽位，时间戳来自相机 SDK
 * 的设备时间戳。IMU 原始数据和同步命令由板端/同步模块负责。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class HikCamera : public LibXR::Application,
                  public CameraBase<CameraInfoV>
{
 public:
  using Self = HikCamera<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using ImageFrame = typename Base::ImageFrame;
  using RecordingParam = typename Base::RecordingParam;

  static inline constexpr auto camera_info = Base::camera_info;
  static constexpr int channel_count = 3;
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);
  static constexpr uint64_t microseconds_per_second = 1000000ULL;
  static constexpr uint32_t image_sink_wait_log_ms = 1000;
  static constexpr float max_gain = 16.0F;

  static_assert(camera_info.encoding == CameraTypes::Encoding::BGR8,
                "HikCamera publishes BGR8 frames through MV_CC_GetImageForBGR");
  static_assert(frame_step == static_cast<std::size_t>(camera_info.width) * channel_count,
                "HikCamera expects tightly packed BGR8 frames");
  static_assert(Base::image_bytes <= std::numeric_limits<unsigned int>::max(),
                "Hik SDK image buffer size is unsigned int");
  static_assert(Base::image_bytes >= channel_count,
                "HikCamera image buffer must contain at least one BGR pixel");
  static_assert(Base::image_bytes % channel_count == 0,
                "HikCamera expects complete BGR pixels");

  /**
   * @brief xrobot YAML 传入的运行时参数。
   */
  struct RuntimeParam
  {
    std::string_view camera_name = "camera";  ///< CameraBase 相机名。
    std::string_view image_topic_name = "camera_image";  ///< 图像共享话题名。
    std::string_view imu_topic_name = "camera_imu";  ///< 同步后 IMU 话题名。
    float gain = 16.0F;  ///< 相机增益。
    float exposure_time = 2000.0F;  ///< 曝光时间，单位微秒。
    bool external_trigger = true;  ///< true 时使用 Line0 上升沿外触发。
    float acquisition_frame_rate = 249.0F;  ///< 非外触发模式下的自由运行帧率。
    uint32_t grab_timeout_ms = 100;  ///< SDK 等待一帧图像的超时时间。
    uint32_t image_node_num = 3;  ///< SDK 内部取流缓存节点数。
    bool rotate_180 = false;  ///< true 时使用相机 ReverseX/Y 做 180 度旋转。
    RecordingParam recording{};  ///< CameraBase 生产者侧图像内录配置。
  };

  explicit HikCamera(LibXR::HardwareContainer& hw,
                     LibXR::ApplicationManager& app,
                     RuntimeParam runtime)
      : Base(hw, runtime.camera_name, runtime.image_topic_name, runtime.imu_topic_name,
             runtime.recording),
        runtime_(runtime)
  {
    runtime_.gain = ClampGain(runtime_.gain);
    XR_LOG_INFO("Starting HikCamera: external_trigger=%d rotate_180=%d",
                runtime_.external_trigger ? 1 : 0, runtime_.rotate_180 ? 1 : 0);
    if (CaptureStart() && StartGrabbing())
    {
      camera_state_.store(true);
      capture_thread_ = std::thread(CaptureThreadMain, this);
      capture_thread_created_ = true;
    }
    else
    {
      CaptureStop();
      throw std::runtime_error("HikCamera: failed to start camera");
    }
    app.Register(*this);
  }

  ~HikCamera() override
  {
    camera_state_.store(false);
    if (capture_thread_created_ && capture_thread_.joinable())
    {
      capture_thread_.join();
      capture_thread_created_ = false;
    }
    CaptureStop();
  }

  void OnMonitor() override
  {
    XR_LOG_INFO("HikCamera monitor: frames=%u failures=%u",
                frames_committed_, failure_count_);
  }

  void SetExposure(double exposure) override
  {
    runtime_.exposure_time = static_cast<float>(exposure);
    UpdateParameters();
  }

  void SetGain(double gain) override
  {
    runtime_.gain = ClampGain(static_cast<float>(gain));
    UpdateParameters();
  }

 private:
  static float ClampGain(float gain)
  {
    if (gain > max_gain)
    {
      XR_LOG_WARN("HikCamera gain %.3f exceeds max %.3f; clamping",
                  static_cast<double>(gain), static_cast<double>(max_gain));
      return max_gain;
    }
    return gain;
  }

  bool SetFloatValue(const char* name, double value)
  {
    const auto ret = MV_CC_SetFloatValue(camera_handle_, name, static_cast<float>(value));
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_SetFloatValue(%s, %.3f) failed: %d",
                   name, value, ret);
      return false;
    }
    return true;
  }

  bool SetEnumValue(const char* name, unsigned int value)
  {
    const auto ret = MV_CC_SetEnumValue(camera_handle_, name, value);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_SetEnumValue(%s, %u) failed: %d",
                   name, value, ret);
      return false;
    }
    return true;
  }

  bool SetEnumValueByString(const char* name, const char* value)
  {
    const auto ret = MV_CC_SetEnumValueByString(camera_handle_, name, value);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_SetEnumValueByString(%s, %s) failed: %d",
                   name, value, ret);
      return false;
    }
    return true;
  }

  bool GetBoolValue(const char* name, bool& value)
  {
    const auto ret = MV_CC_GetBoolValue(camera_handle_, name, &value);
    if (ret != MV_OK)
    {
      XR_LOG_WARN("HikCamera MV_CC_GetBoolValue(%s) failed: %d", name, ret);
      return false;
    }
    return true;
  }

  bool SetBoolValue(const char* name, bool value)
  {
    const auto ret = MV_CC_SetBoolValue(camera_handle_, name, value);
    if (ret != MV_OK)
    {
      XR_LOG_WARN("HikCamera MV_CC_SetBoolValue(%s, %d) failed: %d",
                  name, value ? 1 : 0, ret);
      return false;
    }
    return true;
  }

  bool ConfigureRotation()
  {
    device_rotate_180_ = false;
    if (!runtime_.rotate_180)
    {
      return true;
    }

    bool old_reverse_x = false;
    bool old_reverse_y = false;
    if (!GetBoolValue("ReverseX", old_reverse_x) ||
        !GetBoolValue("ReverseY", old_reverse_y))
    {
      XR_LOG_ERROR("HikCamera rotate_180 requires camera ReverseX/ReverseY");
      return false;
    }

    old_reverse_x_ = old_reverse_x;
    old_reverse_y_ = old_reverse_y;
    reverse_state_saved_ = true;
    if (SetBoolValue("ReverseX", true) && SetBoolValue("ReverseY", true))
    {
      device_rotate_180_ = true;
      XR_LOG_PASS("HikCamera using camera ReverseX+ReverseY for 180-degree rotation");
      return true;
    }

    (void)SetBoolValue("ReverseX", old_reverse_x);
    (void)SetBoolValue("ReverseY", old_reverse_y);
    reverse_state_saved_ = false;
    XR_LOG_ERROR("HikCamera failed to enable camera ReverseX+ReverseY");
    return false;
  }

  const char* RotationModeName() const
  {
    if (device_rotate_180_)
    {
      return "device_reverse_xy";
    }
    return "none";
  }

  void RestoreDeviceRotation()
  {
    if (!device_rotate_180_ || !reverse_state_saved_ || camera_handle_ == nullptr)
    {
      return;
    }

    (void)SetBoolValue("ReverseX", old_reverse_x_);
    (void)SetBoolValue("ReverseY", old_reverse_y_);
    device_rotate_180_ = false;
    reverse_state_saved_ = false;
  }

  bool CaptureStart()
  {
    MV_CC_DEVICE_INFO_LIST device_list{};
    auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0)
    {
      XR_LOG_ERROR("HikCamera no USB camera found: ret=%d count=%u",
                   ret, device_list.nDeviceNum);
      return false;
    }

    ret = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_CreateHandle failed: %d", ret);
      return false;
    }
    ret = MV_CC_OpenDevice(camera_handle_);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_OpenDevice failed: %d", ret);
      return false;
    }

    ret = MV_CC_SetImageNodeNum(camera_handle_, runtime_.image_node_num);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_SetImageNodeNum failed: %d", ret);
      return false;
    }

    if (!SetEnumValueByString("AcquisitionMode", "Continuous"))
    {
      return false;
    }

    if (runtime_.external_trigger)
    {
      if (!SetEnumValue("TriggerMode", 1) ||
          !SetEnumValueByString("TriggerSource", "Line0") ||
          !SetEnumValueByString("TriggerActivation", "RisingEdge"))
      {
        return false;
      }
    }
    else if (!SetEnumValue("TriggerMode", 0) ||
             !SetFloatValue("AcquisitionFrameRate", runtime_.acquisition_frame_rate))
    {
      return false;
    }

    if (!SetEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS) ||
        !SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF) ||
        !SetEnumValue("GainAuto", MV_GAIN_MODE_OFF) ||
        !SetFloatValue("ExposureTime", runtime_.exposure_time) ||
        !SetFloatValue("Gain", runtime_.gain))
    {
      return false;
    }

    if (!ConfigureRotation())
    {
      return false;
    }
    ProbeDeviceTimestampFrequency();

    XR_LOG_PASS("HikCamera configured: trigger=%s rotate_180=%d rotate_mode=%s "
                "gain=%.3f exposure=%.3f us timestamp_freq_hz=%llu",
                runtime_.external_trigger ? "external" : "freerun",
                runtime_.rotate_180 ? 1 : 0, RotationModeName(), runtime_.gain,
                runtime_.exposure_time,
                static_cast<unsigned long long>(device_timestamp_frequency_hz_));
    return true;
  }

  bool StartGrabbing()
  {
    const auto ret = MV_CC_StartGrabbing(camera_handle_);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_StartGrabbing failed: %d", ret);
      return false;
    }
    return true;
  }

  void CaptureStop()
  {
    if (camera_handle_ == nullptr)
    {
      return;
    }
    (void)MV_CC_StopGrabbing(camera_handle_);
    RestoreDeviceRotation();
    (void)MV_CC_CloseDevice(camera_handle_);
    (void)MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
  }

  void UpdateParameters()
  {
    if (camera_handle_ == nullptr)
    {
      return;
    }
    (void)SetFloatValue("ExposureTime", runtime_.exposure_time);
    (void)SetFloatValue("Gain", runtime_.gain);
  }

  void ProbeDeviceTimestampFrequency()
  {
    MVCC_INTVALUE_EX value{};
    const auto ret = MV_CC_GetIntValueEx(camera_handle_, "DeviceTimestampIncrement", &value);
    if (ret == MV_OK && value.nCurValue > 0)
    {
      device_timestamp_frequency_hz_ = static_cast<uint64_t>(value.nCurValue);
      return;
    }

    device_timestamp_frequency_hz_ = microseconds_per_second;
    XR_LOG_WARN("HikCamera DeviceTimestampIncrement unavailable: ret=%d value=%lld, "
                "assume %llu Hz",
                ret, static_cast<long long>(value.nCurValue),
                static_cast<unsigned long long>(device_timestamp_frequency_hz_));
  }

  [[nodiscard]] bool ResolveImageTimestampUs(const MV_FRAME_OUT_INFO_EX& frame_info,
                                             uint64_t& timestamp_us)
  {
    // dev_ts 是 SDK 提供的设备侧时间戳；不能退回主机到达时间做同步。
    const uint64_t dev_ts =
        CombineU32(frame_info.nDevTimeStampHigh, frame_info.nDevTimeStampLow);
    if (dev_ts == 0)
    {
      XR_LOG_ERROR("HikCamera frame has no device timestamp: frame=%u host_ts=%lld",
                   frame_info.nFrameNum,
                   static_cast<long long>(frame_info.nHostTimeStamp));
      return false;
    }

    timestamp_us = DeviceTicksToUs(dev_ts);
    return true;
  }

  static uint64_t CombineU32(uint32_t high, uint32_t low)
  {
    return (static_cast<uint64_t>(high) << 32U) | static_cast<uint64_t>(low);
  }

  uint64_t DeviceTicksToUs(uint64_t dev_ts) const
  {
    const uint64_t freq = device_timestamp_frequency_hz_;
    if (freq == 0 || freq == microseconds_per_second)
    {
      return dev_ts;
    }

    const uint64_t seconds = dev_ts / freq;
    const uint64_t remainder = dev_ts % freq;
    return seconds * microseconds_per_second +
           (remainder * microseconds_per_second) / freq;
  }

  void LogFirstCommittedFrame(const MV_FRAME_OUT_INFO_EX& frame_info, uint64_t timestamp_us)
  {
    const uint64_t dev_ts =
        CombineU32(frame_info.nDevTimeStampHigh, frame_info.nDevTimeStampLow);
    XR_LOG_INFO("HikCamera first frame: frame=%u sensor_ts=%llu us dev_ts=%llu "
                "host_ts=%lld counter=%u trigger=%u lost=%u",
                frame_info.nFrameNum,
                static_cast<unsigned long long>(timestamp_us),
                static_cast<unsigned long long>(dev_ts),
                static_cast<long long>(frame_info.nHostTimeStamp),
                frame_info.nFrameCounter,
                frame_info.nTriggerIndex,
                frame_info.nLostPacket);
  }

  void WaitForImageSink()
  {
    uint32_t waited_ms = 0;
    while (camera_state_.load() && !this->ImageSinkReady())
    {
      LibXR::Thread::Sleep(1);
      ++waited_ms;
      if (waited_ms % image_sink_wait_log_ms == 0)
      {
        XR_LOG_WARN("HikCamera waiting image sink: %u ms", waited_ms);
      }
    }
  }

  void CaptureLoop()
  {
    WaitForImageSink();
    while (camera_state_.load())
    {
      ImageFrame* image = this->GetWritableImage();
      if (image == nullptr)
      {
        LibXR::Thread::Sleep(1);
        continue;
      }

      MV_FRAME_OUT_INFO_EX frame_info{};
      const auto ret = MV_CC_GetImageForBGR(
          camera_handle_, image->data.data(), static_cast<unsigned int>(Base::image_bytes),
          &frame_info, static_cast<int>(runtime_.grab_timeout_ms));
      if (ret != MV_OK)
      {
        ++failure_count_;
        continue;
      }

      if (frame_info.nWidth != camera_info.width || frame_info.nHeight != camera_info.height ||
          frame_info.nFrameLen != static_cast<unsigned int>(Base::image_bytes))
      {
        XR_LOG_ERROR("HikCamera frame geometry mismatch: %ux%u len=%u",
                     frame_info.nWidth, frame_info.nHeight, frame_info.nFrameLen);
        ++failure_count_;
        continue;
      }

      uint64_t image_timestamp_us = 0;
      if (!ResolveImageTimestampUs(frame_info, image_timestamp_us))
      {
        ++failure_count_;
        continue;
      }

      image->timestamp_us = image_timestamp_us;
      if (this->CommitImage())
      {
        if (frames_committed_ == 0)
        {
          LogFirstCommittedFrame(frame_info, image_timestamp_us);
        }
        ++frames_committed_;
      }
      else
      {
        ++failure_count_;
      }
    }
  }

  static void CaptureThreadMain(Self* self) { self->CaptureLoop(); }

 private:
  RuntimeParam runtime_{};
  void* camera_handle_{nullptr};
  std::atomic<bool> camera_state_{false};
  std::thread capture_thread_{};
  bool capture_thread_created_{false};
  bool device_rotate_180_{false};
  bool reverse_state_saved_{false};
  bool old_reverse_x_{false};
  bool old_reverse_y_{false};
  uint64_t device_timestamp_frequency_hz_{microseconds_per_second};
  uint32_t frames_committed_{0};
  uint32_t failure_count_{0};
};
