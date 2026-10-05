#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 海康 USB 相机驱动：BayerRG8 原始图像直接写入 CameraBase 图像槽 / Hikrobot USB camera driver that writes raw BayerRG8 images into the CameraBase slots
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "CameraBase.hpp"
#include "MvCameraControl.h"
#include "libxr_def.hpp"
#include "logger.hpp"

/// 可单独测试的换算 / Conversions that can be tested on their own.
namespace HikCameraDetail
{
/// 相机节点上的几何：偏移按跳采后的像素计 / Geometry as camera nodes; offsets are in
/// decimated pixels.
struct NodeGeometry
{
  int64_t decimation;
  int64_t width;
  int64_t height;
  int64_t offset_x;
  int64_t offset_y;
};

inline NodeGeometry ToNodeGeometry(const CameraTypes::FrameGeometry& geometry)
{
  const int64_t d = geometry.decimation;
  return {d, CameraTypes::FRAME_WIDTH, CameraTypes::FRAME_HEIGHT, geometry.roi_x / d,
          geometry.roi_y / d};
}

/// 设备时间戳 tick 换算成微秒 / Device timestamp ticks to microseconds.
inline uint64_t TicksToUs(uint64_t ticks, uint64_t ticks_per_second)
{
  constexpr uint64_t US_PER_SECOND = 1000000;
  if (ticks_per_second == US_PER_SECOND)
  {
    return ticks;
  }
  return ticks / ticks_per_second * US_PER_SECOND +
         ticks % ticks_per_second * US_PER_SECOND / ticks_per_second;
}

/// `ADCBitDepth` 节点的枚举值：8 位为 0，12 位为 3 / Enum value of the `ADCBitDepth`
/// node.
inline uint32_t AdcBitDepthEnum(uint8_t bits) { return bits == 8 ? 0 : 3; }
}  // namespace HikCameraDetail

/// 相机设置，与 YAML 一一对应 / Camera settings, one-to-one with the YAML.
struct HikSettings
{
  float exposure_us;
  float gain_db;
  uint8_t adc_bits;       ///< 8 或 12 / 8 or 12
  bool external_trigger;  ///< Line0 上升沿触发 / Line0 rising-edge trigger
  float free_run_fps;     ///< 不用外触发时的帧率 / Frame rate without external trigger
};

/**
 * @brief 海康 USB 相机（MV-CS016-10UC）。打开第一台 USB 相机，按 WIDE 档开始取流。
 *        Hikrobot USB camera (MV-CS016-10UC). Opens the first USB camera and starts
 *        streaming in the WIDE view.
 *
 * 像素为 BayerRG8，SDK 不做去马赛克；帧计数取 SDK 的 `nFrameNum`，每次开始取流从 0 起。
 * 打开或配置失败即致命退出。
 * Pixels are BayerRG8 with no demosaicing in the SDK. The frame counter is the SDK's
 * `nFrameNum`, which restarts at 0 on every stream start. Failing to open or configure
 * the camera is fatal.
 */
class HikCamera : public CameraBase
{
 public:
  HikCamera(const CameraTypes::CameraCalibration& calibration, NarrowPosition narrow,
            std::string_view name, const HikSettings& settings)
      : CameraBase(calibration, narrow, name, SlotPolicy::DROP)
  {
    REQUIRE(settings.adc_bits == 8 || settings.adc_bits == 12);
    REQUIRE(settings.external_trigger || settings.free_run_fps > 0.0F);
    // 无触发时取图超时须长于一帧 / Without a trigger the timeout must exceed a frame.
    if (!settings.external_trigger)
    {
      grab_timeout_ms_ = std::max(grab_timeout_ms_,
                                  static_cast<uint32_t>(2000.0F / settings.free_run_fps));
    }
    REQUIRE(Open() && Configure(settings) && WriteGeometry(CurrentGeometry()) &&
            Check(MV_CC_StartGrabbing(handle_), "StartGrabbing"));
    StartCapture();
  }

  ~HikCamera() override
  {
    StopCapture();
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    MV_CC_Finalize();
  }

 private:
  /// 外触发下取图超时：停触发后采集线程最多再等这么久才退出（切档盲区的一部分）。
  /// Grab timeout with external trigger: after the trigger stops, the capture thread
  /// waits at most this long before exiting (part of the view-switch gap).
  static constexpr uint32_t TRIGGERED_GRAB_TIMEOUT_MS = 20;
  /// 切档失败时的重试次数 / Retries when a view switch fails.
  static constexpr int VIEW_RETRIES = 3;
  /// SDK 取流缓存节点数 / SDK stream buffer nodes.
  static constexpr unsigned int IMAGE_NODES = 3;

  bool GrabFrame(ImageFrame& frame) override
  {
    MV_FRAME_OUT out{};
    if (MV_CC_GetImageBuffer(handle_, &out, grab_timeout_ms_) != MV_OK)
    {
      return false;
    }
    const MV_FRAME_OUT_INFO_EX& info = out.stFrameInfo;
    const bool layout_ok = info.nWidth == CameraTypes::FRAME_WIDTH &&
                           info.nHeight == CameraTypes::FRAME_HEIGHT &&
                           info.enPixelType == PixelType_Gvsp_BayerRG8 &&
                           info.nFrameLen == CameraTypes::FRAME_BYTES;
    if (layout_ok)
    {
      std::memcpy(frame.data.data(), out.pBufAddr, CameraTypes::FRAME_BYTES);
      const uint64_t ticks =
          (static_cast<uint64_t>(info.nDevTimeStampHigh) << 32) | info.nDevTimeStampLow;
      frame.timestamp_us = HikCameraDetail::TicksToUs(ticks, ticks_per_second_);
      frame.frame_counter = info.nFrameNum;
    }
    else
    {
      XR_LOG_ERROR("%s: unexpected frame %ux%u type=0x%x len=%u", Name().c_str(),
                   info.nWidth, info.nHeight, static_cast<unsigned>(info.enPixelType),
                   info.nFrameLen);
    }
    MV_CC_FreeImageBuffer(handle_, &out);
    return layout_ok;
  }

  LibXR::ErrorCode ApplyView(const CameraTypes::FrameGeometry& geometry) override
  {
    MV_CC_StopGrabbing(handle_);
    for (int attempt = 0; attempt <= VIEW_RETRIES; ++attempt)
    {
      if (WriteGeometry(geometry) && Check(MV_CC_StartGrabbing(handle_), "StartGrabbing"))
      {
        return LibXR::ErrorCode::OK;
      }
      MV_CC_StopGrabbing(handle_);
    }
    return LibXR::ErrorCode::FAILED;
  }

  bool Open()
  {
    MV_CC_DEVICE_INFO_LIST devices{};
    if (!Check(MV_CC_Initialize(), "Initialize") ||
        !Check(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "EnumDevices"))
    {
      return false;
    }
    if (devices.nDeviceNum == 0)
    {
      XR_LOG_ERROR("%s: no USB camera found", Name().c_str());
      return false;
    }
    return Check(MV_CC_CreateHandle(&handle_, devices.pDeviceInfo[0]), "CreateHandle") &&
           Check(MV_CC_OpenDevice(handle_), "OpenDevice");
  }

  /// 与 v4 数据采集（rawcap）相同的设置 / The same settings as the v4 data capture.
  bool Configure(const HikSettings& settings)
  {
    const bool image_ok =
        Check(MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous"),
              "AcquisitionMode") &&
        Check(MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BayerRG8),
              "PixelFormat") &&
        Check(MV_CC_SetEnumValue(handle_, "ADCBitDepth",
                                 HikCameraDetail::AdcBitDepthEnum(settings.adc_bits)),
              "ADCBitDepth") &&
        Check(MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF),
              "ExposureAuto") &&
        Check(MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF), "GainAuto") &&
        Check(MV_CC_SetFloatValue(handle_, "ExposureTime", settings.exposure_us),
              "ExposureTime") &&
        Check(MV_CC_SetFloatValue(handle_, "Gain", settings.gain_db), "Gain") &&
        Check(MV_CC_SetImageNodeNum(handle_, IMAGE_NODES), "SetImageNodeNum");
    if (!image_ok)
    {
      return false;
    }

    bool trigger_ok = false;
    if (settings.external_trigger)
    {
      trigger_ok =
          Check(MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_ON),
                "TriggerMode") &&
          Check(MV_CC_SetEnumValueByString(handle_, "TriggerSource", "Line0"),
                "TriggerSource") &&
          Check(MV_CC_SetEnumValueByString(handle_, "TriggerActivation", "RisingEdge"),
                "TriggerActivation");
    }
    else
    {
      trigger_ok = Check(MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF),
                         "TriggerMode") &&
                   Check(MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true),
                         "AcquisitionFrameRateEnable") &&
                   Check(MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
                                             settings.free_run_fps),
                         "AcquisitionFrameRate");
    }

    MVCC_INTVALUE_EX increment{};
    if (MV_CC_GetIntValueEx(handle_, "DeviceTimestampIncrement", &increment) == MV_OK &&
        increment.nCurValue > 0)
    {
      ticks_per_second_ = static_cast<uint64_t>(increment.nCurValue);
    }
    else
    {
      XR_LOG_WARN("%s: DeviceTimestampIncrement unavailable, assuming 1 tick per us",
                  Name().c_str());
    }
    return trigger_ok;
  }

  /// 写跳采、宽高、偏移并读回核对；调用时不在取流 / Write decimation, size and offsets
  /// and read them back; the stream is stopped.
  bool WriteGeometry(const CameraTypes::FrameGeometry& geometry)
  {
    const HikCameraDetail::NodeGeometry node = HikCameraDetail::ToNodeGeometry(geometry);
    // 先把偏移清零，否则缩小跳采前的窗口可能越界 / Zero the offsets first so the window
    // stays inside the sensor while the decimation changes.
    const bool written =
        SetInt("OffsetX", 0) && SetInt("OffsetY", 0) &&
        Check(MV_CC_SetEnumValue(handle_, "DecimationHorizontal",
                                 static_cast<unsigned int>(node.decimation)),
              "DecimationHorizontal") &&
        Check(MV_CC_SetEnumValue(handle_, "DecimationVertical",
                                 static_cast<unsigned int>(node.decimation)),
              "DecimationVertical") &&
        SetInt("Width", node.width) && SetInt("Height", node.height) &&
        SetInt("OffsetX", node.offset_x) && SetInt("OffsetY", node.offset_y);
    if (!written)
    {
      return false;
    }

    MVCC_ENUMVALUE decimation{};
    MVCC_INTVALUE_EX width{}, height{}, offset_x{}, offset_y{};
    const bool read =
        MV_CC_GetEnumValue(handle_, "DecimationHorizontal", &decimation) == MV_OK &&
        MV_CC_GetIntValueEx(handle_, "Width", &width) == MV_OK &&
        MV_CC_GetIntValueEx(handle_, "Height", &height) == MV_OK &&
        MV_CC_GetIntValueEx(handle_, "OffsetX", &offset_x) == MV_OK &&
        MV_CC_GetIntValueEx(handle_, "OffsetY", &offset_y) == MV_OK;
    if (!read || decimation.nCurValue != node.decimation ||
        width.nCurValue != node.width || height.nCurValue != node.height ||
        offset_x.nCurValue != node.offset_x || offset_y.nCurValue != node.offset_y)
    {
      XR_LOG_ERROR("%s: geometry read back as d=%u %dx%d+%d+%d", Name().c_str(),
                   decimation.nCurValue, static_cast<int>(width.nCurValue),
                   static_cast<int>(height.nCurValue),
                   static_cast<int>(offset_x.nCurValue),
                   static_cast<int>(offset_y.nCurValue));
      return false;
    }
    return true;
  }

  bool SetInt(const char* node, int64_t value)
  {
    return Check(MV_CC_SetIntValueEx(handle_, node, value), node);
  }

  bool Check(int result, const char* what) const
  {
    if (result == MV_OK)
    {
      return true;
    }
    XR_LOG_ERROR("%s: %s failed: 0x%08x", Name().c_str(), what,
                 static_cast<unsigned>(result));
    return false;
  }

  void* handle_ = nullptr;
  uint32_t grab_timeout_ms_ = TRIGGERED_GRAB_TIMEOUT_MS;
  uint64_t ticks_per_second_ = 1000000;
};
