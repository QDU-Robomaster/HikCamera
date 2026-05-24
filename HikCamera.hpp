#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Hikrobot USB 相机采集模块，向 CameraBase 图像槽写入图像
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
      decimation_horizontal: 1
      decimation_vertical: 1
      rotate_180: false
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
 * 本模块从 Hikrobot SDK 读取 BGR8 图像，写入 `CameraBase` 当前可写图像槽。
 * `ImageFrame::timestamp_us` 使用相机设备时间戳换算得到的微秒值。
 *
 * @tparam CameraInfoV 相机输出图像尺寸、像素格式和内参。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class HikCamera : public LibXR::Application,
                  public CameraBase<CameraInfoV>
{
 public:
  using Self = HikCamera<CameraInfoV>;          ///< 当前模板实例类型。
  using Base = CameraBase<CameraInfoV>;         ///< CameraBase 基类类型。
  using ImageFrame = typename Base::ImageFrame; ///< 图像槽载荷类型。

  /// 编译期相机信息。
  static inline constexpr auto camera_info = Base::camera_info;
  /// Hik SDK 当前取图路径固定输出 BGR 三通道。
  static constexpr int channel_count = 3;
  /// 每行字节数。
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);
  /// 一秒对应的微秒数。
  static constexpr uint64_t microseconds_per_second = 1000000ULL;
  /// 等待图像槽时的日志周期。
  static constexpr uint32_t image_sink_wait_log_ms = 1000;
  /// 当前实机使用的增益上限。
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
    uint32_t decimation_horizontal = 1;  ///< 横向下采样倍率；1 表示不启用。
    uint32_t decimation_vertical = 1;  ///< 纵向下采样倍率；1 表示不启用。
    bool rotate_180 = false;  ///< true 时使用相机 ReverseX/Y 做 180 度旋转。
  };

  /**
   * @brief 打开相机、配置参数并启动采集线程。
   *
   * @param hw 硬件容器，传给 `CameraBase` 注册 RamFS 命令。
   * @param app 应用管理器。
   * @param runtime 运行时相机参数。
   *
   * 配置或开始取流失败时会抛出 `std::runtime_error`。
   */
  explicit HikCamera(LibXR::HardwareContainer& hw,
                     LibXR::ApplicationManager& app,
                     RuntimeParam runtime)
      : Base(hw, runtime.camera_name, runtime.image_topic_name, runtime.imu_topic_name),
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

  /**
   * @brief 停止采集线程，关闭相机并恢复启动前保存的相机设置。
   */
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
  /**
   * @brief 把增益限制在当前配置允许的范围内。
   */
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

  /**
   * @brief 写入 Hik SDK float 节点。
   */
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

  /**
   * @brief 写入 Hik SDK enum 节点。
   */
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

  /**
   * @brief 用字符串写入 Hik SDK enum 节点。
   */
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

  /**
   * @brief 读取 Hik SDK enum 节点。
   */
  bool GetEnumValue(const char* name, MVCC_ENUMVALUE& value, bool required = true)
  {
    const auto ret = MV_CC_GetEnumValue(camera_handle_, name, &value);
    if (ret != MV_OK)
    {
      if (required)
      {
        XR_LOG_ERROR("HikCamera MV_CC_GetEnumValue(%s) failed: %d", name, ret);
      }
      return false;
    }
    return true;
  }

  /**
   * @brief 读取 Hik SDK bool 节点。
   */
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

  /**
   * @brief 写入 Hik SDK bool 节点。
   */
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

  /**
   * @brief 读取 Hik SDK integer 节点。
   */
  bool GetIntValue(const char* name, MVCC_INTVALUE_EX& value)
  {
    const auto ret = MV_CC_GetIntValueEx(camera_handle_, name, &value);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_GetIntValueEx(%s) failed: %d", name, ret);
      return false;
    }
    return true;
  }

  /**
   * @brief 写入 Hik SDK integer 节点。
   */
  bool SetIntValue(const char* name, int64_t value)
  {
    const auto ret = MV_CC_SetIntValueEx(camera_handle_, name, value);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_SetIntValueEx(%s, %lld) failed: %d",
                   name, static_cast<long long>(value), ret);
      return false;
    }
    return true;
  }

  static bool IsIntegerValueAllowed(const MVCC_INTVALUE_EX& range, int64_t value)
  {
    if (value < range.nMin || value > range.nMax)
    {
      return false;
    }
    return range.nInc <= 1 || ((value - range.nMin) % range.nInc) == 0;
  }

  static int64_t AlignDown(int64_t value, const MVCC_INTVALUE_EX& range)
  {
    if (value < range.nMin)
    {
      value = range.nMin;
    }
    if (value > range.nMax)
    {
      value = range.nMax;
    }
    if (range.nInc <= 1)
    {
      return value;
    }
    return range.nMin + ((value - range.nMin) / range.nInc) * range.nInc;
  }

  /**
   * @brief 配置相机下采样倍率。
   *
   * `1x1` 表示不下采样。请求其它倍率时，相机必须提供对应 SDK 节点。
   */
  bool ConfigureDecimation()
  {
    if (runtime_.decimation_horizontal == 0 || runtime_.decimation_vertical == 0)
    {
      XR_LOG_ERROR("HikCamera decimation must be >= 1: horizontal=%u vertical=%u",
                   runtime_.decimation_horizontal, runtime_.decimation_vertical);
      return false;
    }

    MVCC_ENUMVALUE old_horizontal{};
    MVCC_ENUMVALUE old_vertical{};
    const bool wants_decimation = runtime_.decimation_horizontal != 1 ||
                                  runtime_.decimation_vertical != 1;
    const bool have_horizontal =
        GetEnumValue("DecimationHorizontal", old_horizontal, wants_decimation);
    const bool have_vertical =
        GetEnumValue("DecimationVertical", old_vertical, wants_decimation);
    if (!have_horizontal || !have_vertical)
    {
      if (wants_decimation)
      {
        XR_LOG_ERROR("HikCamera requested decimation %ux%u but camera nodes are unavailable",
                     runtime_.decimation_horizontal, runtime_.decimation_vertical);
        return false;
      }
      return true;
    }

    old_decimation_horizontal_ = old_horizontal.nCurValue;
    old_decimation_vertical_ = old_vertical.nCurValue;
    decimation_state_saved_ = true;

    if (!SetEnumValue("DecimationHorizontal", runtime_.decimation_horizontal) ||
        !SetEnumValue("DecimationVertical", runtime_.decimation_vertical))
    {
      return false;
    }

    XR_LOG_PASS("HikCamera decimation: horizontal=%u vertical=%u",
                runtime_.decimation_horizontal, runtime_.decimation_vertical);
    return true;
  }

  /**
   * @brief 按 `CameraInfoV` 配置输出图像尺寸和居中 ROI。
   *
   * 配置前会保存启动时的宽高、偏移和下采样设置，关闭相机时恢复。
   */
  bool ConfigureImageGeometry()
  {
    MVCC_INTVALUE_EX old_width{};
    MVCC_INTVALUE_EX old_height{};
    MVCC_INTVALUE_EX old_offset_x{};
    MVCC_INTVALUE_EX old_offset_y{};
    if (!GetIntValue("Width", old_width) || !GetIntValue("Height", old_height) ||
        !GetIntValue("OffsetX", old_offset_x) ||
        !GetIntValue("OffsetY", old_offset_y))
    {
      return false;
    }

    old_width_ = old_width.nCurValue;
    old_height_ = old_height.nCurValue;
    old_offset_x_ = old_offset_x.nCurValue;
    old_offset_y_ = old_offset_y.nCurValue;
    geometry_state_saved_ = true;

    if (!SetIntValue("OffsetX", 0) || !SetIntValue("OffsetY", 0) ||
        !ConfigureDecimation() ||
        !SetIntValue("OffsetX", 0) || !SetIntValue("OffsetY", 0) ||
        !GetIntValue("Width", full_width_range_) ||
        !GetIntValue("Height", full_height_range_))
    {
      return false;
    }

    const int64_t target_width = static_cast<int64_t>(camera_info.width);
    const int64_t target_height = static_cast<int64_t>(camera_info.height);
    if (!IsIntegerValueAllowed(full_width_range_, target_width) ||
        !IsIntegerValueAllowed(full_height_range_, target_height))
    {
      XR_LOG_ERROR("HikCamera image geometry invalid: target=%lldx%lld "
                   "width_range=[%lld,%lld/%lld] height_range=[%lld,%lld/%lld]",
                   static_cast<long long>(target_width),
                   static_cast<long long>(target_height),
                   static_cast<long long>(full_width_range_.nMin),
                   static_cast<long long>(full_width_range_.nMax),
                   static_cast<long long>(full_width_range_.nInc),
                   static_cast<long long>(full_height_range_.nMin),
                   static_cast<long long>(full_height_range_.nMax),
                   static_cast<long long>(full_height_range_.nInc));
      return false;
    }

    if (!SetIntValue("Width", target_width) ||
        !SetIntValue("Height", target_height))
    {
      return false;
    }

    MVCC_INTVALUE_EX offset_x_range{};
    MVCC_INTVALUE_EX offset_y_range{};
    if (!GetIntValue("OffsetX", offset_x_range) ||
        !GetIntValue("OffsetY", offset_y_range))
    {
      return false;
    }

    const int64_t offset_x =
        AlignDown((full_width_range_.nMax - target_width) / 2, offset_x_range);
    const int64_t offset_y =
        AlignDown((full_height_range_.nMax - target_height) / 2, offset_y_range);
    if (!SetIntValue("OffsetX", offset_x) || !SetIntValue("OffsetY", offset_y))
    {
      return false;
    }

    XR_LOG_PASS("HikCamera image geometry: max=%lldx%lld roi=%lldx%lld "
                "offset=%lld,%lld decimation=%ux%u",
                static_cast<long long>(full_width_range_.nMax),
                static_cast<long long>(full_height_range_.nMax),
                static_cast<long long>(target_width),
                static_cast<long long>(target_height),
                static_cast<long long>(offset_x),
                static_cast<long long>(offset_y),
                runtime_.decimation_horizontal, runtime_.decimation_vertical);
    return true;
  }

  /**
   * @brief 恢复启动前保存的图像尺寸、偏移和下采样设置。
   */
  void RestoreImageGeometry()
  {
    if (!geometry_state_saved_ || camera_handle_ == nullptr)
    {
      return;
    }

    (void)SetIntValue("OffsetX", 0);
    (void)SetIntValue("OffsetY", 0);
    if (decimation_state_saved_)
    {
      (void)SetEnumValue("DecimationHorizontal", old_decimation_horizontal_);
      (void)SetEnumValue("DecimationVertical", old_decimation_vertical_);
      decimation_state_saved_ = false;
    }
    (void)SetIntValue("Width", old_width_);
    (void)SetIntValue("Height", old_height_);
    (void)SetIntValue("OffsetX", old_offset_x_);
    (void)SetIntValue("OffsetY", old_offset_y_);
    geometry_state_saved_ = false;
  }

  /**
   * @brief 配置 180 度图像旋转。
   *
   * 旋转通过相机 `ReverseX` 和 `ReverseY` 节点完成。请求旋转但节点不可用时启动失败。
   */
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

  /**
   * @brief 返回当前旋转方式名称，用于启动日志。
   */
  const char* RotationModeName() const
  {
    if (device_rotate_180_)
    {
      return "device_reverse_xy";
    }
    return "none";
  }

  /**
   * @brief 恢复启动前保存的 `ReverseX` 和 `ReverseY` 设置。
   */
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

  /**
   * @brief 枚举 USB 相机、打开设备并配置采集参数。
   */
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

    if (!ConfigureImageGeometry())
    {
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

  /**
   * @brief 开始 Hik SDK 取流。
   */
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

  /**
   * @brief 停止取流、恢复相机设置并销毁 SDK handle。
   */
  void CaptureStop()
  {
    if (camera_handle_ == nullptr)
    {
      return;
    }
    (void)MV_CC_StopGrabbing(camera_handle_);
    RestoreDeviceRotation();
    RestoreImageGeometry();
    (void)MV_CC_CloseDevice(camera_handle_);
    (void)MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
  }

  /**
   * @brief 下发当前曝光和增益设置。
   */
  void UpdateParameters()
  {
    if (camera_handle_ == nullptr)
    {
      return;
    }
    (void)SetFloatValue("ExposureTime", runtime_.exposure_time);
    (void)SetFloatValue("Gain", runtime_.gain);
  }

  /**
   * @brief 读取设备时间戳频率。
   *
   * 读取失败时按微秒 tick 处理，并打印警告。
   */
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

  /**
   * @brief 从 SDK 帧信息计算图像时间戳，单位微秒。
   *
   * 没有设备时间戳的帧会被拒绝。
   */
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

  /**
   * @brief 合并 Hik SDK 提供的高低 32 位时间戳。
   */
  static uint64_t CombineU32(uint32_t high, uint32_t low)
  {
    return (static_cast<uint64_t>(high) << 32U) | static_cast<uint64_t>(low);
  }

  /**
   * @brief 把设备 tick 换算成微秒。
   */
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

  /**
   * @brief 打印首帧时间戳和 SDK 帧号信息。
   */
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

  /**
   * @brief 等待其它模块注册图像槽。
   */
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

  /**
   * @brief 采集循环：取图、检查尺寸、写时间戳并提交图像。
   */
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

  /**
   * @brief `std::thread` 入口。
   */
  static void CaptureThreadMain(Self* self) { self->CaptureLoop(); }

 private:
  RuntimeParam runtime_{};  ///< 当前运行参数快照。
  void* camera_handle_{nullptr};  ///< Hik SDK 设备 handle。
  std::atomic<bool> camera_state_{false};  ///< 采集线程运行标志。
  std::thread capture_thread_{};  ///< 采集线程。
  bool capture_thread_created_{false};  ///< 线程是否已创建。
  bool device_rotate_180_{false};  ///< 当前是否由相机完成 180 度旋转。
  bool reverse_state_saved_{false};  ///< 是否保存过 ReverseX / ReverseY 原值。
  bool geometry_state_saved_{false};  ///< 是否保存过宽高和偏移原值。
  bool decimation_state_saved_{false};  ///< 是否保存过下采样原值。
  bool old_reverse_x_{false};  ///< 启动前的 ReverseX。
  bool old_reverse_y_{false};  ///< 启动前的 ReverseY。
  uint32_t old_decimation_horizontal_{1};  ///< 启动前的横向下采样。
  uint32_t old_decimation_vertical_{1};  ///< 启动前的纵向下采样。
  int64_t old_width_{0};  ///< 启动前的宽度。
  int64_t old_height_{0};  ///< 启动前的高度。
  int64_t old_offset_x_{0};  ///< 启动前的 X 偏移。
  int64_t old_offset_y_{0};  ///< 启动前的 Y 偏移。
  MVCC_INTVALUE_EX full_width_range_{};  ///< 相机支持的宽度范围。
  MVCC_INTVALUE_EX full_height_range_{};  ///< 相机支持的高度范围。
  uint64_t device_timestamp_frequency_hz_{microseconds_per_second};  ///< 设备时间戳频率。
  uint32_t frames_committed_{0};  ///< 已提交帧数。
  uint32_t failure_count_{0};  ///< 失败帧数。
};
