#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Hikrobot USB 相机采集模块，向 CameraBase 图像槽写入图像
constructor_args:
  - calibration:
      native_width: 1440
      native_height: 1080
      camera_matrix: [2328.6857198980888, 0.0, 733.35646250924742, 0.0, 2328.6701077899961, 540.61872869227727, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [-0.091821039187099038, 0.46399073468302049, 0.0026098786426372819, 0.0009819586010405485, -0.47512788503104569]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [2328.6857198980888, 0.0, 733.35646250924742, 0.0, 0.0, 2328.6701077899961, 540.61872869227727, 0.0, 0.0, 0.0, 1.0, 0.0]
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
      wide_decimation_x: 2
      wide_decimation_y: 2
      wide_trigger_period_us: 10000
      narrow_trigger_period_us: 5000
template_args:
  - Layout:
      width: 720
      height: 540
      step: 2160
      encoding: CameraTypes::Encoding::BGR8
required_hardware:
  - Hikrobot USB camera
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>

#include "CameraBase.hpp"
#include "HikCameraProfileControl.hpp"
#include "MvCameraControl.h"
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
 * @tparam FrameLayoutV 相机输出图像的固定存储容量和像素格式。
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class HikCamera : public LibXR::Application, public CameraBase<FrameLayoutV>
{
 public:
  using Self = HikCamera<FrameLayoutV>;                        ///< 当前模板实例类型。
  using Base = CameraBase<FrameLayoutV>;                       ///< CameraBase 基类类型。
  using ImageFrame = typename Base::ImageFrame;                ///< 图像槽载荷类型。
  using CameraCalibration = typename Base::CameraCalibration;  ///< 原生相机标定。
  using FrameGeometry = typename Base::FrameGeometry;          ///< 逐帧采样几何。
  using ProfileId = typename Base::ProfileId;                  ///< 固定档位标识。
  using CameraProfile = typename Base::CameraProfile;          ///< 固定档位描述。
  using AppliedProfile = typename Base::AppliedProfile;        ///< 已应用档位快照。

  /// 编译期帧存储布局。
  static inline constexpr auto frame_layout = Base::frame_layout;
  /// Hik SDK 当前取图路径固定输出 BGR 三通道。
  static constexpr int channel_count = 3;
  /// 每行字节数。
  static constexpr std::size_t frame_step = static_cast<std::size_t>(frame_layout.step);
  /// 一秒对应的微秒数。
  static constexpr uint64_t microseconds_per_second = 1000000ULL;
  /// 当前实机使用的增益上限。
  static constexpr float max_gain = 16.0F;
  /// 产品宽视场档位的默认触发周期。
  static constexpr uint32_t default_wide_trigger_period_us = 10000U;
  /// 产品窄视场档位的默认触发周期。
  static constexpr uint32_t default_narrow_trigger_period_us = 5000U;
  /// 产品宽视场档位的默认横向下采样倍率。
  static constexpr uint32_t default_wide_decimation_x = 2U;
  /// 产品宽视场档位的默认纵向下采样倍率。
  static constexpr uint32_t default_wide_decimation_y = 2U;

  static_assert(frame_layout.encoding == CameraTypes::Encoding::BGR8,
                "HikCamera publishes BGR8 frames through MV_CC_GetImageForBGR");
  static_assert(frame_step ==
                    static_cast<std::size_t>(frame_layout.width) * channel_count,
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
    std::string_view camera_name = "camera";             ///< CameraBase 相机名。
    std::string_view image_topic_name = "camera_image";  ///< 图像共享话题名。
    std::string_view imu_topic_name = "camera_imu";      ///< 同步后 IMU 话题名。
    float gain = 16.0F;                                  ///< 相机增益。
    float exposure_time = 2000.0F;                       ///< 曝光时间，单位微秒。
    bool external_trigger = true;           ///< true 时使用 Line0 上升沿外触发。
    float acquisition_frame_rate = 249.0F;  ///< 非外触发模式下的自由运行帧率。
    uint32_t grab_timeout_ms = 100;         ///< SDK 等待一帧图像的超时时间。
    uint32_t image_node_num = 3;            ///< SDK 内部取流缓存节点数。
    bool rotate_180 = false;  ///< true 时使用相机 ReverseX/Y 做 180 度旋转。
    uint32_t wide_decimation_x = default_wide_decimation_x;  ///< WIDE 档横向下采样倍率。
    uint32_t wide_decimation_y = default_wide_decimation_y;  ///< WIDE 档纵向下采样倍率。
    uint32_t wide_trigger_period_us =
        default_wide_trigger_period_us;  ///< WIDE 档外触发周期，单位 us。
    uint32_t narrow_trigger_period_us =
        default_narrow_trigger_period_us;  ///< NARROW 档外触发周期，单位 us。

    RuntimeParam() = default;

    constexpr RuntimeParam(std::string_view camera_name,
                           std::string_view image_topic_name,
                           std::string_view imu_topic_name, float gain,
                           float exposure_time, bool external_trigger,
                           float acquisition_frame_rate, uint32_t grab_timeout_ms,
                           uint32_t image_node_num, bool rotate_180)
        : camera_name(camera_name),
          image_topic_name(image_topic_name),
          imu_topic_name(imu_topic_name),
          gain(gain),
          exposure_time(exposure_time),
          external_trigger(external_trigger),
          acquisition_frame_rate(acquisition_frame_rate),
          grab_timeout_ms(grab_timeout_ms),
          image_node_num(image_node_num),
          rotate_180(rotate_180)
    {
    }

    /** 兼容旧 YAML 中位于 rotate_180 之前的两个下采样字段。 */
    constexpr RuntimeParam(std::string_view camera_name,
                           std::string_view image_topic_name,
                           std::string_view imu_topic_name, float gain,
                           float exposure_time, bool external_trigger,
                           float acquisition_frame_rate, uint32_t grab_timeout_ms,
                           uint32_t image_node_num, uint32_t decimation_horizontal,
                           uint32_t decimation_vertical, bool rotate_180)
        : RuntimeParam(camera_name, image_topic_name, imu_topic_name, gain, exposure_time,
                       external_trigger, acquisition_frame_rate, grab_timeout_ms,
                       image_node_num, rotate_180)
    {
      wide_decimation_x = decimation_horizontal;
      wide_decimation_y = decimation_vertical;
    }

    constexpr RuntimeParam(std::string_view camera_name,
                           std::string_view image_topic_name,
                           std::string_view imu_topic_name, float gain,
                           float exposure_time, bool external_trigger,
                           float acquisition_frame_rate, uint32_t grab_timeout_ms,
                           uint32_t image_node_num, bool rotate_180,
                           uint32_t wide_decimation_x, uint32_t wide_decimation_y,
                           uint32_t wide_trigger_period_us,
                           uint32_t narrow_trigger_period_us)
        : RuntimeParam(camera_name, image_topic_name, imu_topic_name, gain, exposure_time,
                       external_trigger, acquisition_frame_rate, grab_timeout_ms,
                       image_node_num, rotate_180)
    {
      this->wide_decimation_x = wide_decimation_x;
      this->wide_decimation_y = wide_decimation_y;
      this->wide_trigger_period_us = wide_trigger_period_us;
      this->narrow_trigger_period_us = narrow_trigger_period_us;
    }
  };

  /**
   * @brief 打开相机、配置参数并启动采集线程。
   *
   * @param hw 硬件容器，传给 `CameraBase` 注册 RamFS 命令。
   * @param app 应用管理器。
   * @param calibration 原生传感器坐标系下的相机标定。
   * @param runtime 运行时相机参数。
   *
   * 配置或开始取流失败时会抛出 `std::runtime_error`。
   */
  explicit HikCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                     CameraCalibration calibration, RuntimeParam runtime)
      : Base(hw, calibration, runtime.camera_name, runtime.image_topic_name,
             runtime.imu_topic_name),
        runtime_(runtime)
  {
    runtime_.gain = ClampGain(runtime_.gain);
    XR_LOG_INFO("Starting HikCamera: external_trigger=%d rotate_180=%d",
                runtime_.external_trigger ? 1 : 0, runtime_.rotate_180 ? 1 : 0);
    if (!(ValidateRuntimeProfileConfig() && CaptureStart() && InitializeProfiles() &&
          StartGrabbing() && StartCaptureThread()))
    {
      (void)CaptureStop(true);
      throw std::runtime_error("HikCamera: failed to start camera");
    }
    app.Register(*this);
  }

  /**
   * @brief 停止采集线程，关闭相机并恢复启动前保存的相机设置。
   */
  ~HikCamera() noexcept override
  {
    LibXR::Mutex::LockGuard lock(control_mutex_);
    const auto stopped = CaptureStop(true);
    if (!stopped.AllSucceeded())
    {
      XR_LOG_ERROR(
          "HikCamera teardown incomplete: thread=%d stop=%d rotation=%d "
          "geometry=%d close=%d destroy=%d",
          stopped.thread_stopped ? 1 : 0, stopped.stream_stopped ? 1 : 0,
          stopped.rotation_restored ? 1 : 0, stopped.geometry_restored ? 1 : 0,
          stopped.device_closed ? 1 : 0, stopped.handle_destroyed ? 1 : 0);
    }
  }

  void OnMonitor() override
  {
    XR_LOG_INFO("HikCamera monitor: frames=%u failures=%u",
                frames_committed_.load(std::memory_order_relaxed),
                failure_count_.load(std::memory_order_relaxed));
  }

  void SetExposure(double exposure) override
  {
    LibXR::Mutex::LockGuard lock(control_mutex_);
    runtime_.exposure_time = static_cast<float>(exposure);
    UpdateParameters();
  }

  void SetGain(double gain) override
  {
    LibXR::Mutex::LockGuard lock(control_mutex_);
    runtime_.gain = ClampGain(static_cast<float>(gain));
    UpdateParameters();
  }

  /**
   * @brief 返回固定的 WIDE/NARROW 档位表。
   */
  [[nodiscard]] std::span<const CameraProfile> Profiles() const noexcept override
  {
    return profiles_;
  }

  /**
   * @brief 停止 SDK 取流并阻塞切换到指定档位。
   *
   * 调用方必须先停止外部触发。切换失败时先在原 handle 上恢复上一档；恢复失败时
   * 关闭并重开设备，再尝试恢复上一档。请求失败时不会改写 `applied`。
   */
  LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) override
  {
    LibXR::Mutex::LockGuard lock(control_mutex_);
    const CameraProfile* requested = FindProfile(id);
    if (requested == nullptr)
    {
      return LibXR::ErrorCode::NOT_SUPPORT;
    }
    if (id == active_profile_)
    {
      if (!camera_state_.load(std::memory_order_acquire))
      {
        if (!ReopenProfile(id, requested->geometry))
        {
          return LibXR::ErrorCode::STATE_ERR;
        }
      }
      applied = {.id = id, .geometry = frame_geometry_};
      return LibXR::ErrorCode::OK;
    }

    const CameraProfile* previous = FindProfile(active_profile_);
    if (previous == nullptr)
    {
      return LibXR::ErrorCode::STATE_ERR;
    }

    if (!StopCaptureThread())
    {
      return LibXR::ErrorCode::FAILED;
    }
    this->DiscardWritableImage();

    const auto outcome = HikCameraDetail::RunProfileSwitchWithRecovery(
        [this, id, requested]()
        {
          return PrepareProfileSwitch(id) && TryActivateProfile(id, requested->geometry);
        },
        [this, previous]()
        { return TryActivateProfile(previous->id, previous->geometry); },
        [this, previous]() { return ReopenProfile(previous->id, previous->geometry); });

    if (outcome == HikCameraDetail::ProfileSwitchOutcome::APPLIED)
    {
      active_profile_ = id;
      applied = {.id = id, .geometry = frame_geometry_};
      return LibXR::ErrorCode::OK;
    }

    if (outcome == HikCameraDetail::ProfileSwitchOutcome::ROLLED_BACK)
    {
      XR_LOG_WARN("HikCamera profile=%u failed; previous profile restored",
                  static_cast<unsigned>(id));
    }
    else if (outcome == HikCameraDetail::ProfileSwitchOutcome::REOPENED)
    {
      XR_LOG_WARN("HikCamera profile=%u failed; device reopened on previous profile",
                  static_cast<unsigned>(id));
    }
    else
    {
      XR_LOG_ERROR("HikCamera profile=%u failed and recovery was unsuccessful",
                   static_cast<unsigned>(id));
    }
    return LibXR::ErrorCode::FAILED;
  }

 private:
  struct ImageGeometryState
  {
    int64_t width{0};
    int64_t height{0};
    int64_t offset_x{0};
    int64_t offset_y{0};
  };

  struct DecimationState
  {
    uint32_t decimation_horizontal{1};
    uint32_t decimation_vertical{1};
  };

  struct RotationState
  {
    bool reverse_x{false};
    bool reverse_y{false};
  };

  struct CaptureStopResult
  {
    bool thread_stopped{true};
    bool stream_stopped{true};
    bool rotation_restored{true};
    bool geometry_restored{true};
    bool device_closed{true};
    bool handle_destroyed{true};

    [[nodiscard]] bool HandleReleased() const noexcept { return handle_destroyed; }

    [[nodiscard]] bool AllSucceeded() const noexcept
    {
      return thread_stopped && stream_stopped && rotation_restored && geometry_restored &&
             device_closed && handle_destroyed;
    }
  };

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
   * @brief 检查档位下采样和触发周期是否可用于相机配置。
   */
  bool ValidateRuntimeProfileConfig() const
  {
    if (runtime_.wide_decimation_x == 0U || runtime_.wide_decimation_y == 0U ||
        runtime_.wide_trigger_period_us == 0U || runtime_.narrow_trigger_period_us == 0U)
    {
      XR_LOG_ERROR(
          "HikCamera invalid profile config: wide_decimation=%ux%u "
          "trigger_period_us=%u/%u",
          runtime_.wide_decimation_x, runtime_.wide_decimation_y,
          runtime_.wide_trigger_period_us, runtime_.narrow_trigger_period_us);
      return false;
    }
    return true;
  }

  bool InitializeProfiles()
  {
    const auto& calibration = this->Calibration();
    if (frame_geometry_.roi_offset_x_native != 0U ||
        frame_geometry_.roi_offset_y_native != 0U ||
        frame_geometry_.decimation_x != runtime_.wide_decimation_x ||
        frame_geometry_.decimation_y != runtime_.wide_decimation_y ||
        static_cast<uint64_t>(frame_geometry_.width) * frame_geometry_.decimation_x !=
            calibration.native_width ||
        static_cast<uint64_t>(frame_geometry_.height) * frame_geometry_.decimation_y !=
            calibration.native_height)
    {
      XR_LOG_ERROR("HikCamera WIDE profile does not cover the native sensor");
      return false;
    }

    const FrameGeometry wide = frame_geometry_;
    if (!ConfigureImageGeometry(ProfileId::NARROW))
    {
      XR_LOG_ERROR("HikCamera failed to probe NARROW profile geometry");
      return false;
    }
    const FrameGeometry narrow = frame_geometry_;
    if (!ConfigureImageGeometry(ProfileId::WIDE) ||
        !CameraTypes::SameFrameGeometry(frame_geometry_, wide))
    {
      XR_LOG_ERROR("HikCamera failed to restore WIDE geometry after profile probe");
      return false;
    }

    profiles_[0] = {.id = ProfileId::WIDE,
                    .geometry = wide,
                    .trigger_period_us = runtime_.wide_trigger_period_us};
    profiles_[1] = {.id = ProfileId::NARROW,
                    .geometry = narrow,
                    .trigger_period_us = runtime_.narrow_trigger_period_us};
    active_profile_ = ProfileId::WIDE;
    return true;
  }

  [[nodiscard]] const CameraProfile* FindProfile(ProfileId id) const noexcept
  {
    for (const auto& profile : profiles_)
    {
      if (profile.id == id)
      {
        return &profile;
      }
    }
    return nullptr;
  }

  [[nodiscard]] bool StopCaptureThread() noexcept
  {
    camera_state_.store(false, std::memory_order_release);
    if (!capture_thread_.joinable())
    {
      return true;
    }
    try
    {
      capture_thread_.join();
      return true;
    }
    catch (const std::system_error& error)
    {
      XR_LOG_ERROR("HikCamera failed to join capture thread: %s", error.what());
      return false;
    }
  }

  bool PrepareProfileSwitch(ProfileId requested)
  {
    if (camera_handle_ == nullptr)
    {
      XR_LOG_ERROR("HikCamera failed to switch profile=%u; camera handle is null",
                   static_cast<unsigned>(requested));
      return false;
    }

    const int clear_result = MV_CC_ClearImageBuffer(camera_handle_);
    const bool stream_stopped = StopGrabbing("profile switch");
    if (clear_result != MV_OK || !stream_stopped)
    {
      XR_LOG_ERROR("HikCamera failed to prepare profile=%u; clear=%d stop=%d",
                   static_cast<unsigned>(requested), clear_result,
                   stream_stopped ? 1 : 0);
      return false;
    }
    return true;
  }

  bool TryActivateProfile(ProfileId id, const FrameGeometry& expected_geometry)
  {
    return camera_handle_ != nullptr && !stream_state_.Active() &&
           ConfigureImageGeometry(id) &&
           CameraTypes::SameFrameGeometry(frame_geometry_, expected_geometry) &&
           StartGrabbing() && StartCaptureThread();
  }

  bool ReopenProfile(ProfileId id, const FrameGeometry& expected_geometry)
  {
    const auto stopped = CaptureStop(false);
    if (!stopped.HandleReleased())
    {
      XR_LOG_ERROR(
          "HikCamera cannot reopen profile=%u because the previous handle remains live",
          static_cast<unsigned>(id));
      return false;
    }
    if (!CaptureStart())
    {
      (void)CaptureStop(false);
      return false;
    }

    const bool geometry_ready =
        (id == ProfileId::WIDE || ConfigureImageGeometry(id)) &&
        CameraTypes::SameFrameGeometry(frame_geometry_, expected_geometry);
    if (!geometry_ready || !StartGrabbing() || !StartCaptureThread())
    {
      (void)CaptureStop(false);
      return false;
    }
    return true;
  }

  /**
   * @brief 写入 Hik SDK float 节点。
   */
  bool SetFloatValue(const char* name, double value)
  {
    const auto ret = MV_CC_SetFloatValue(camera_handle_, name, static_cast<float>(value));
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_SetFloatValue(%s, %.3f) failed: %d", name, value,
                   ret);
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
      XR_LOG_ERROR("HikCamera MV_CC_SetEnumValue(%s, %u) failed: %d", name, value, ret);
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
      XR_LOG_ERROR("HikCamera MV_CC_SetEnumValueByString(%s, %s) failed: %d", name, value,
                   ret);
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
      XR_LOG_WARN("HikCamera MV_CC_SetBoolValue(%s, %d) failed: %d", name, value ? 1 : 0,
                  ret);
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
      XR_LOG_ERROR("HikCamera MV_CC_SetIntValueEx(%s, %lld) failed: %d", name,
                   static_cast<long long>(value), ret);
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

  /**
   * @brief 配置相机下采样倍率。
   *
   * `1x1` 表示不下采样。请求其它倍率时，相机必须提供对应 SDK 节点。
   */
  bool ConfigureDecimation(uint32_t horizontal, uint32_t vertical)
  {
    if (horizontal == 0U || vertical == 0U)
    {
      XR_LOG_ERROR("HikCamera decimation must be >= 1: horizontal=%u vertical=%u",
                   horizontal, vertical);
      return false;
    }

    MVCC_ENUMVALUE old_horizontal{};
    MVCC_ENUMVALUE old_vertical{};
    const bool wants_decimation = horizontal != 1U || vertical != 1U;
    const bool have_horizontal =
        GetEnumValue("DecimationHorizontal", old_horizontal, wants_decimation);
    const bool have_vertical =
        GetEnumValue("DecimationVertical", old_vertical, wants_decimation);
    if (!have_horizontal || !have_vertical)
    {
      if (wants_decimation)
      {
        XR_LOG_ERROR(
            "HikCamera requested decimation %ux%u but camera nodes are unavailable",
            horizontal, vertical);
        return false;
      }
      applied_decimation_horizontal_ = 1;
      applied_decimation_vertical_ = 1;
      return true;
    }

    (void)original_decimation_state_.CaptureOnce(
        {.decimation_horizontal = old_horizontal.nCurValue,
         .decimation_vertical = old_vertical.nCurValue});

    if (!SetEnumValue("DecimationHorizontal", horizontal) ||
        !SetEnumValue("DecimationVertical", vertical))
    {
      return false;
    }

    MVCC_ENUMVALUE applied_horizontal{};
    MVCC_ENUMVALUE applied_vertical{};
    if (!GetEnumValue("DecimationHorizontal", applied_horizontal) ||
        !GetEnumValue("DecimationVertical", applied_vertical) ||
        applied_horizontal.nCurValue != horizontal ||
        applied_vertical.nCurValue != vertical ||
        applied_horizontal.nCurValue > std::numeric_limits<uint16_t>::max() ||
        applied_vertical.nCurValue > std::numeric_limits<uint16_t>::max())
    {
      XR_LOG_ERROR(
          "HikCamera decimation readback mismatch: requested=%ux%u "
          "applied=%ux%u",
          horizontal, vertical, applied_horizontal.nCurValue, applied_vertical.nCurValue);
      return false;
    }
    applied_decimation_horizontal_ = applied_horizontal.nCurValue;
    applied_decimation_vertical_ = applied_vertical.nCurValue;

    XR_LOG_PASS("HikCamera decimation: horizontal=%u vertical=%u",
                applied_decimation_horizontal_, applied_decimation_vertical_);
    return true;
  }

  /**
   * @brief 按固定档位配置 `FrameLayoutV` 输出尺寸、ROI 和下采样。
   *
   * 配置前会保存启动时的宽高、偏移和下采样设置，关闭相机时恢复。
   */
  bool ConfigureImageGeometry(ProfileId profile)
  {
    const uint16_t geometry_flags = frame_geometry_.flags;
    MVCC_INTVALUE_EX old_width{};
    MVCC_INTVALUE_EX old_height{};
    MVCC_INTVALUE_EX old_offset_x{};
    MVCC_INTVALUE_EX old_offset_y{};
    if (!GetIntValue("Width", old_width) || !GetIntValue("Height", old_height) ||
        !GetIntValue("OffsetX", old_offset_x) || !GetIntValue("OffsetY", old_offset_y))
    {
      return false;
    }

    (void)original_geometry_state_.CaptureOnce({.width = old_width.nCurValue,
                                                .height = old_height.nCurValue,
                                                .offset_x = old_offset_x.nCurValue,
                                                .offset_y = old_offset_y.nCurValue});

    uint32_t decimation_x = 0U;
    uint32_t decimation_y = 0U;
    if (profile == ProfileId::WIDE)
    {
      decimation_x = runtime_.wide_decimation_x;
      decimation_y = runtime_.wide_decimation_y;
    }
    else if (profile == ProfileId::NARROW)
    {
      decimation_x = 1U;
      decimation_y = 1U;
    }
    else
    {
      return false;
    }

    if (!SetIntValue("OffsetX", 0) || !SetIntValue("OffsetY", 0) ||
        !ConfigureDecimation(decimation_x, decimation_y) || !SetIntValue("OffsetX", 0) ||
        !SetIntValue("OffsetY", 0) || !GetIntValue("Width", full_width_range_) ||
        !GetIntValue("Height", full_height_range_))
    {
      return false;
    }

    const int64_t target_width = static_cast<int64_t>(frame_layout.width);
    const int64_t target_height = static_cast<int64_t>(frame_layout.height);
    if (!IsIntegerValueAllowed(full_width_range_, target_width) ||
        !IsIntegerValueAllowed(full_height_range_, target_height) ||
        (profile == ProfileId::WIDE && (target_width != full_width_range_.nMax ||
                                        target_height != full_height_range_.nMax)))
    {
      XR_LOG_ERROR(
          "HikCamera profile geometry invalid: profile=%u target=%lldx%lld "
          "width_range=[%lld,%lld/%lld] height_range=[%lld,%lld/%lld]",
          static_cast<unsigned>(profile), static_cast<long long>(target_width),
          static_cast<long long>(target_height),
          static_cast<long long>(full_width_range_.nMin),
          static_cast<long long>(full_width_range_.nMax),
          static_cast<long long>(full_width_range_.nInc),
          static_cast<long long>(full_height_range_.nMin),
          static_cast<long long>(full_height_range_.nMax),
          static_cast<long long>(full_height_range_.nInc));
      return false;
    }

    if (!SetIntValue("Width", target_width) || !SetIntValue("Height", target_height))
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
        profile == ProfileId::WIDE
            ? 0
            : HikCameraDetail::CenteredAlignedOffset(full_width_range_.nMax, target_width,
                                                     {.minimum = offset_x_range.nMin,
                                                      .maximum = offset_x_range.nMax,
                                                      .increment = offset_x_range.nInc});
    const int64_t offset_y = profile == ProfileId::WIDE
                                 ? 0
                                 : HikCameraDetail::CenteredAlignedOffset(
                                       full_height_range_.nMax, target_height,
                                       {.minimum = offset_y_range.nMin,
                                        .maximum = offset_y_range.nMax,
                                        .increment = offset_y_range.nInc});
    if (!SetIntValue("OffsetX", offset_x) || !SetIntValue("OffsetY", offset_y))
    {
      return false;
    }

    MVCC_INTVALUE_EX applied_width{};
    MVCC_INTVALUE_EX applied_height{};
    MVCC_INTVALUE_EX applied_offset_x{};
    MVCC_INTVALUE_EX applied_offset_y{};
    if (!GetIntValue("Width", applied_width) || !GetIntValue("Height", applied_height) ||
        !GetIntValue("OffsetX", applied_offset_x) ||
        !GetIntValue("OffsetY", applied_offset_y) ||
        applied_width.nCurValue != target_width ||
        applied_height.nCurValue != target_height ||
        applied_offset_x.nCurValue != offset_x || applied_offset_y.nCurValue != offset_y)
    {
      XR_LOG_ERROR(
          "HikCamera profile geometry readback mismatch: "
          "size=%lldx%lld offset=%lld,%lld",
          static_cast<long long>(applied_width.nCurValue),
          static_cast<long long>(applied_height.nCurValue),
          static_cast<long long>(applied_offset_x.nCurValue),
          static_cast<long long>(applied_offset_y.nCurValue));
      return false;
    }

    const uint64_t native_offset_x = static_cast<uint64_t>(applied_offset_x.nCurValue) *
                                     applied_decimation_horizontal_;
    const uint64_t native_offset_y =
        static_cast<uint64_t>(applied_offset_y.nCurValue) * applied_decimation_vertical_;
    const uint64_t native_width =
        static_cast<uint64_t>(applied_width.nCurValue) * applied_decimation_horizontal_;
    const uint64_t native_height =
        static_cast<uint64_t>(applied_height.nCurValue) * applied_decimation_vertical_;
    const auto& calibration = this->Calibration();
    if (native_offset_x + native_width > calibration.native_width ||
        native_offset_y + native_height > calibration.native_height ||
        (profile == ProfileId::WIDE && (native_offset_x != 0U || native_offset_y != 0U ||
                                        native_width != calibration.native_width ||
                                        native_height != calibration.native_height)))
    {
      XR_LOG_ERROR(
          "HikCamera calibration/native geometry mismatch: applied=%llux%llu "
          "calibration=%ux%u",
          static_cast<unsigned long long>(native_width),
          static_cast<unsigned long long>(native_height), calibration.native_width,
          calibration.native_height);
      return false;
    }

    frame_geometry_ = FrameGeometry{
        static_cast<uint32_t>(applied_width.nCurValue),
        static_cast<uint32_t>(applied_height.nCurValue),
        frame_layout.step,
        static_cast<uint32_t>(native_offset_x),
        static_cast<uint32_t>(native_offset_y),
        static_cast<uint16_t>(applied_decimation_horizontal_),
        static_cast<uint16_t>(applied_decimation_vertical_),
        geometry_flags,
        0,
        0.0F,
        0.0F,
    };
    if (!CameraTypes::ValidateFrameGeometry(frame_layout, calibration, frame_geometry_))
    {
      XR_LOG_ERROR("HikCamera generated invalid profile FrameGeometry");
      return false;
    }

    XR_LOG_PASS(
        "HikCamera image geometry: profile=%u max=%lldx%lld roi=%lldx%lld "
        "offset=%lld,%lld decimation=%ux%u",
        static_cast<unsigned>(profile), static_cast<long long>(full_width_range_.nMax),
        static_cast<long long>(full_height_range_.nMax),
        static_cast<long long>(target_width), static_cast<long long>(target_height),
        static_cast<long long>(offset_x), static_cast<long long>(offset_y),
        applied_decimation_horizontal_, applied_decimation_vertical_);
    return true;
  }

  /**
   * @brief 恢复启动前保存的图像尺寸、偏移和下采样设置。
   */
  [[nodiscard]] bool RestoreImageGeometry()
  {
    const auto* geometry = original_geometry_state_.Get();
    const auto* decimation = original_decimation_state_.Get();
    if (geometry == nullptr && decimation == nullptr)
    {
      return true;
    }
    if (camera_handle_ == nullptr)
    {
      return false;
    }

    bool geometry_writes_succeeded = true;
    bool decimation_writes_succeeded = true;
    const auto record_geometry_write = [&geometry_writes_succeeded](bool result)
    { geometry_writes_succeeded = result && geometry_writes_succeeded; };
    const auto record_decimation_write = [&decimation_writes_succeeded](bool result)
    { decimation_writes_succeeded = result && decimation_writes_succeeded; };

    if (geometry != nullptr)
    {
      record_geometry_write(SetIntValue("OffsetX", 0));
      record_geometry_write(SetIntValue("OffsetY", 0));
    }
    if (decimation != nullptr)
    {
      record_decimation_write(
          SetEnumValue("DecimationHorizontal", decimation->decimation_horizontal));
      record_decimation_write(
          SetEnumValue("DecimationVertical", decimation->decimation_vertical));
    }
    if (geometry != nullptr)
    {
      record_geometry_write(SetIntValue("Width", geometry->width));
      record_geometry_write(SetIntValue("Height", geometry->height));
      record_geometry_write(SetIntValue("OffsetX", geometry->offset_x));
      record_geometry_write(SetIntValue("OffsetY", geometry->offset_y));
    }

    bool geometry_readback_matches = true;
    if (geometry != nullptr)
    {
      MVCC_INTVALUE_EX width{};
      MVCC_INTVALUE_EX height{};
      MVCC_INTVALUE_EX offset_x{};
      MVCC_INTVALUE_EX offset_y{};
      const bool width_read = GetIntValue("Width", width);
      const bool height_read = GetIntValue("Height", height);
      const bool offset_x_read = GetIntValue("OffsetX", offset_x);
      const bool offset_y_read = GetIntValue("OffsetY", offset_y);
      const bool readback_succeeded =
          width_read && height_read && offset_x_read && offset_y_read;
      geometry_readback_matches = readback_succeeded &&
                                  width.nCurValue == geometry->width &&
                                  height.nCurValue == geometry->height &&
                                  offset_x.nCurValue == geometry->offset_x &&
                                  offset_y.nCurValue == geometry->offset_y;
      if (!geometry_readback_matches)
      {
        XR_LOG_ERROR("HikCamera original image geometry restore readback mismatch");
      }
    }

    bool decimation_readback_matches = true;
    if (decimation != nullptr)
    {
      MVCC_ENUMVALUE horizontal{};
      MVCC_ENUMVALUE vertical{};
      const bool horizontal_read = GetEnumValue("DecimationHorizontal", horizontal);
      const bool vertical_read = GetEnumValue("DecimationVertical", vertical);
      const bool readback_succeeded = horizontal_read && vertical_read;
      decimation_readback_matches =
          readback_succeeded &&
          horizontal.nCurValue == decimation->decimation_horizontal &&
          vertical.nCurValue == decimation->decimation_vertical;
      if (!decimation_readback_matches)
      {
        XR_LOG_ERROR("HikCamera original decimation restore readback mismatch");
      }
    }

    const bool geometry_restored = original_geometry_state_.CompleteRestore(
        geometry_writes_succeeded, geometry_readback_matches, false);
    const bool decimation_restored = original_decimation_state_.CompleteRestore(
        decimation_writes_succeeded, decimation_readback_matches, false);
    return geometry_restored && decimation_restored;
  }

  /**
   * @brief 配置 180 度图像旋转。
   *
   * 旋转通过相机 `ReverseX` 和 `ReverseY` 节点完成。请求旋转但节点不可用时启动失败。
   */
  bool ConfigureRotation()
  {
    device_rotate_180_ = false;
    bool old_reverse_x = false;
    bool old_reverse_y = false;
    if (!GetBoolValue("ReverseX", old_reverse_x) ||
        !GetBoolValue("ReverseY", old_reverse_y))
    {
      XR_LOG_ERROR("HikCamera requires readable ReverseX/ReverseY geometry state");
      return false;
    }

    if (!runtime_.rotate_180)
    {
      frame_geometry_.flags =
          (old_reverse_x ? CameraTypes::FRAME_GEOMETRY_REVERSE_X : 0U) |
          (old_reverse_y ? CameraTypes::FRAME_GEOMETRY_REVERSE_Y : 0U);
      XR_LOG_INFO("HikCamera retained device rotation state: reverse_x=%d reverse_y=%d",
                  old_reverse_x ? 1 : 0, old_reverse_y ? 1 : 0);
      return true;
    }

    (void)original_rotation_state_.CaptureOnce(
        {.reverse_x = old_reverse_x, .reverse_y = old_reverse_y});
    if (SetBoolValue("ReverseX", true) && SetBoolValue("ReverseY", true))
    {
      bool applied_reverse_x = false;
      bool applied_reverse_y = false;
      if (GetBoolValue("ReverseX", applied_reverse_x) &&
          GetBoolValue("ReverseY", applied_reverse_y) && applied_reverse_x &&
          applied_reverse_y)
      {
        device_rotate_180_ = true;
        frame_geometry_.flags =
            CameraTypes::FRAME_GEOMETRY_REVERSE_X | CameraTypes::FRAME_GEOMETRY_REVERSE_Y;
        XR_LOG_PASS("HikCamera using camera ReverseX+ReverseY for 180-degree rotation");
        return true;
      }
    }

    const bool rollback_succeeded = RestoreDeviceRotation();
    XR_LOG_ERROR("HikCamera failed to enable camera ReverseX+ReverseY");
    if (!rollback_succeeded)
    {
      XR_LOG_ERROR("HikCamera failed to restore rotation after configuration failure");
    }
    return false;
  }

  /**
   * @brief 返回当前旋转方式名称，用于启动日志。
   */
  const char* RotationModeName() const
  {
    const bool reverse_x = CameraTypes::HasGeometryFlag(
        frame_geometry_, CameraTypes::FRAME_GEOMETRY_REVERSE_X);
    const bool reverse_y = CameraTypes::HasGeometryFlag(
        frame_geometry_, CameraTypes::FRAME_GEOMETRY_REVERSE_Y);
    if (reverse_x && reverse_y)
    {
      return "device_reverse_xy";
    }
    if (reverse_x)
    {
      return "device_reverse_x";
    }
    if (reverse_y)
    {
      return "device_reverse_y";
    }
    return "none";
  }

  /**
   * @brief 恢复启动前保存的 `ReverseX` 和 `ReverseY` 设置。
   */
  [[nodiscard]] bool RestoreDeviceRotation()
  {
    const auto* rotation = original_rotation_state_.Get();
    if (rotation == nullptr)
    {
      return true;
    }
    if (camera_handle_ == nullptr)
    {
      return false;
    }

    const bool reverse_x_written = SetBoolValue("ReverseX", rotation->reverse_x);
    const bool reverse_y_written = SetBoolValue("ReverseY", rotation->reverse_y);
    const bool writes_succeeded = reverse_x_written && reverse_y_written;
    bool applied_reverse_x = false;
    bool applied_reverse_y = false;
    const bool reverse_x_read = GetBoolValue("ReverseX", applied_reverse_x);
    const bool reverse_y_read = GetBoolValue("ReverseY", applied_reverse_y);
    const bool readback_succeeded = reverse_x_read && reverse_y_read;
    const bool readback_matches = readback_succeeded &&
                                  applied_reverse_x == rotation->reverse_x &&
                                  applied_reverse_y == rotation->reverse_y;
    if (!readback_matches)
    {
      XR_LOG_ERROR("HikCamera original rotation restore readback mismatch");
    }
    const bool restored = original_rotation_state_.CompleteRestore(
        writes_succeeded, readback_matches, false);
    if (restored)
    {
      device_rotate_180_ = false;
      frame_geometry_.flags =
          (rotation->reverse_x ? CameraTypes::FRAME_GEOMETRY_REVERSE_X : 0U) |
          (rotation->reverse_y ? CameraTypes::FRAME_GEOMETRY_REVERSE_Y : 0U);
    }
    return restored;
  }

  /**
   * @brief 枚举 USB 相机、打开设备并配置采集参数。
   */
  bool CaptureStart()
  {
    if (camera_handle_ != nullptr)
    {
      XR_LOG_ERROR("HikCamera refused to open a second SDK handle");
      return false;
    }

    MV_CC_DEVICE_INFO_LIST device_list{};
    auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0)
    {
      XR_LOG_ERROR("HikCamera no USB camera found: ret=%d count=%u", ret,
                   device_list.nDeviceNum);
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

    if (!ConfigureImageGeometry(ProfileId::WIDE))
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

    XR_LOG_PASS(
        "HikCamera configured: trigger=%s rotate_180=%d rotate_mode=%s "
        "gain=%.3f exposure=%.3f us timestamp_freq_hz=%llu",
        runtime_.external_trigger ? "external" : "freerun", runtime_.rotate_180 ? 1 : 0,
        RotationModeName(), runtime_.gain, runtime_.exposure_time,
        static_cast<unsigned long long>(device_timestamp_frequency_hz_));
    return true;
  }

  /**
   * @brief 开始 Hik SDK 取流。
   */
  bool StartGrabbing()
  {
    if (camera_handle_ == nullptr || stream_state_.Active())
    {
      XR_LOG_ERROR("HikCamera refused to start SDK stream: handle=%d active=%d",
                   camera_handle_ != nullptr ? 1 : 0, stream_state_.Active() ? 1 : 0);
      return false;
    }
    const auto ret = MV_CC_StartGrabbing(camera_handle_);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_StartGrabbing failed: %d", ret);
      return false;
    }
    stream_state_.MarkStarted();
    return true;
  }

  /**
   * @brief Stop the SDK stream once for the current successful start.
   */
  [[nodiscard]] bool StopGrabbing(const char* phase) noexcept
  {
    int ret = MV_OK;
    const bool stopped = stream_state_.Stop(
        [this, &ret]() noexcept
        {
          ret = MV_CC_StopGrabbing(camera_handle_);
          return ret == MV_OK;
        });
    if (!stopped)
    {
      XR_LOG_ERROR("HikCamera MV_CC_StopGrabbing failed during %s: %d", phase, ret);
    }
    return stopped;
  }

  /**
   * @brief 启动采集线程；创建失败时保持 SDK 取流停止。
   */
  bool StartCaptureThread()
  {
    if (capture_thread_.joinable())
    {
      XR_LOG_ERROR("HikCamera refused to replace a live capture thread");
      return false;
    }
    camera_state_.store(true, std::memory_order_release);
    try
    {
      capture_thread_ = std::thread(CaptureThreadMain, this);
      return true;
    }
    catch (const std::system_error& error)
    {
      camera_state_.store(false, std::memory_order_release);
      (void)StopGrabbing("capture thread start rollback");
      XR_LOG_ERROR("HikCamera failed to create capture thread: %s", error.what());
      return false;
    }
  }

  /**
   * @brief 停止取流、验证原始设置恢复并销毁 SDK handle。
   *
   * @param release_snapshots 成功销毁最终 handle 后是否提交原始状态恢复。
   */
  [[nodiscard]] CaptureStopResult CaptureStop(bool release_snapshots) noexcept
  {
    CaptureStopResult result{};
    result.thread_stopped = StopCaptureThread();
    if (camera_handle_ == nullptr)
    {
      if (stream_state_.Active())
      {
        XR_LOG_ERROR("HikCamera SDK stream is active without a camera handle");
        result.stream_stopped = false;
      }
      return result;
    }
    if (!result.thread_stopped)
    {
      result.stream_stopped = false;
      result.rotation_restored = false;
      result.geometry_restored = false;
      result.device_closed = false;
      result.handle_destroyed = false;
      return result;
    }

    result.stream_stopped = StopGrabbing("teardown");

    result.rotation_restored = RestoreDeviceRotation();
    result.geometry_restored = RestoreImageGeometry();

    const int close_result = MV_CC_CloseDevice(camera_handle_);
    result.device_closed = close_result == MV_OK;
    if (!result.device_closed)
    {
      XR_LOG_ERROR("HikCamera MV_CC_CloseDevice failed: %d", close_result);
    }
    else
    {
      stream_state_.MarkReleased();
    }

    const int destroy_result = MV_CC_DestroyHandle(camera_handle_);
    result.handle_destroyed = destroy_result == MV_OK;
    if (result.handle_destroyed)
    {
      camera_handle_ = nullptr;
      stream_state_.MarkReleased();
    }
    else
    {
      XR_LOG_ERROR("HikCamera MV_CC_DestroyHandle failed: %d", destroy_result);
    }

    if (release_snapshots && result.handle_destroyed)
    {
      (void)original_rotation_state_.CompleteRestore(result.rotation_restored,
                                                     result.rotation_restored, true);
      (void)original_geometry_state_.CompleteRestore(result.geometry_restored,
                                                     result.geometry_restored, true);
      (void)original_decimation_state_.CompleteRestore(result.geometry_restored,
                                                       result.geometry_restored, true);
    }
    return result;
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
    const auto ret =
        MV_CC_GetIntValueEx(camera_handle_, "DeviceTimestampIncrement", &value);
    if (ret == MV_OK && value.nCurValue > 0)
    {
      device_timestamp_frequency_hz_ = static_cast<uint64_t>(value.nCurValue);
      return;
    }

    device_timestamp_frequency_hz_ = microseconds_per_second;
    XR_LOG_WARN(
        "HikCamera DeviceTimestampIncrement unavailable: ret=%d value=%lld, "
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
  void LogFirstCommittedFrame(const MV_FRAME_OUT_INFO_EX& frame_info,
                              uint64_t timestamp_us)
  {
    const uint64_t dev_ts =
        CombineU32(frame_info.nDevTimeStampHigh, frame_info.nDevTimeStampLow);
    XR_LOG_INFO(
        "HikCamera first frame: frame=%u sensor_ts=%llu us dev_ts=%llu "
        "host_ts=%lld counter=%u trigger=%u lost=%u",
        frame_info.nFrameNum, static_cast<unsigned long long>(timestamp_us),
        static_cast<unsigned long long>(dev_ts),
        static_cast<long long>(frame_info.nHostTimeStamp), frame_info.nFrameCounter,
        frame_info.nTriggerIndex, frame_info.nLostPacket);
  }

  /**
   * @brief 采集循环：取图、检查尺寸、写时间戳并提交图像。
   */
  void CaptureLoop()
  {
    while (camera_state_.load())
    {
      ImageFrame* image = this->GetWritableImage();
      if (image == nullptr)
      {
        LibXR::Thread::Sleep(1);
        continue;
      }

      MV_FRAME_OUT_INFO_EX frame_info{};
      const auto ret =
          MV_CC_GetImageForBGR(camera_handle_, image->data.data(),
                               static_cast<unsigned int>(Base::image_bytes), &frame_info,
                               static_cast<int>(runtime_.grab_timeout_ms));
      if (ret != MV_OK)
      {
        failure_count_.fetch_add(1U, std::memory_order_relaxed);
        continue;
      }

      if (frame_info.nWidth != frame_geometry_.width ||
          frame_info.nHeight != frame_geometry_.height ||
          frame_info.nFrameLen != static_cast<unsigned int>(Base::image_bytes))
      {
        XR_LOG_ERROR("HikCamera frame geometry mismatch: %ux%u len=%u", frame_info.nWidth,
                     frame_info.nHeight, frame_info.nFrameLen);
        failure_count_.fetch_add(1U, std::memory_order_relaxed);
        continue;
      }

      uint64_t image_timestamp_us = 0;
      if (!ResolveImageTimestampUs(frame_info, image_timestamp_us))
      {
        failure_count_.fetch_add(1U, std::memory_order_relaxed);
        continue;
      }

      image->timestamp_us = image_timestamp_us;
      image->geometry = frame_geometry_;
      if (this->CommitImage())
      {
        const uint32_t previous =
            frames_committed_.fetch_add(1U, std::memory_order_relaxed);
        if (previous == 0U)
        {
          LogFirstCommittedFrame(frame_info, image_timestamp_us);
        }
      }
      else
      {
        failure_count_.fetch_add(1U, std::memory_order_relaxed);
      }
    }
  }

  /**
   * @brief `std::thread` 入口。
   */
  static void CaptureThreadMain(Self* self) { self->CaptureLoop(); }

 private:
  RuntimeParam runtime_{};                          ///< 当前运行参数快照。
  void* camera_handle_{nullptr};                    ///< Hik SDK 设备 handle。
  LibXR::Mutex control_mutex_{};                    ///< 串行设备控制和 handle 生命周期。
  HikCameraDetail::SdkStreamState stream_state_{};  ///< SDK 取流启停状态。
  std::atomic<bool> camera_state_{false};           ///< 采集线程运行标志。
  std::thread capture_thread_{};                    ///< 采集线程。
  bool device_rotate_180_{false};                   ///< 当前是否由相机完成 180 度旋转。
  HikCameraDetail::OriginalStateSnapshot<RotationState> original_rotation_state_{};
  HikCameraDetail::OriginalStateSnapshot<ImageGeometryState> original_geometry_state_{};
  HikCameraDetail::OriginalStateSnapshot<DecimationState> original_decimation_state_{};
  uint32_t applied_decimation_horizontal_{1};  ///< SDK 实际应用的横向下采样。
  uint32_t applied_decimation_vertical_{1};    ///< SDK 实际应用的纵向下采样。
  MVCC_INTVALUE_EX full_width_range_{};        ///< 相机支持的宽度范围。
  MVCC_INTVALUE_EX full_height_range_{};       ///< 相机支持的高度范围。
  FrameGeometry frame_geometry_{};             ///< 每帧按值发布的固定采样几何。
  std::array<CameraProfile, 2U> profiles_{};   ///< 生命周期内稳定的 WIDE/NARROW 档位。
  ProfileId active_profile_{ProfileId::WIDE};  ///< 当前成功生效的档位。
  uint64_t device_timestamp_frequency_hz_{microseconds_per_second};  ///< 设备时间戳频率。
  std::atomic<uint32_t> frames_committed_{0};                        ///< 已提交帧数。
  std::atomic<uint32_t> failure_count_{0};                           ///< 失败帧数。
};
