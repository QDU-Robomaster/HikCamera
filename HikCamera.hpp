#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Hikrobot USB 相机采集模块：读取 BGR8 图像并写入 CameraBase 图像槽 / Hikrobot USB camera capture Module that reads BGR8 images into the CameraBase image slots
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
- id: xrobot-org/DurationStatistics
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>

#include "CameraBase.hpp"
#include "DurationStatistics.hpp"
#include "HikCameraProfileControl.hpp"
#include "MvCameraControl.h"
#include "libxr.hpp"
#include "logger.hpp"
#include "ramfs.hpp"
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
class HikCamera : public CameraBase<FrameLayoutV>
{
 public:
  /// 当前模板实例类型
  /// Current template instantiation type
  using Self = HikCamera<FrameLayoutV>;
  /// CameraBase 基类类型
  /// CameraBase base class type
  using Base = CameraBase<FrameLayoutV>;
  /// 图像槽载荷类型
  /// Image slot payload type
  using ImageFrame = typename Base::ImageFrame;
  /// 原生相机标定
  /// Native camera calibration
  using CameraCalibration = typename Base::CameraCalibration;
  /// 逐帧采样几何
  /// Per-frame sampling geometry
  using FrameGeometry = typename Base::FrameGeometry;
  /// 固定档位标识
  /// Fixed profile identifier
  using ProfileId = typename Base::ProfileId;
  /// 固定档位描述
  /// Fixed profile descriptor
  using CameraProfile = typename Base::CameraProfile;
  /// 已应用档位快照
  /// Applied profile snapshot
  using AppliedProfile = typename Base::AppliedProfile;

  /// 编译期帧存储布局。
  /// Compile-time frame storage layout.
  static inline constexpr auto frame_layout = Base::frame_layout;
  /// Hik SDK 当前取图路径固定输出 BGR 三通道。
  /// The Hik SDK grab path always outputs three BGR channels.
  static constexpr int channel_count = 3;
  /// 每行字节数。
  /// Bytes per row.
  static constexpr std::size_t frame_step = static_cast<std::size_t>(frame_layout.step);
  /// 一秒对应的微秒数。
  /// Microseconds per second.
  static constexpr uint64_t microseconds_per_second = 1000000ULL;
  /// 增益上限。
  /// Gain limit.
  static constexpr float max_gain = 16.0F;
  /// 产品宽视场档位的默认触发周期。
  /// Default trigger period of the product WIDE profile.
  static constexpr uint32_t default_wide_trigger_period_us = 10000U;
  /// 产品窄视场档位的默认触发周期。
  /// Default trigger period of the product NARROW profile.
  static constexpr uint32_t default_narrow_trigger_period_us = 5000U;
  /// 产品宽视场档位的默认横向下采样倍率。
  /// Default horizontal decimation factor of the product WIDE profile.
  static constexpr uint32_t default_wide_decimation_x = 2U;
  /// 产品宽视场档位的默认纵向下采样倍率。
  /// Default vertical decimation factor of the product WIDE profile.
  static constexpr uint32_t default_wide_decimation_y = 2U;

  /// 档位参数的默认值；实际值由 RuntimeParam 指定。
  /// Default values of the profile parameters; the actual values are given by
  /// RuntimeParam.
  static constexpr uint32_t wide_trigger_period_us = default_wide_trigger_period_us;
  static constexpr uint32_t narrow_trigger_period_us = default_narrow_trigger_period_us;
  static constexpr uint32_t wide_decimation_x = default_wide_decimation_x;
  static constexpr uint32_t wide_decimation_y = default_wide_decimation_y;

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
   * @brief 传感器 ADC 位深；SDK 数值在驱动内部转换。
   *        Sensor ADC bit depth; the SDK values are converted inside the driver.
   */
  enum class AdcBitDepth : uint32_t
  {
    BIT_8 = 8,    ///< 8 位 8 bits
    BIT_10 = 10,  ///< 10 位 10 bits
    BIT_11 = 11,  ///< 11 位 11 bits
    BIT_12 = 12,  ///< 12 位 12 bits
  };

  /**
   * @brief 运行时参数，字段值来自配置（YAML）。
   *        Runtime parameters; the field values come from the configuration (YAML).
   */
  struct RuntimeParam
  {
    /// CameraBase 相机名
    /// CameraBase camera name
    std::string_view camera_name = "camera";
    /// 图像 Topic 名
    /// Image Topic name
    std::string_view image_topic_name = "camera_image";
    /// 同步后的 IMU Topic 名
    /// Synchronized IMU Topic name
    std::string_view imu_topic_name = "camera_imu";
    /// 相机增益
    /// Camera gain
    float gain = 16.0F;
    /// 曝光时间，单位微秒
    /// Exposure time in us
    float exposure_time = 2000.0F;
    /// true 时使用 Line0 上升沿外触发
    /// Line0 rising-edge external trigger when true
    bool external_trigger = true;
    /// 非外触发模式下的自由运行帧率
    /// Free-running frame rate without external trigger
    float acquisition_frame_rate = 249.0F;
    /// SDK 等待一帧图像的超时时间
    /// Timeout in ms for the SDK to wait for one image
    uint32_t grab_timeout_ms = 100;
    /// SDK 内部取流缓存节点数
    /// Number of SDK internal stream buffer nodes
    uint32_t image_node_num = 3;
    /// true 时使用相机 ReverseX/Y 做 180 度旋转
    /// Rotate by 180 degrees with the camera ReverseX/Y when true
    bool rotate_180 = false;
    /// WIDE 档横向下采样倍率
    /// Horizontal decimation factor of the WIDE profile
    uint32_t wide_decimation_x = default_wide_decimation_x;
    /// WIDE 档纵向下采样倍率
    /// Vertical decimation factor of the WIDE profile
    uint32_t wide_decimation_y = default_wide_decimation_y;
    /// WIDE 档外触发周期，单位 us
    /// External trigger period of the WIDE profile in us
    uint32_t wide_trigger_period_us = default_wide_trigger_period_us;
    /// NARROW 档外触发周期，单位 us
    /// External trigger period of the NARROW profile in us
    uint32_t narrow_trigger_period_us = default_narrow_trigger_period_us;
    /// 未指定时保留设备 ADC 位深
    /// Device ADC bit depth kept when unspecified
    std::optional<AdcBitDepth> adc_bit_depth{};
    /// true 时设置 User Gamma，false 时不改 Gamma 节点
    /// User Gamma set when true, Gamma kept when false
    bool gamma_enabled = false;
    /// User Gamma 请求值，须有限且在设备支持范围内
    /// Requested User Gamma, finite and within the device range
    float gamma = 1.0F;

    /**
     * @brief 使用全部字段的默认值构造。
     *        Construct with the default values of all fields.
     */
    RuntimeParam() = default;

    /**
     * @brief 基本形式：不含 WIDE / NARROW 档位字段，这些字段取默认值。
     *        Basic form without the WIDE / NARROW profile fields, which take their
     *        default values.
     *
     * @param camera_name CameraBase 相机名。
     *                    CameraBase camera name.
     * @param image_topic_name 图像 Topic 名。
     *                         Image Topic name.
     * @param imu_topic_name 同步后的 IMU Topic 名。
     *                       Synchronized IMU Topic name.
     * @param gain 相机增益。
     *             Camera gain.
     * @param exposure_time 曝光时间，单位 us。
     *                      Exposure time in us.
     * @param external_trigger true 时使用 Line0 上升沿外触发。
     *                         Line0 rising-edge external trigger when true.
     * @param acquisition_frame_rate 非外触发模式下的自由运行帧率。
     *                               Free-running frame rate without external trigger.
     * @param grab_timeout_ms SDK 等待一帧图像的超时时间，单位 ms。
     *                        Timeout in ms for the SDK to wait for one image.
     * @param image_node_num SDK 内部取流缓存节点数。
     *                       Number of SDK internal stream buffer nodes.
     * @param rotate_180 true 时使用相机 ReverseX/Y 做 180 度旋转。
     *                   Rotate by 180 degrees with the camera ReverseX/Y when true.
     * @param adc_bit_depth ADC 位深，未指定时保留设备值。
     *                      ADC bit depth; the device value is kept when unspecified.
     * @param gamma_enabled true 时设置 User Gamma。
     *                      Set User Gamma when true.
     * @param gamma User Gamma 请求值。
     *              Requested User Gamma.
     */
    constexpr RuntimeParam(std::string_view camera_name,
                           std::string_view image_topic_name,
                           std::string_view imu_topic_name, float gain,
                           float exposure_time, bool external_trigger,
                           float acquisition_frame_rate, uint32_t grab_timeout_ms,
                           uint32_t image_node_num, bool rotate_180,
                           std::optional<AdcBitDepth> adc_bit_depth = std::nullopt,
                           bool gamma_enabled = false, float gamma = 1.0F)
        : camera_name(camera_name),
          image_topic_name(image_topic_name),
          imu_topic_name(imu_topic_name),
          gain(gain),
          exposure_time(exposure_time),
          external_trigger(external_trigger),
          acquisition_frame_rate(acquisition_frame_rate),
          grab_timeout_ms(grab_timeout_ms),
          image_node_num(image_node_num),
          rotate_180(rotate_180),
          adc_bit_depth(adc_bit_depth),
          gamma_enabled(gamma_enabled),
          gamma(gamma)
    {
    }

    /**
     * @brief 在 `rotate_180` 之前带 `decimation_horizontal`、`decimation_vertical`
     * 的形式， 二者设置 WIDE 档的下采样倍率。 Form with `decimation_horizontal` and
     * `decimation_vertical` before `rotate_180`, which set the WIDE profile decimation
     * factors.
     *
     * @param camera_name CameraBase 相机名。
     *                    CameraBase camera name.
     * @param image_topic_name 图像 Topic 名。
     *                         Image Topic name.
     * @param imu_topic_name 同步后的 IMU Topic 名。
     *                       Synchronized IMU Topic name.
     * @param gain 相机增益。
     *             Camera gain.
     * @param exposure_time 曝光时间，单位 us。
     *                      Exposure time in us.
     * @param external_trigger true 时使用 Line0 上升沿外触发。
     *                         Line0 rising-edge external trigger when true.
     * @param acquisition_frame_rate 非外触发模式下的自由运行帧率。
     *                               Free-running frame rate without external trigger.
     * @param grab_timeout_ms SDK 等待一帧图像的超时时间，单位 ms。
     *                        Timeout in ms for the SDK to wait for one image.
     * @param image_node_num SDK 内部取流缓存节点数。
     *                       Number of SDK internal stream buffer nodes.
     * @param decimation_horizontal WIDE 档横向下采样倍率。
     *                              Horizontal decimation factor of the WIDE profile.
     * @param decimation_vertical WIDE 档纵向下采样倍率。
     *                            Vertical decimation factor of the WIDE profile.
     * @param rotate_180 true 时使用相机 ReverseX/Y 做 180 度旋转。
     *                   Rotate by 180 degrees with the camera ReverseX/Y when true.
     * @param adc_bit_depth ADC 位深，未指定时保留设备值。
     *                      ADC bit depth; the device value is kept when unspecified.
     * @param gamma_enabled true 时设置 User Gamma。
     *                      Set User Gamma when true.
     * @param gamma User Gamma 请求值。
     *              Requested User Gamma.
     */
    constexpr RuntimeParam(std::string_view camera_name,
                           std::string_view image_topic_name,
                           std::string_view imu_topic_name, float gain,
                           float exposure_time, bool external_trigger,
                           float acquisition_frame_rate, uint32_t grab_timeout_ms,
                           uint32_t image_node_num, uint32_t decimation_horizontal,
                           uint32_t decimation_vertical, bool rotate_180,
                           std::optional<AdcBitDepth> adc_bit_depth = std::nullopt,
                           bool gamma_enabled = false, float gamma = 1.0F)
        : RuntimeParam(camera_name, image_topic_name, imu_topic_name, gain, exposure_time,
                       external_trigger, acquisition_frame_rate, grab_timeout_ms,
                       image_node_num, rotate_180, adc_bit_depth, gamma_enabled, gamma)
    {
      wide_decimation_x = decimation_horizontal;
      wide_decimation_y = decimation_vertical;
    }

    /**
     * @brief 完整形式：在 `rotate_180` 之后带 WIDE 档下采样倍率和两档的外触发周期。
     *        Full form with the WIDE decimation factors and the external trigger
     *        periods of both profiles after `rotate_180`.
     *
     * @param camera_name CameraBase 相机名。
     *                    CameraBase camera name.
     * @param image_topic_name 图像 Topic 名。
     *                         Image Topic name.
     * @param imu_topic_name 同步后的 IMU Topic 名。
     *                       Synchronized IMU Topic name.
     * @param gain 相机增益。
     *             Camera gain.
     * @param exposure_time 曝光时间，单位 us。
     *                      Exposure time in us.
     * @param external_trigger true 时使用 Line0 上升沿外触发。
     *                         Line0 rising-edge external trigger when true.
     * @param acquisition_frame_rate 非外触发模式下的自由运行帧率。
     *                               Free-running frame rate without external trigger.
     * @param grab_timeout_ms SDK 等待一帧图像的超时时间，单位 ms。
     *                        Timeout in ms for the SDK to wait for one image.
     * @param image_node_num SDK 内部取流缓存节点数。
     *                       Number of SDK internal stream buffer nodes.
     * @param rotate_180 true 时使用相机 ReverseX/Y 做 180 度旋转。
     *                   Rotate by 180 degrees with the camera ReverseX/Y when true.
     * @param wide_decimation_x WIDE 档横向下采样倍率。
     *                          Horizontal decimation factor of the WIDE profile.
     * @param wide_decimation_y WIDE 档纵向下采样倍率。
     *                          Vertical decimation factor of the WIDE profile.
     * @param wide_trigger_period_us WIDE 档外触发周期，单位 us。
     *                               External trigger period of the WIDE profile in us.
     * @param narrow_trigger_period_us NARROW 档外触发周期，单位 us。
     *                                 External trigger period of the NARROW profile in
     *                                 us.
     * @param adc_bit_depth ADC 位深，未指定时保留设备值。
     *                      ADC bit depth; the device value is kept when unspecified.
     * @param gamma_enabled true 时设置 User Gamma。
     *                      Set User Gamma when true.
     * @param gamma User Gamma 请求值。
     *              Requested User Gamma.
     */
    constexpr RuntimeParam(std::string_view camera_name,
                           std::string_view image_topic_name,
                           std::string_view imu_topic_name, float gain,
                           float exposure_time, bool external_trigger,
                           float acquisition_frame_rate, uint32_t grab_timeout_ms,
                           uint32_t image_node_num, bool rotate_180,
                           uint32_t wide_decimation_x, uint32_t wide_decimation_y,
                           uint32_t wide_trigger_period_us,
                           uint32_t narrow_trigger_period_us,
                           std::optional<AdcBitDepth> adc_bit_depth = std::nullopt,
                           bool gamma_enabled = false, float gamma = 1.0F)
        : RuntimeParam(camera_name, image_topic_name, imu_topic_name, gain, exposure_time,
                       external_trigger, acquisition_frame_rate, grab_timeout_ms,
                       image_node_num, rotate_180, adc_bit_depth, gamma_enabled, gamma)
    {
      this->wide_decimation_x = wide_decimation_x;
      this->wide_decimation_y = wide_decimation_y;
      this->wide_trigger_period_us = wide_trigger_period_us;
      this->narrow_trigger_period_us = narrow_trigger_period_us;
    }
  };

  /**
   * @brief 返回默认的原生标定：1440x1080，`PLUMB_BOB` 五项畸变的实机标定。
   *        Return the default native calibration: a calibration measured on the real
   *        camera at 1440x1080 with five `PLUMB_BOB` distortion terms.
   *
   * @return 默认标定。
   *         Default calibration.
   */
  static CameraCalibration DefaultCalibration() { return {.native_width = 1440, .native_height = 1080, .camera_matrix = {2328.685719898089, 0.0, 733.3564625092474, 0.0, 2328.670107789996, 540.6187286922773, 0.0, 0.0, 1.0}, .distortion_model = CameraTypes::DistortionModel::PLUMB_BOB, .distortion_coefficients = {-0.09182103918709904, 0.4639907346830205, 0.002609878642637282, 0.0009819586010405485, -0.4751278850310457}, .rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, .projection_matrix = {2328.685719898089, 0.0, 733.3564625092474, 0.0, 0.0, 2328.670107789996, 540.6187286922773, 0.0, 0.0, 0.0, 1.0, 0.0}}; }

  /**
   * @brief 返回默认的运行时参数。
   *        Return the default runtime parameters.
   *
   * @return 默认运行时参数。
   *         Default runtime parameters.
   */
  static RuntimeParam DefaultRuntime() { return {}; }

  /**
   * @brief 打开相机、配置参数并启动采集线程；配置或开始取流失败时抛出
   *        `std::runtime_error`。
   *        Open the camera, configure the parameters and start the capture thread;
   *        throw `std::runtime_error` when configuration or starting the stream fails.
   *
   * @param ramfs 注册相机命令文件的 RamFS。
   *              RamFS that registers the camera command file.
   * @param calibration 原生传感器坐标系下的相机标定。
   *                    Camera calibration in the native sensor coordinates.
   * @param runtime 运行时参数。
   *                Runtime parameters.
   */
  explicit HikCamera(
      LibXR::RamFS& ramfs,
      CameraCalibration calibration = DefaultCalibration(),
      RuntimeParam runtime = DefaultRuntime())
      : Base(ramfs, calibration, runtime.camera_name, runtime.image_topic_name,
             runtime.imu_topic_name),
        runtime_(runtime)
  {
    runtime_.gain = ClampGain(runtime_.gain);
    XR_LOG_INFO("Starting HikCamera: external_trigger=%d rotate_180=%d",
                runtime_.external_trigger ? 1 : 0, runtime_.rotate_180 ? 1 : 0);
    if (!(ValidateRuntimeProfileConfig() && CaptureStart() && InitializeProfiles() &&
          StartGrabbing() && StartCaptureThread()))
    {
      CaptureStop();
      throw std::runtime_error("HikCamera: failed to start camera");
    }
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

  /**
   * @brief 打印已提交帧数、失败帧数和单帧采集耗时统计。
   *        Print the committed frame count, the failed frame count and the per-frame
   *        capture duration statistics.
   */
  void OnMonitor()
  {
    const auto frame_capture = frame_capture_duration_.GetSummary();
    XR_LOG_INFO("HikCamera monitor: frames=%u failures=%u",
                frames_committed_.load(std::memory_order_relaxed),
                failure_count_.load(std::memory_order_relaxed));
    XR_LOG_INFO(
        "HikCamera capture count=%llu average_us=%llu minimum_us=%llu maximum_us=%llu",
        static_cast<unsigned long long>(frame_capture.sample_count),
        static_cast<unsigned long long>(frame_capture.average_us),
        static_cast<unsigned long long>(frame_capture.minimum_us),
        static_cast<unsigned long long>(frame_capture.maximum_us));
  }

  /**
   * @brief 设置曝光时间并下发到相机。
   *        Set the exposure time and send it to the camera.
   *
   * @param exposure 曝光时间，单位 us。
   *                 Exposure time in us.
   */
  void SetExposure(double exposure) override
  {
    runtime_.exposure_time = static_cast<float>(exposure);
    UpdateParameters();
  }

  /**
   * @brief 设置增益并下发到相机，超过 16 时截断为 16。
   *        Set the gain and send it to the camera; a value above 16 is clamped to 16.
   *
   * @param gain 增益。
   *             Gain.
   */
  void SetGain(double gain) override
  {
    runtime_.gain = ClampGain(static_cast<float>(gain));
    UpdateParameters();
  }

  /**
   * @brief 返回固定的 WIDE/NARROW 档位表。
   *        Return the fixed WIDE/NARROW profile table.
   *
   * @return 档位表。
   *         Profile table.
   */
  [[nodiscard]] std::span<const CameraProfile> Profiles() const noexcept override
  {
    return profiles_;
  }

  /**
   * @brief 停止 SDK 取流并阻塞切换到指定档位。
   *        Stop the SDK stream and switch to the given profile, blocking until done.
   *
   * 调用方先停止外部触发。同一目标最多尝试四次；每次确认 SDK 停流后再写入配置并读回
   * 校验。超过次数时记录错误，采集线程保持停止，`applied` 不变。
   *
   * The caller stops the external trigger first. The same target is tried at most four
   * times; each attempt confirms the SDK stream is stopped, then writes the
   * configuration and verifies it by reading back. When the attempts are exhausted the
   * error is logged, the capture thread stays stopped and `applied` is unchanged.
   *
   * @param id 目标档位。
   *           Target profile.
   * @param applied 输出：切换成功后实际生效的档位与几何。
   *                Output: the profile and geometry in effect after a successful switch.
   * @return `OK` 表示成功；`NOT_SUPPORT` 表示档位不存在；`STATE_ERR` 表示请求当前档位但
   *         采集已停止；`FAILED` 表示超过尝试次数。
   *         `OK` on success; `NOT_SUPPORT` when the profile does not exist; `STATE_ERR`
   *         when the current profile is requested while capture is stopped; `FAILED` when
   *         the attempts are exhausted.
   */
  LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) override
  {
    const CameraProfile* requested = FindProfile(id);
    if (requested == nullptr)
    {
      return LibXR::ErrorCode::NOT_SUPPORT;
    }
    if (id == active_profile_)
    {
      if (!camera_state_.load(std::memory_order_acquire))
      {
        return LibXR::ErrorCode::STATE_ERR;
      }
      applied = {.id = id, .geometry = frame_geometry_};
      return LibXR::ErrorCode::OK;
    }

    camera_state_.store(false, std::memory_order_release);
    if (capture_thread_created_ && capture_thread_.joinable())
    {
      capture_thread_.join();
      capture_thread_created_ = false;
    }
    this->DiscardWritableImage();

    const bool success = HikCameraDetail::RunProfileSwitchWithRetry(
        [this, id, requested](uint32_t attempt)
        {
          const auto failed = [id, requested, attempt](const char* stage)
          {
            XR_LOG_ERROR(
                "HikCamera switch profile=%u attempt=%u/4 stage=%s failed "
                "target=%ux%u decimation=%ux%u period_us=%u",
                static_cast<unsigned>(id), static_cast<unsigned>(attempt), stage,
                requested->geometry.width, requested->geometry.height,
                static_cast<unsigned>(requested->geometry.decimation_x),
                static_cast<unsigned>(requested->geometry.decimation_y),
                requested->trigger_period_us);
            return false;
          };
          if (camera_handle_ == nullptr)
          {
            return failed("handle");
          }
          const int clear_result = MV_CC_ClearImageBuffer(camera_handle_);
          const bool stopped = StopGrabbing();
          if (clear_result != MV_OK || !stopped)
          {
            XR_LOG_ERROR("HikCamera prepare switch: clear=%d stopped=%d", clear_result,
                         stopped ? 1 : 0);
            return failed("stop/clear");
          }
          if (!ConfigureImageGeometry(id))
          {
            return failed("configure/readback");
          }
          if (!CameraTypes::SameFrameGeometry(frame_geometry_, requested->geometry))
          {
            XR_LOG_ERROR("HikCamera switch geometry mismatch: actual=%ux%u offset=%u,%u",
                         frame_geometry_.width, frame_geometry_.height,
                         frame_geometry_.roi_offset_x_native,
                         frame_geometry_.roi_offset_y_native);
            return failed("profile geometry");
          }
          if (!StartGrabbing())
          {
            return failed("start SDK");
          }
          if (!StartCaptureThread())
          {
            return failed("start capture thread");
          }
          return true;
        });
    if (!success)
    {
      static_cast<void>(StopGrabbing());
      XR_LOG_ERROR(
          "HikCamera profile=%u failed after 4 attempts; capture stopped, "
          "developer intervention required",
          static_cast<unsigned>(id));
      return LibXR::ErrorCode::FAILED;
    }

    active_profile_ = id;
    applied = {.id = id, .geometry = frame_geometry_};
    return LibXR::ErrorCode::OK;
  }

 private:
  /**
   * @brief 把增益限制在上限以内。
   *        Clamp the gain to the limit.
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
   * @brief 检查档位下采样倍率和触发周期是否大于零。
   *        Check that the profile decimation factors and trigger periods are greater
   *        than zero.
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

    FrameGeometry narrow = frame_geometry_;
    narrow.roi_offset_x_native = (calibration.native_width - frame_layout.width) / 2U;
    narrow.roi_offset_y_native = (calibration.native_height - frame_layout.height) / 2U;
    narrow.decimation_x = 1U;
    narrow.decimation_y = 1U;
    if (!CameraTypes::ValidateFrameGeometry(frame_layout, calibration, narrow))
    {
      XR_LOG_ERROR("HikCamera generated invalid NARROW profile geometry");
      return false;
    }

    profiles_[0] = {.id = ProfileId::WIDE,
                    .geometry = frame_geometry_,
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

  /**
   * @brief 写入 Hik SDK float 节点。
   *        Write a Hik SDK float node.
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
   *        Write a Hik SDK enum node.
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
   *        Write a Hik SDK enum node by string.
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
   *        Read a Hik SDK enum node.
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
   *        Read a Hik SDK bool node.
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
   *        Write a Hik SDK bool node.
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
   *        Read a Hik SDK integer node.
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
   *        Write a Hik SDK integer node.
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
   *        Configure the camera decimation factors.
   *
   * `1x1` 表示不下采样；请求其它倍率时，相机提供对应的 SDK 节点。
   * `1x1` means no decimation; for any other factor the camera provides the matching SDK
   * nodes.
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

    if (!decimation_state_saved_)
    {
      old_decimation_horizontal_ = old_horizontal.nCurValue;
      old_decimation_vertical_ = old_vertical.nCurValue;
      decimation_state_saved_ = true;
    }

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
   *        Configure the `FrameLayoutV` output size, ROI and decimation for a fixed
   *        profile.
   *
   * 配置前保存启动时的宽高、偏移和下采样设置，关闭相机时恢复。
   * The width, offset and decimation settings from startup are saved before
   * configuration and restored when the camera is closed.
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

    if (!geometry_state_saved_)
    {
      old_width_ = old_width.nCurValue;
      old_height_ = old_height.nCurValue;
      old_offset_x_ = old_offset_x.nCurValue;
      old_offset_y_ = old_offset_y.nCurValue;
      geometry_state_saved_ = true;
    }

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
            : AlignDown((full_width_range_.nMax - target_width) / 2, offset_x_range);
    const int64_t offset_y =
        profile == ProfileId::WIDE
            ? 0
            : AlignDown((full_height_range_.nMax - target_height) / 2, offset_y_range);
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
   *        Restore the image size, offset and decimation settings saved before startup.
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
   *        Configure the 180-degree image rotation.
   *
   * 旋转通过相机 `ReverseX` 和 `ReverseY` 节点完成；请求旋转而节点不可用时启动失败。
   * The rotation uses the camera `ReverseX` and `ReverseY` nodes; startup fails when
   * rotation is requested and a node is unavailable.
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

    old_reverse_x_ = old_reverse_x;
    old_reverse_y_ = old_reverse_y;
    reverse_state_saved_ = true;
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

    (void)SetBoolValue("ReverseX", old_reverse_x);
    (void)SetBoolValue("ReverseY", old_reverse_y);
    reverse_state_saved_ = false;
    XR_LOG_ERROR("HikCamera failed to enable camera ReverseX+ReverseY");
    return false;
  }

  /**
   * @brief 返回当前旋转方式名称，用于启动日志。
   *        Return the name of the current rotation mode for the startup log.
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
   *        Restore the `ReverseX` and `ReverseY` settings saved before startup.
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
   * @brief 显式设置 ADC 位深，保存原值并读回确认。
   *        Set the ADC bit depth explicitly, saving the original value and confirming
   *        by reading back.
   */
  bool ConfigureAdcBitDepth()
  {
    if (!runtime_.adc_bit_depth)
    {
      return true;
    }
    unsigned int requested = 0U;
    switch (*runtime_.adc_bit_depth)
    {
      case AdcBitDepth::BIT_8:
        requested = 0U;
        break;
      case AdcBitDepth::BIT_10:
        requested = 1U;
        break;
      case AdcBitDepth::BIT_11:
        requested = 2U;
        break;
      case AdcBitDepth::BIT_12:
        requested = 3U;
        break;
      default:
        XR_LOG_ERROR("HikCamera invalid ADC bit depth: %u",
                     static_cast<unsigned>(*runtime_.adc_bit_depth));
        return false;
    }
    MVCC_ENUMVALUE original{};
    if (!GetEnumValue("ADCBitDepth", original))
    {
      return false;
    }
    old_adc_bit_depth_ = original.nCurValue;
    if (!SetEnumValue("ADCBitDepth", requested))
    {
      return false;
    }
    MVCC_ENUMVALUE applied{};
    if (!GetEnumValue("ADCBitDepth", applied) || applied.nCurValue != requested)
    {
      XR_LOG_ERROR("HikCamera ADC readback mismatch: requested=%u applied=%u", requested,
                   applied.nCurValue);
      return false;
    }
    return true;
  }

  /**
   * @brief 恢复已保存的 ADC 设置，失败时记录错误日志。
   *        Restore the saved ADC setting and log an error on failure.
   */
  void RestoreAdcBitDepth()
  {
    if (old_adc_bit_depth_)
    {
      if (!SetEnumValue("ADCBitDepth", *old_adc_bit_depth_))
      {
        XR_LOG_ERROR("HikCamera failed to restore ADC bit depth");
      }
      old_adc_bit_depth_.reset();
    }
  }

  /**
   * @brief 配置 User Gamma；写入新值之前先保存原值。
   *        Configure User Gamma; the original value is saved before a new value is
   *        written.
   */
  bool ConfigureGamma()
  {
    if (!runtime_.gamma_enabled)
    {
      return true;
    }
    if (!std::isfinite(runtime_.gamma))
    {
      XR_LOG_ERROR("HikCamera requires finite User Gamma");
      return false;
    }
    MVCC_ENUMVALUE selector{};
    auto ret = MV_CC_GetGammaSelector(camera_handle_, &selector);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera failed to read GammaSelector: %d", ret);
      return false;
    }
    if (selector.nCurValue != MV_GAMMA_SELECTOR_USER)
    {
      old_gamma_selector_ = selector.nCurValue;
      ret = MV_CC_SetGammaSelector(camera_handle_, MV_GAMMA_SELECTOR_USER);
      if (ret != MV_OK)
      {
        XR_LOG_ERROR("HikCamera failed to select User Gamma: %d", ret);
        return false;
      }
    }
    MVCC_FLOATVALUE original{};
    ret = MV_CC_GetGamma(camera_handle_, &original);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera failed to save original User Gamma: %d", ret);
      return false;
    }
    if (runtime_.gamma < original.fMin || runtime_.gamma > original.fMax)
    {
      XR_LOG_ERROR("HikCamera Gamma %.3f outside device range [%.3f, %.3f]",
                   runtime_.gamma, original.fMin, original.fMax);
      return false;
    }
    old_gamma_value_ = original.fCurValue;
    ret = MV_CC_SetGamma(camera_handle_, runtime_.gamma);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera failed to set User Gamma: %d", ret);
      return false;
    }
    return true;
  }

  /**
   * @brief 先恢复 User Gamma 值，再恢复原选择器，两项各自尝试。
   *        Restore the User Gamma value first and then the original selector, each
   *        attempted separately.
   */
  void RestoreGamma()
  {
    if (old_gamma_value_)
    {
      const auto ret = MV_CC_SetGamma(camera_handle_, *old_gamma_value_);
      if (ret != MV_OK)
      {
        XR_LOG_ERROR("HikCamera failed to restore User Gamma: %d", ret);
      }
      old_gamma_value_.reset();
    }
    if (old_gamma_selector_)
    {
      const auto ret = MV_CC_SetGammaSelector(camera_handle_, *old_gamma_selector_);
      if (ret != MV_OK)
      {
        XR_LOG_ERROR("HikCamera failed to restore GammaSelector: %d", ret);
      }
      old_gamma_selector_.reset();
    }
  }

  /**
   * @brief 自由运行时开启已有的帧率控制，保存原开关和帧率。
   *        Enable the existing frame rate control for free-running mode, saving the
   *        original switch and frame rate.
   */
  bool ConfigureFrameRate()
  {
    MVCC_FLOATVALUE original{};
    if (!GetBoolValue("AcquisitionFrameRateEnable", old_frame_rate_enabled_))
    {
      XR_LOG_ERROR("HikCamera failed to save frame-rate enable state");
      return false;
    }
    const auto ret =
        MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &original);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera failed to save frame rate: %d", ret);
      return false;
    }
    old_frame_rate_ = original.fCurValue;
    frame_rate_state_saved_ = true;
    if (!SetBoolValue("AcquisitionFrameRateEnable", true) ||
        !SetFloatValue("AcquisitionFrameRate", runtime_.acquisition_frame_rate))
    {
      XR_LOG_ERROR("HikCamera failed to configure free-run frame rate");
      return false;
    }
    return true;
  }

  /**
   * @brief 先恢复帧率再恢复使能开关，使帧率节点在恢复时可写。
   *        Restore the frame rate first and then the enable switch, so that the frame
   *        rate node is writable during restoration.
   */
  void RestoreFrameRate()
  {
    if (!frame_rate_state_saved_)
    {
      return;
    }
    if (!SetFloatValue("AcquisitionFrameRate", old_frame_rate_))
    {
      XR_LOG_ERROR("HikCamera failed to restore frame rate");
    }
    if (!SetBoolValue("AcquisitionFrameRateEnable", old_frame_rate_enabled_))
    {
      XR_LOG_ERROR("HikCamera failed to restore frame-rate enable state");
    }
    frame_rate_state_saved_ = false;
  }

  /**
   * @brief 关闭自动曝光；仅当 SDK 明确返回不支持时跳过。
   *        Turn off auto exposure; skipped only when the SDK explicitly reports it as
   *        unsupported.
   */
  bool DisableAutoExposure()
  {
    const auto ret =
        MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    if (ret == static_cast<int>(MV_E_SUPPORT))
    {
      XR_LOG_WARN("HikCamera ExposureAuto is unsupported; applying manual exposure");
      return true;
    }
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera failed to disable ExposureAuto: %d", ret);
      return false;
    }
    return true;
  }

  /**
   * @brief 枚举 USB 相机、打开设备并配置采集参数。
   *        Enumerate the USB cameras, open the device and configure the capture
   *        parameters.
   */
  bool CaptureStart()
  {
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

    if (!ConfigureAdcBitDepth() || !ConfigureImageGeometry(ProfileId::WIDE))
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
    else if (!SetEnumValue("TriggerMode", 0) || !ConfigureFrameRate())
    {
      return false;
    }

    if (!SetEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS) ||
        !DisableAutoExposure() || !SetEnumValue("GainAuto", MV_GAIN_MODE_OFF) ||
        !SetFloatValue("ExposureTime", runtime_.exposure_time) ||
        !SetFloatValue("Gain", runtime_.gain))
    {
      return false;
    }

    if (!ConfigureGamma() || !ConfigureRotation())
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
   *        Start the Hik SDK stream.
   */
  bool StartGrabbing()
  {
    const auto ret = MV_CC_StartGrabbing(camera_handle_);
    if (ret != MV_OK)
    {
      XR_LOG_ERROR("HikCamera MV_CC_StartGrabbing failed: %d", ret);
      return false;
    }
    sdk_stream_.MarkStarted();
    return true;
  }

  /**
   * @brief 仅在成功启动的取流仍需对应停止时停止 SDK 取流。
   *        Stop the SDK stream only when a successfully started stream still needs a
   *        matching stop.
   */
  bool StopGrabbing()
  {
    return sdk_stream_.StopIfActive(
        [this]()
        {
          const int result = MV_CC_StopGrabbing(camera_handle_);
          if (result != MV_OK)
          {
            XR_LOG_ERROR("HikCamera MV_CC_StopGrabbing failed: %d", result);
            return false;
          }
          return true;
        });
  }

  /**
   * @brief 启动采集线程；创建失败时请求 SDK 停流，后续重试再次确认停流状态。
   *        Start the capture thread; when creation fails, request the SDK to stop the
   *        stream, and later retries confirm the stream state again.
   */
  bool StartCaptureThread()
  {
    camera_state_.store(true, std::memory_order_release);
    try
    {
      capture_thread_ = std::thread(CaptureThreadMain, this);
      capture_thread_created_ = true;
      return true;
    }
    catch (const std::system_error& error)
    {
      camera_state_.store(false, std::memory_order_release);
      static_cast<void>(StopGrabbing());
      XR_LOG_ERROR("HikCamera failed to create capture thread: %s", error.what());
      return false;
    }
  }

  /**
   * @brief 停止取流、恢复相机设置并销毁 SDK handle。
   *        Stop the stream, restore the camera settings and destroy the SDK handle.
   */
  void CaptureStop()
  {
    if (camera_handle_ == nullptr)
    {
      return;
    }
    (void)MV_CC_StopGrabbing(camera_handle_);
    RestoreGamma();
    RestoreAdcBitDepth();
    RestoreDeviceRotation();
    RestoreImageGeometry();
    RestoreFrameRate();
    (void)MV_CC_CloseDevice(camera_handle_);
    (void)MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
  }

  /**
   * @brief 下发当前曝光和增益设置。
   *        Send the current exposure and gain settings.
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
   *        Read the device timestamp frequency.
   *
   * 读取失败时按微秒 tick 处理，并打印警告。
   * When reading fails, one tick per microsecond is assumed and a warning is printed.
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
   *        Compute the image timestamp in us from the SDK frame information.
   *
   * 没有设备时间戳的帧被拒绝。
   * A frame without a device timestamp is rejected.
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
   *        Combine the high and low 32-bit timestamps provided by the Hik SDK.
   */
  static uint64_t CombineU32(uint32_t high, uint32_t low)
  {
    return (static_cast<uint64_t>(high) << 32U) | static_cast<uint64_t>(low);
  }

  /**
   * @brief 把设备 tick 换算成微秒。
   *        Convert device ticks to microseconds.
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
   *        Print the first frame timestamps and the SDK frame number information.
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
   *        Capture loop: grab an image, check the size, write the timestamp and commit
   *        the image.
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

      auto frame_capture_measurement = frame_capture_duration_.Measure();
      MV_FRAME_OUT_INFO_EX frame_info{};
      const auto ret =
          MV_CC_GetImageForBGR(camera_handle_, image->data.data(),
                               static_cast<unsigned int>(Base::image_bytes), &frame_info,
                               static_cast<int>(runtime_.grab_timeout_ms));
      if (ret != MV_OK)
      {
        failure_count_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }

      if (frame_info.nWidth != frame_geometry_.width ||
          frame_info.nHeight != frame_geometry_.height ||
          frame_info.nFrameLen != static_cast<unsigned int>(Base::image_bytes))
      {
        XR_LOG_ERROR("HikCamera frame geometry mismatch: %ux%u len=%u", frame_info.nWidth,
                     frame_info.nHeight, frame_info.nFrameLen);
        failure_count_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }

      uint64_t image_timestamp_us = 0;
      if (!ResolveImageTimestampUs(frame_info, image_timestamp_us))
      {
        failure_count_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }

      image->timestamp_us = image_timestamp_us;
      image->geometry = frame_geometry_;
      if (this->CommitImage())
      {
        if (frames_committed_.load(std::memory_order_relaxed) == 0)
        {
          LogFirstCommittedFrame(frame_info, image_timestamp_us);
        }
        frames_committed_.fetch_add(1, std::memory_order_relaxed);
      }
      else
      {
        failure_count_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  /**
   * @brief `std::thread` 入口。
   *        Entry point of the `std::thread`.
   */
  static void CaptureThreadMain(Self* self) { self->CaptureLoop(); }

 private:
  /// 当前运行参数快照
  /// Snapshot of the runtime parameters
  RuntimeParam runtime_{};
  /// 原 ADC 值及待恢复状态
  /// Original ADC value awaiting restoration
  std::optional<unsigned int> old_adc_bit_depth_{};
  /// 改动前的 Gamma 选择器
  /// Gamma selector before the change
  std::optional<unsigned int> old_gamma_selector_{};
  /// 写入新 User Gamma 前保存的原值
  /// Original User Gamma saved before writing the new value
  std::optional<float> old_gamma_value_{};
  /// 自由运行帧率及开关需要恢复
  /// Free-running frame rate and switch need restoration
  bool frame_rate_state_saved_{false};
  /// 原帧率控制开关
  /// Original frame rate control switch
  bool old_frame_rate_enabled_{false};
  /// 原自由运行帧率
  /// Original free-running frame rate
  float old_frame_rate_{};
  /// SDK 已启动但尚未成功停止
  /// SDK started and not yet stopped successfully
  HikCameraDetail::SdkStreamState sdk_stream_{};
  /// Hik SDK 设备 handle
  /// Hik SDK device handle
  void* camera_handle_{nullptr};
  /// 采集线程运行标志
  /// Capture thread running flag
  std::atomic<bool> camera_state_{false};
  /// 采集线程
  /// Capture thread
  std::thread capture_thread_{};
  /// 线程是否已创建
  /// Whether the thread has been created
  bool capture_thread_created_{false};
  /// 当前是否由相机完成 180 度旋转
  /// Whether the camera performs the 180-degree rotation
  bool device_rotate_180_{false};
  /// 是否保存过 ReverseX / ReverseY 原值
  /// Whether the original ReverseX / ReverseY were saved
  bool reverse_state_saved_{false};
  /// 是否保存过宽高和偏移原值
  /// Whether the original width, height and offsets were saved
  bool geometry_state_saved_{false};
  /// 是否保存过下采样原值
  /// Whether the original decimation was saved
  bool decimation_state_saved_{false};
  /// 启动前的 ReverseX
  /// ReverseX before startup
  bool old_reverse_x_{false};
  /// 启动前的 ReverseY
  /// ReverseY before startup
  bool old_reverse_y_{false};
  /// 启动前的横向下采样
  /// Horizontal decimation before startup
  uint32_t old_decimation_horizontal_{1};
  /// 启动前的纵向下采样
  /// Vertical decimation before startup
  uint32_t old_decimation_vertical_{1};
  /// SDK 实际应用的横向下采样
  /// Horizontal decimation applied by the SDK
  uint32_t applied_decimation_horizontal_{1};
  /// SDK 实际应用的纵向下采样
  /// Vertical decimation applied by the SDK
  uint32_t applied_decimation_vertical_{1};
  /// 启动前的宽度
  /// Width before startup
  int64_t old_width_{0};
  /// 启动前的高度
  /// Height before startup
  int64_t old_height_{0};
  /// 启动前的 X 偏移
  /// X offset before startup
  int64_t old_offset_x_{0};
  /// 启动前的 Y 偏移
  /// Y offset before startup
  int64_t old_offset_y_{0};
  /// 相机支持的宽度范围
  /// Width range supported by the camera
  MVCC_INTVALUE_EX full_width_range_{};
  /// 相机支持的高度范围
  /// Height range supported by the camera
  MVCC_INTVALUE_EX full_height_range_{};
  /// 每帧按值发布的固定采样几何
  /// Fixed sampling geometry published by value with every frame
  FrameGeometry frame_geometry_{};
  /// 生命周期内稳定的 WIDE/NARROW 档位
  /// WIDE/NARROW profiles that stay stable over the lifetime
  std::array<CameraProfile, 2U> profiles_{};
  /// 当前成功生效的档位
  /// Profile currently in effect
  ProfileId active_profile_{ProfileId::WIDE};
  /// 设备时间戳频率
  /// Device timestamp frequency
  uint64_t device_timestamp_frequency_hz_{microseconds_per_second};
  XRobot::DurationStatistics frame_capture_duration_{};
  /// 已提交帧数
  /// Number of committed frames
  std::atomic<uint32_t> frames_committed_{0};
  /// 失败帧数
  /// Number of failed frames
  std::atomic<uint32_t> failure_count_{0};
};
