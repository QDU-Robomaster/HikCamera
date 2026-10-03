# HikCamera

Hikrobot USB 相机采集模块：读取 BGR8 图像并写入 CameraBase 图像槽 / Hikrobot USB camera capture Module that reads BGR8 images into the CameraBase image slots

## 1. 模块作用 / Purpose

HikCamera 从 Hikrobot USB 相机读取 BGR8 图像，写入 `CameraBase<FrameLayoutV>` 提供的图像槽。模块打开枚举到的第一台 USB 相机，在后台线程中取图并发布；初始化的任何一步失败时，构造抛出 `std::runtime_error`。

HikCamera reads BGR8 images from a Hikrobot USB camera and writes them into the image slots provided by `CameraBase<FrameLayoutV>`. The Module opens the first enumerated USB camera and captures and publishes in a background thread; when any initialization step fails, the constructor throws `std::runtime_error`.

## 2. 相机几何与档位 / Camera Geometry and Profiles

模板参数 `FrameLayoutV` 描述相机实际输出图像的固定存储布局：

- `encoding` 为 `CameraTypes::Encoding::BGR8`，`step` 等于 `width * 3`，二者在编译期检查。
- `width` 与 `height` 须为相机 SDK 可接受的值。
- 使用下采样时，`FrameLayoutV` 写下采样后的图像尺寸；`CameraCalibration` 始终保留原生传感器尺寸下的内参和畸变。

模块启动时按 `FrameLayoutV.width`、`FrameLayoutV.height` 和 WIDE 下采样倍率配置相机，并读回校验。WIDE 档以零偏移覆盖完整原生传感器：`width * wide_decimation_x` 与 `height * wide_decimation_y` 等于标定的原生尺寸，也等于下采样后 SDK 允许的最大宽高。默认产品配置使用 `720x540 / 2x2` 覆盖完整的 `1440x1080` 传感器，标定配置可使用 `1440x1080 / 1x1`。NARROW 档使用 `1x1` 下采样和同尺寸的居中 ROI，偏移按 SDK 步进向下对齐。启动前的宽高、偏移和下采样设置在关闭相机时恢复。下采样倍率和触发周期须大于零。

`Profiles()` 返回 WIDE 和 NARROW 两档，触发周期分别为 `wide_trigger_period_us` 与 `narrow_trigger_period_us`。请求当前档位时直接返回当前几何，采集已停止时返回 `STATE_ERR`。切换到另一档时，先停止采集线程并丢弃未提交的写槽，调用方在此之前先停止外部触发。同一目标配置最多尝试四次（首次加三次重试）：每次清空 SDK 缓存并确认停流，写入配置并读回校验，再启动采集。超过次数时记录失败步骤和目标参数，返回 `FAILED`，采集保持停止，`applied` 不变。

`rotate_180 = true` 时，模块设置相机的 `ReverseX` 与 `ReverseY` 并读回确认，逐帧 geometry 带 `REVERSE_X | REVERSE_Y` 标志，图像旋转由相机完成，停止相机时恢复启动前的 `ReverseX` 与 `ReverseY`。`rotate_180 = false` 时保留设备当前的 `ReverseX` / `ReverseY` 状态，并写入 geometry 标志。两种情况下这两个节点都须可读，否则启动失败。

The template parameter `FrameLayoutV` describes the fixed storage layout of the images the camera actually outputs:

- `encoding` is `CameraTypes::Encoding::BGR8` and `step` equals `width * 3`; both are checked at compile time.
- `width` and `height` are values the camera SDK accepts.
- When decimation is used, `FrameLayoutV` holds the decimated image size; `CameraCalibration` always keeps the intrinsics and distortion at the native sensor size.

At startup the Module configures the camera from `FrameLayoutV.width`, `FrameLayoutV.height` and the WIDE decimation factors, and verifies by reading back. The WIDE profile covers the whole native sensor with zero offset: `width * wide_decimation_x` and `height * wide_decimation_y` equal the calibrated native size and also equal the maximum width and height the SDK allows after decimation. The default product configuration uses `720x540 / 2x2` to cover the full `1440x1080` sensor, and a calibration configuration can use `1440x1080 / 1x1`. The NARROW profile uses `1x1` decimation and a centered ROI of the same size, with offsets aligned down to the SDK step. The width, offset and decimation settings from before startup are restored when the camera is closed. Decimation factors and trigger periods are greater than zero.

`Profiles()` returns the WIDE and NARROW profiles with the trigger periods `wide_trigger_period_us` and `narrow_trigger_period_us`. Requesting the current profile returns the current geometry directly, and `STATE_ERR` when capture is stopped. Switching to the other profile first stops the capture thread and discards the uncommitted write slot; the caller stops the external trigger before that. The same target configuration is tried at most four times (the first attempt plus three retries): each attempt clears the SDK buffers and confirms the stream is stopped, writes the configuration and verifies by reading back, and then restarts capture. When the attempts are exhausted, the failed step and the target parameters are logged, `FAILED` is returned, capture stays stopped and `applied` is unchanged.

With `rotate_180 = true`, the Module sets the camera `ReverseX` and `ReverseY` and confirms by reading back; the per-frame geometry carries the `REVERSE_X | REVERSE_Y` flags, the camera performs the rotation, and the `ReverseX` and `ReverseY` from before startup are restored when the camera stops. With `rotate_180 = false`, the current `ReverseX` / `ReverseY` state of the device is kept and written into the geometry flags. In both cases the two nodes are readable, otherwise startup fails.

## 3. 时间戳 / Timestamps

`ImageFrame::timestamp_us` 来自 Hikrobot 帧信息中的设备时间戳。代码读取：

```text
nDevTimeStampHigh
nDevTimeStampLow
DeviceTimestampIncrement
```

`DeviceTimestampIncrement` 作为设备 tick 频率（Hz），用于把设备 tick 换算成微秒；该节点不可用时按 1 MHz（tick 即微秒）处理并打印警告。没有设备时间戳的帧被丢弃。`nHostTimeStamp` 只用于日志。

`ImageFrame::timestamp_us` comes from the device timestamp in the Hikrobot frame information. The code reads the three fields listed in the code block above.

`DeviceTimestampIncrement` is taken as the device tick frequency (Hz) and converts the device ticks to microseconds; when the node is unavailable, 1 MHz (one tick per microsecond) is assumed and a warning is printed. A frame without a device timestamp is discarded. `nHostTimeStamp` is used for logging only.

## 4. 触发、曝光与恢复 / Trigger, Exposure and Restoration

`external_trigger = true` 时使用：

```text
TriggerMode = On
TriggerSource = Line0
TriggerActivation = RisingEdge
```

`external_trigger = false` 时关闭触发，模块自动开启 SDK 的 `AcquisitionFrameRateEnable` 并配置 `acquisition_frame_rate`，退出时恢复原帧率和使能状态。实际帧率受曝光、读出和传输能力约束。

初始化设置 `AcquisitionMode = Continuous`、连续自动白平衡，关闭 `GainAuto` 与 `ExposureAuto`，并设置手动曝光时间和增益。仅当 SDK 明确返回 `MV_E_SUPPORT` 时跳过关闭自动曝光这一步，超时、访问错误等使初始化失败。增益超过 16 时截断为 16。

`adc_bit_depth` 显式配置时，保存原值、设置并读回，失败则停止初始化。`gamma_enabled = true` 时先保存 Gamma 选择器，必要时切到 User，读取并保存原 User Gamma 后再写入新值；`gamma` 须有限且在 SDK 报告的 User Gamma 范围内，否则初始化报错。

退出或后续初始化失败时，尝试恢复已修改的 ADC、Gamma 值及选择器、旋转、几何和帧率，恢复错误打印到日志。

With `external_trigger = true` the camera uses the settings in the code block above.

With `external_trigger = false` the trigger is off, and the Module enables the SDK `AcquisitionFrameRateEnable` and configures `acquisition_frame_rate`; the original frame rate and enable state are restored on exit. The actual frame rate is bounded by the exposure, readout and transfer capability.

Initialization sets `AcquisitionMode = Continuous` and continuous auto white balance, turns off `GainAuto` and `ExposureAuto`, and sets the manual exposure time and gain. The step of turning off auto exposure is skipped only when the SDK explicitly returns `MV_E_SUPPORT`; timeouts, access errors and the like fail the initialization. A gain above 16 is clamped to 16.

When `adc_bit_depth` is configured explicitly, the original value is saved, the new value is set and read back, and a failure stops the initialization. With `gamma_enabled = true`, the Gamma selector is saved first, switched to User when necessary, and the original User Gamma is read and saved before the new value is written; `gamma` is finite and within the User Gamma range the SDK reports, otherwise initialization reports an error.

On exit or a later initialization failure, the Module tries to restore the modified ADC, Gamma value and selector, rotation, geometry and frame rate; restoration errors are printed to the log.

## 5. 图像写入与 RamFS 命令 / Image Writing and RamFS Commands

采集线程（`std::thread`）的流程：

1. 等待图像槽可用，无空槽时每 1 ms 重试。
2. 调用 `MV_CC_GetImageForBGR`，超时为 `grab_timeout_ms`。
3. 检查图像宽高和字节数与当前 geometry 一致。
4. 写入 `ImageFrame::timestamp_us` 和 `geometry`。
5. 调用 `CommitImage()`。

取图失败、尺寸不符或缺少设备时间戳都计入失败计数，该帧不提交。首个提交帧打印设备与主机时间戳、帧计数和丢包数。`OnMonitor()` 打印已提交帧数、失败帧数，以及单帧采集耗时统计（单位 us）。

继承自 `CameraBase` 的 RamFS 命令（文件名为 `camera_name`）用于调整曝光和增益：

```text
set_exposure <microseconds>
set_gain <value>
```

The capture thread (`std::thread`) proceeds as follows:

1. Wait for an available image slot, retrying every 1 ms when no slot is free.
2. Call `MV_CC_GetImageForBGR` with the timeout `grab_timeout_ms`.
3. Check that the image width, height and byte count match the current geometry.
4. Write `ImageFrame::timestamp_us` and `geometry`.
5. Call `CommitImage()`.

A failed grab, a size mismatch or a missing device timestamp counts as a failure and the frame is not committed. The first committed frame prints the device and host timestamps, the frame count and the lost-packet count. `OnMonitor()` prints the committed frame count, the failed frame count and the per-frame capture duration statistics in us.

The RamFS command inherited from `CameraBase` (file name `camera_name`) adjusts exposure and gain, with the commands in the code block above.

## 6. 构造接口 / Constructor

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class HikCamera : public CameraBase<FrameLayoutV>;

explicit HikCamera(
    LibXR::RamFS& ramfs,
    CameraCalibration calibration = DefaultCalibration(),
    RuntimeParam runtime = DefaultRuntime());
```

模板参数：

- `FrameLayoutV`：相机输出布局，紧密排列的 BGR8，WIDE 档覆盖完整原生传感器。

依赖：

- `ramfs`：`LibXR::RamFS`，注册相机命令文件，取自 BSP 的硬件注册（`XR_REGISTER`）。

配置参数：

- `calibration`：原生标定，默认 `DefaultCalibration()`，即 1440x1080、`fx ≈ fy ≈ 2328.7`、`PLUMB_BOB` 五项畸变的实机标定。
- `runtime`：`RuntimeParam`，字段如下表，默认值为 `DefaultRuntime()`。

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `camera_name` | `"camera"` | 相机实例名、RamFS 命令文件名，也是 CameraFrameSync 原始 IMU Topic 前缀。 |
| `image_topic_name` | `"camera_image"` | 图像 Topic 名。 |
| `imu_topic_name` | `"camera_imu"` | 同步 IMU Topic 名，交给 CameraBase。 |
| `gain` | `16.0` | 相机增益，最大 16。 |
| `exposure_time` | `2000.0` | 曝光时间，单位 us。 |
| `external_trigger` | `true` | 使用 Line0 上升沿外触发。 |
| `acquisition_frame_rate` | `249.0` | 非外触发时的自由运行帧率。 |
| `grab_timeout_ms` | `100` | SDK 等待一帧图像的超时时间，单位 ms。 |
| `image_node_num` | `3` | SDK 取流缓存节点数。 |
| `rotate_180` | `false` | 使用相机 `ReverseX` / `ReverseY` 旋转 180 度。 |
| `wide_decimation_x` | `2` | WIDE 档横向下采样倍率。 |
| `wide_decimation_y` | `2` | WIDE 档纵向下采样倍率。 |
| `wide_trigger_period_us` | `10000` | WIDE 档外触发周期，单位 us。 |
| `narrow_trigger_period_us` | `5000` | NARROW 档外触发周期，单位 us。 |
| `adc_bit_depth` | `std::nullopt` | 可选 `AdcBitDepth::BIT_8`、`BIT_10`、`BIT_11`、`BIT_12`；未指定时保留设备 ADC 位深。 |
| `gamma_enabled` | `false` | `true` 时设置 User Gamma，`false` 时保留 Gamma。 |
| `gamma` | `1.0` | User Gamma 请求值。 |

`RuntimeParam` 有三个构造函数：不含 WIDE / NARROW 字段的基本形式；在 `rotate_180` 之前带 `decimation_horizontal, decimation_vertical` 的形式；在 `rotate_180` 之后带 `wide_decimation_x, wide_decimation_y, wide_trigger_period_us, narrow_trigger_period_us` 的完整形式。三者末尾都是带默认值的 `adc_bit_depth, gamma_enabled, gamma`。

Template parameter:

- `FrameLayoutV`: the camera output layout, tightly packed BGR8; the WIDE profile covers the whole native sensor.

Dependencies:

- `ramfs`: the `LibXR::RamFS` that registers the camera command file, taken from the BSP's Registration (`XR_REGISTER`).

Configuration parameters:

- `calibration`: the native calibration, default `DefaultCalibration()`, a calibration measured on the real camera at 1440x1080 with `fx ≈ fy ≈ 2328.7` and five `PLUMB_BOB` distortion terms.
- `runtime`: `RuntimeParam` with the fields in the table below, default `DefaultRuntime()`.

| Field | Default | Meaning |
| --- | --- | --- |
| `camera_name` | `"camera"` | Camera instance name, the RamFS command file name, and the prefix of the raw IMU Topic of CameraFrameSync. |
| `image_topic_name` | `"camera_image"` | Image Topic name. |
| `imu_topic_name` | `"camera_imu"` | Synchronized IMU Topic name, passed to CameraBase. |
| `gain` | `16.0` | Camera gain, at most 16. |
| `exposure_time` | `2000.0` | Exposure time in us. |
| `external_trigger` | `true` | Use the Line0 rising edge as the external trigger. |
| `acquisition_frame_rate` | `249.0` | Free-running frame rate without external trigger. |
| `grab_timeout_ms` | `100` | Timeout in ms for the SDK to wait for one image. |
| `image_node_num` | `3` | Number of SDK stream buffer nodes. |
| `rotate_180` | `false` | Rotate by 180 degrees with the camera `ReverseX` / `ReverseY`. |
| `wide_decimation_x` | `2` | Horizontal decimation factor of the WIDE profile. |
| `wide_decimation_y` | `2` | Vertical decimation factor of the WIDE profile. |
| `wide_trigger_period_us` | `10000` | External trigger period of the WIDE profile in us. |
| `narrow_trigger_period_us` | `5000` | External trigger period of the NARROW profile in us. |
| `adc_bit_depth` | `std::nullopt` | Optional `AdcBitDepth::BIT_8`, `BIT_10`, `BIT_11` or `BIT_12`; the device ADC bit depth is kept when unspecified. |
| `gamma_enabled` | `false` | `true` sets User Gamma, `false` keeps Gamma. |
| `gamma` | `1.0` | Requested User Gamma value. |

`RuntimeParam` has three constructors: the basic form without the WIDE / NARROW fields; a form with `decimation_horizontal, decimation_vertical` before `rotate_180`; and the full form with `wide_decimation_x, wide_decimation_y, wide_trigger_period_us, narrow_trigger_period_us` after `rotate_180`. All three end with `adc_bit_depth, gamma_enabled, gamma`, which have defaults.

## 7. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `runtime.image_topic_name`（默认 `camera_image`） | 发布 | `CameraBase<FrameLayoutV>::ImageTopicPayload`（`const SharedFrame*`） | 每个提交的图像，载荷仅在同步回调期间有效 |
| `runtime.imu_topic_name`（默认 `camera_imu`） | 创建 | `CameraBase<FrameLayoutV>::ImuStamped` | CameraBase 创建的同步 IMU Topic，由 `PublishImu()` 发布 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `runtime.image_topic_name` (default `camera_image`) | Publish | `CameraBase<FrameLayoutV>::ImageTopicPayload` (`const SharedFrame*`) | Each committed image; the payload is valid only during the synchronous callback |
| `runtime.imu_topic_name` (default `camera_imu`) | Create | `CameraBase<FrameLayoutV>::ImuStamped` | Synchronized IMU Topic created by CameraBase and published by `PublishImu()` |

## 8. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/HikCamera --template-arg <FrameLayout>` 写入的实例，`template_args` 引用 YAML 中以 constexpr 定义的帧布局，`calibration` 与 `runtime` 为工具写入的 C++ 表达式 `DefaultCalibration()` 与 `DefaultRuntime()`，`ramfs` 填写为 BSP 中用 `XR_REGISTER`（硬件注册）注册的 RamFS 名称。默认标定为 1440x1080、默认 WIDE 下采样为 2x2，因此示例使用 720x540 布局。`runtime` 写成 YAML 映射时，键为所选构造函数的参数名（第 6 节表中的字段名，例如完整形式的 `camera_name` 与 `wide_trigger_period_us`），字符串写成带引号的 C++ 字符串字面量。`CameraFrameSync` 实例用 `camera: camera` 引用本实例，列在本实例之后。

An instance written by `xrobot instance add QDU-Robomaster/HikCamera --template-arg <FrameLayout>`; `template_args` refers to the frame layout defined as a constexpr in the YAML, `calibration` and `runtime` are the C++ expressions `DefaultCalibration()` and `DefaultRuntime()` written by the tool, and `ramfs` is set to a RamFS name registered in the BSP with `XR_REGISTER` (Registration). The default calibration is 1440x1080 and the default WIDE decimation is 2x2, so the example uses a 720x540 layout. When `runtime` is written as a YAML mapping, the keys are the parameter names of the chosen constructor (the field names in the table of section 6, for example `camera_name` and `wide_trigger_period_us` of the full form), and strings are written as quoted C++ string literals. A `CameraFrameSync` instance refers to this instance with `camera: camera` and is listed after it.

```yaml
constexpr_includes:
  - CameraBase.hpp
constexprs:
  HikFrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 720, .height = 540, .step = 2160, .encoding = CameraTypes::Encoding::BGR8}'
modules:
  - module: QDU-Robomaster/HikCamera
    id: camera
    template_args:
      - ProjectConstexpr::HikFrameLayout
    args:
      - ramfs: ramfs
      - calibration: HikCamera<ProjectConstexpr::HikFrameLayout>::DefaultCalibration()
      - runtime: HikCamera<ProjectConstexpr::HikFrameLayout>::DefaultRuntime()
```

## 9. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/CameraBase`：相机基类与共享图像槽。
- `xrobot-org/DurationStatistics`：`OnMonitor()` 中的采集耗时统计。
- Hikrobot MVS SDK：使用仓库内的 `hikSDK/include` 和 `hikSDK/lib`（amd64 与 arm64）构建；系统存在 `/opt/MVS`（含 `include/MvCameraControl.h` 和 `lib/64/libMvCameraControl.so`）时，CMake 优先使用系统安装的 SDK。链接 `MvCameraControl`、`MvUsb3vTL`、`FormatConversion`、`MediaProcess`、`MVRender`，支持 x86_64 和 aarch64 Linux。随仓库分发的 Hikrobot MVS SDK 文件适用其自身的许可条款，见 [NOTICE](NOTICE)。
- LibXR。

硬件：Hikrobot USB 相机，外触发模式下 Line0 接触发信号。

测试：在打开 `BUILD_TESTING` 的 BSP 构建中，模块加入 `hik_camera_profile_control_test`（切档重试与 SDK 停流状态）和 `hik_camera_runtime_param_test`（`RuntimeParam` 各构造形式与默认值，编译期检查），用 `ctest` 运行。

Dependencies:

- `QDU-Robomaster/CameraBase`: the camera base class and the shared image slots.
- `xrobot-org/DurationStatistics`: the capture duration statistics in `OnMonitor()`.
- Hikrobot MVS SDK: built with `hikSDK/include` and `hikSDK/lib` (amd64 and arm64) in the repository; when `/opt/MVS` exists (with `include/MvCameraControl.h` and `lib/64/libMvCameraControl.so`), CMake prefers the system-installed SDK. It links `MvCameraControl`, `MvUsb3vTL`, `FormatConversion`, `MediaProcess` and `MVRender`, and supports x86_64 and aarch64 Linux. The Hikrobot MVS SDK files distributed with the repository are covered by their own license terms, see [NOTICE](NOTICE).
- LibXR.

Hardware: a Hikrobot USB camera, with the trigger signal on Line0 in external trigger mode.

Tests: in a BSP build with `BUILD_TESTING` on, the Module adds `hik_camera_profile_control_test` (profile switch retries and the SDK stream state) and `hik_camera_runtime_param_test` (the constructor forms and defaults of `RuntimeParam`, checked at compile time), run with `ctest`.
