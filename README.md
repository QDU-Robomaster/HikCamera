# HikCamera

HikCamera 从 Hikrobot USB 相机读取 BGR8 图像，并写入 `CameraBase<FrameLayoutV>`
提供的图像槽。模块打开枚举到的第一台 USB 相机，在后台线程中取图并发布。

本模块使用仓库内的 `hikSDK/include` 和 `hikSDK/lib`（amd64 / arm64）构建。如果系统存在
`/opt/MVS`（含 `include/MvCameraControl.h` 和 `lib/64/libMvCameraControl.so`），CMake 优先
使用系统安装的 Hikrobot SDK。随仓库分发的 Hikrobot MVS SDK 文件不适用本仓库许可证，见
[NOTICE](NOTICE)。

## 相机信息

模板参数 `FrameLayoutV` 描述相机实际输出图像的固定存储布局：

- `encoding` 必须是 `CameraTypes::Encoding::BGR8`
- `step` 必须等于 `width * 3`（以上两项编译期检查）
- `width` 和 `height` 必须能被相机 SDK 接受
- 使用下采样时，`FrameLayoutV` 写下采样后的图像尺寸；`CameraCalibration`
  始终保留原生传感器尺寸下的内参和畸变

模块启动时按 `FrameLayoutV.width`、`FrameLayoutV.height` 和 WIDE 下采样倍率配置相机，并读回
校验。WIDE 档必须以零偏移覆盖完整原生传感器：`width * wide_decimation_x` 和
`height * wide_decimation_y` 必须等于标定的原生尺寸，且等于下采样后 SDK 允许的最大宽高。
默认产品配置使用 `720x540 / 2x2` 覆盖完整 `1440x1080` 传感器；标定配置可显式使用
`1440x1080 / 1x1`。NARROW 档固定使用 `1x1` 下采样、同尺寸的居中 ROI（偏移按 SDK 步进
向下对齐）。启动前的宽高、偏移和下采样设置会在关闭相机时恢复。

## 时间戳

`ImageFrame::timestamp_us` 来自 Hikrobot 帧信息中的设备时间戳。代码读取：

```text
nDevTimeStampHigh
nDevTimeStampLow
DeviceTimestampIncrement
```

把 `DeviceTimestampIncrement` 当作设备 tick 频率（Hz），将设备 tick 换算成微秒；该节点
不可用时按 1 MHz（tick 即微秒）处理并打印警告。

如果某一帧没有设备时间戳，该帧会被丢弃。`nHostTimeStamp` 只用于日志，
不会写入 `ImageFrame::timestamp_us`。

## 触发与曝光

`external_trigger = true` 时使用：

```text
TriggerMode = On
TriggerSource = Line0
TriggerActivation = RisingEdge
```

`external_trigger = false` 时关闭触发，模块自动开启 SDK 的
`AcquisitionFrameRateEnable` 并配置 `acquisition_frame_rate`，不需要额外开关。退出时恢复
原帧率和使能状态。实际帧率仍受曝光、读出和传输能力约束。

初始化设置 `AcquisitionMode = Continuous`、自动白平衡连续、关闭 `GainAuto`，关闭
`ExposureAuto` 并设置手动曝光时间和增益。仅 SDK 明确返回 `MV_E_SUPPORT` 时跳过关闭自动
曝光这一步；超时、访问错误等仍导致初始化失败。增益超过 16 时截断为 16。

`adc_bit_depth` 显式配置时保存原值、设置并读回，失败则停止初始化。ADC 不改变 BGR8 输出
合约，也不会额外改写相机原生 PixelFormat。

`gamma_enabled = true` 时先保存 Gamma 选择器，必要时切到 User，再读取并保存原 User Gamma；
未能保存原值时不会写入新值。`gamma` 必须有限且在 SDK 报告的 User Gamma 范围内，否则初始化
报错，不截断或替换请求值。

退出或后续初始化失败时尝试恢复已修改的 ADC、Gamma 值及选择器、旋转、几何和帧率；恢复
错误会打印日志，设备断连时不保证可以完成恢复。任何初始化步骤失败时构造抛出
`std::runtime_error`。

下采样倍率和触发周期必须大于零。标定配置应使用 `1x1` 和较低的真实外触发频率；只降低
预览帧率不能减少相机与同步链路负载。

## 档位切换

`Profiles()` 返回 WIDE 和 NARROW 两档，触发周期分别为 `wide_trigger_period_us` 和
`narrow_trigger_period_us`。请求当前档位直接返回当前几何（采集已停止时返回 `STATE_ERR`）。

切到另一档时先停止采集线程、丢弃未提交的写槽。调用方必须先停止外部触发。同一目标配置
最多尝试四次（首次加三次重试）；每次清空 SDK 缓存并确认停流后写入配置、读回校验，再启动
采集。超限会记录失败步骤和目标参数，返回 `FAILED`，采集保持停止，`applied` 不变；不会
自动切换到其他档位。`CameraFrameSync` 收到失败后不发送新的 START，等待开发者处理。

## 图像旋转

`rotate_180 = true` 时，模块设置相机的 `ReverseX` 和 `ReverseY` 并读回确认，逐帧 geometry
带 `REVERSE_X | REVERSE_Y` 标志。模块不会在主机侧旋转图像；停止相机时恢复启动前的
`ReverseX` 和 `ReverseY`。

`rotate_180 = false` 时保留设备当前的 `ReverseX` / `ReverseY` 状态，并把它写入 geometry
标志。两种情况下这两个节点都必须可读，否则启动失败。

## 图像写入

采集线程（`std::thread`）流程：

1. 等待图像槽可用（无空槽时每 1 ms 重试）
2. 调用 `MV_CC_GetImageForBGR`，超时为 `grab_timeout_ms`
3. 检查图像宽高和字节数与当前 geometry 一致
4. 写入 `ImageFrame::timestamp_us` 和 `geometry`
5. 调用 `CommitImage()`

取图失败、尺寸不符或缺少设备时间戳都计入失败计数，不提交该帧。首个提交帧会打印设备与
主机时间戳、帧计数和丢包数。

`OnMonitor()` 打印已提交帧数、失败帧数，以及单帧采集耗时统计（微秒）。

## RamFS 命令

继承自 `CameraBase` 的 RamFS 命令（文件名为 `camera_name`）可用于调整曝光和增益：

```text
set_exposure <微秒>
set_gain <值>
```

## 依赖

- `QDU-Robomaster/CameraBase`：相机基类与共享图像槽。
- `xrobot-org/DurationStatistics`：`OnMonitor()` 中的采集耗时统计。
- 外部：Hikrobot MVS SDK（仓库内置或 `/opt/MVS`），链接 `MvCameraControl`、`MvUsb3vTL`、
  `FormatConversion`、`MediaProcess`、`MVRender`；支持 x86_64 和 aarch64 Linux。

## 构造接口

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class HikCamera : public CameraBase<FrameLayoutV>;

explicit HikCamera(
    LibXR::RamFS& ramfs,
    CameraCalibration calibration = DefaultCalibration(),
    RuntimeParam runtime = DefaultRuntime());
```

模板参数：

- `FrameLayoutV`：相机输出布局，紧密排列 BGR8，WIDE 档须覆盖完整原生传感器。

依赖：

- `ramfs`：`LibXR::RamFS`，注册相机命令文件。

配置：

- `calibration`：原生标定。默认 `DefaultCalibration()` 为 1440x1080、
  `fx ≈ fy ≈ 2328.7`、`PLUMB_BOB` 五项畸变的实机标定。
- `runtime`（`RuntimeParam`）：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `camera_name` | `"camera"` | 相机实例名、RamFS 命令文件名，也是 CameraFrameSync 原始 IMU topic 前缀。 |
| `image_topic_name` | `"camera_image"` | 图像 topic 名。 |
| `imu_topic_name` | `"camera_imu"` | 同步 IMU topic 名，交给 CameraBase。 |
| `gain` | `16.0` | 相机增益，最大 16。 |
| `exposure_time` | `2000.0` | 曝光时间，单位微秒。 |
| `external_trigger` | `true` | 使用 Line0 上升沿外触发。 |
| `acquisition_frame_rate` | `249.0` | 非外触发时的自由运行帧率。 |
| `grab_timeout_ms` | `100` | SDK 等待一帧图像的超时时间，单位 ms。 |
| `image_node_num` | `3` | SDK 取流缓存节点数。 |
| `rotate_180` | `false` | 使用相机 `ReverseX` / `ReverseY` 旋转 180 度。 |
| `wide_decimation_x` | `2` | WIDE 档横向下采样倍率。 |
| `wide_decimation_y` | `2` | WIDE 档纵向下采样倍率。 |
| `wide_trigger_period_us` | `10000` | WIDE 档外触发周期，单位 us。 |
| `narrow_trigger_period_us` | `5000` | NARROW 档外触发周期，单位 us。 |
| `adc_bit_depth` | `std::nullopt` | 可选 `AdcBitDepth::BIT_8 / BIT_10 / BIT_11 / BIT_12`；未指定时不改设备 ADC。 |
| `gamma_enabled` | `false` | `true` 时设置 User Gamma；`false` 时不改 Gamma。 |
| `gamma` | `1.0` | User Gamma 请求值。 |

`RuntimeParam` 有三个构造函数：不含 WIDE/NARROW 字段的基本形式；在 `rotate_180` 之前带
`decimation_horizontal, decimation_vertical` 的形式；以及在 `rotate_180` 之后带
`wide_decimation_x, wide_decimation_y, wide_trigger_period_us, narrow_trigger_period_us`
的完整形式。三者末尾都是 `adc_bit_depth, gamma_enabled, gamma`（有默认值）。

## 使用

```sh
xrobot module add QDU-Robomaster/HikCamera
xrobot setup
xrobot instance add QDU-Robomaster/HikCamera
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `ramfs` 填为 BSP 中用 `XR_REGISTER` 注册的 RamFS 对象名。帧布局用 constexpr 定义，必须与
相机输出一致；默认标定为 1440x1080、默认 WIDE 下采样为 2x2，因此这里使用 720x540 布局：

```yaml
constexpr_includes:
  - CameraBase.hpp
constexprs:
  FrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 720, .height = 540, .step = 2160, .encoding = CameraTypes::Encoding::BGR8}'
modules:
  - module: QDU-Robomaster/HikCamera
    id: hikcamera_0
    template_args:
      - ProjectConstexpr::FrameLayout
    args:
      - ramfs: ramfs
      - calibration: HikCamera<ProjectConstexpr::FrameLayout>::DefaultCalibration()
      - runtime: HikCamera<ProjectConstexpr::FrameLayout>::DefaultRuntime()
```

实际相机应换成自己的标定（`CameraTypes::CameraCalibration` 类型的 constexpr）。`runtime`
写成 YAML map 时，键必须按顺序写出某个构造函数的参数名；字符串写成 C++ 字符串字面量，
枚举写成 C++ 表达式，例如完整形式：

```yaml
runtime:
  camera_name: '"gimbal"'
  image_topic_name: '"camera_image"'
  imu_topic_name: '"camera_imu"'
  gain: '16.0F'
  exposure_time: '2000.0F'
  external_trigger: true
  acquisition_frame_rate: '249.0F'
  grab_timeout_ms: 100
  image_node_num: 3
  rotate_180: false
  wide_decimation_x: 2
  wide_decimation_y: 2
  wide_trigger_period_us: 10000
  narrow_trigger_period_us: 5000
  adc_bit_depth: 'HikCamera<ProjectConstexpr::FrameLayout>::AdcBitDepth::BIT_8'
  gamma_enabled: false
  gamma: '1.0F'
```

BSP 侧：

```cpp
XR_REGISTER(ramfs, LibXR::RamFS);
```

后续的 CameraFrameSync 实例用 `camera: hikcamera_0` 引用本实例，必须列在本实例之后。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/HikCamera`
（在 BSP 中）打印当前的构造函数。

## 测试

在打开 `BUILD_TESTING` 的 BSP 构建中，本模块加入 `hik_camera_profile_control_test`
（切档重试与 SDK 停流状态）和 `hik_camera_runtime_param_test`（`RuntimeParam` 各构造形式与
默认值，编译期检查），用 `ctest` 运行。
