# HikCamera

海康 USB 相机驱动：BayerRG8 原始图像直接写入 CameraBase 图像槽 / Hikrobot USB camera driver that writes raw BayerRG8 images into the CameraBase slots

## 1. 模块作用 / Purpose

HikCamera 是 CameraBase 的驱动，面向 MV-CS016-10UC（1440×1080）。它打开第一台 USB 相机，按 v4 数据采集时的设置配置相机，在 WIDE 档开始取流；取到的 BayerRG8 原始字节直接拷进图像槽，SDK 不做去马赛克。采集线程、图像池和图像 Topic 由 CameraBase 负责。

HikCamera is a CameraBase driver for the MV-CS016-10UC (1440×1080). It opens the first USB camera, configures it with the settings used for the v4 data capture and starts streaming in the WIDE view. The raw BayerRG8 bytes are copied straight into the image slot with no demosaicing in the SDK. CameraBase runs the capture thread, the image pool and the image Topic.

## 2. 相机设置 / Camera Settings

启动时写入：`AcquisitionMode = Continuous`、`PixelFormat = BayerRG8`、`ADCBitDepth`、关闭自动曝光与自动增益、`ExposureTime`、`Gain`，以及触发方式。外触发为 Line0 上升沿，每个边沿一帧；不用外触发时按 `free_run_fps` 自由运行，供台架调试。关闭时不恢复相机原设置，其他工具启动时自行设置。

At start-up the driver writes `AcquisitionMode = Continuous`, `PixelFormat = BayerRG8`, `ADCBitDepth`, auto exposure and auto gain off, `ExposureTime`, `Gain` and the trigger mode. External trigger is a Line0 rising edge, one frame per edge; without it the camera free-runs at `free_run_fps` for bench work. The previous camera settings are not restored on shutdown; other tools set their own.

`HikSettings` 与 YAML 一一对应：

| 字段 / Field | 含义 / Meaning |
| --- | --- |
| `exposure_us` | 曝光时间，微秒 / Exposure time in µs |
| `gain_db` | 增益，dB / Gain in dB |
| `adc_bits` | ADC 位深，8 或 12，必填 / ADC bit depth, 8 or 12, required |
| `external_trigger` | true 为 Line0 外触发 / true for the Line0 trigger |
| `free_run_fps` | 自由运行帧率，仅 `external_trigger = false` 时使用 / Free-run rate |

ADC 位深决定读出时间：WIDE 档下 8 位约 3.4 ms、12 位约 5.7 ms。曝光与读出可以重叠，最短帧周期约为读出时间与“曝光时间 + 0.1 ms”中的较大者，外触发周期不能小于它。

The ADC bit depth sets the readout time: in WIDE about 3.4 ms at 8 bits and 5.7 ms at 12 bits. Exposure and readout overlap, so the shortest frame period is about the larger of the readout time and the exposure plus 0.1 ms; the trigger period must not be shorter.

## 3. 几何 / Geometry

`ApplyView` 停止取流，依次写偏移清零、`DecimationHorizontal/Vertical`、`Width/Height = 640×512`、偏移，读回核对后重新开始取流；失败时重试 3 次。相机节点上的偏移以跳采后的像素计：WIDE 的原生起点 (80, 24) 写成 (40, 12)，NARROW 的偏移即原生偏移。

`ApplyView` stops the stream, zeroes the offsets, writes `DecimationHorizontal/Vertical`, `Width/Height = 640×512` and the offsets, reads them back and restarts the stream; it retries 3 times on failure. Offsets on the camera are in decimated pixels: the WIDE native origin (80, 24) is written as (40, 12), and NARROW offsets are the native offsets.

`ApplyOffset`（NARROW 内移窗）不停流，只写 `OffsetX/Y` 并读回核对；失败时写回原偏移并返回 `FAILED`。

`ApplyOffset` (a window move within NARROW) keeps the stream running, writes only `OffsetX/Y` and reads them back; on failure it writes the previous offsets back and returns `FAILED`.

在 PI-HAILO-13T 上实测（MVS 5.0.2，自由运行 100 fps）：`SwitchView` 调用约 28 ms，按设备时间戳计两档之间的空档约 27 ms。

Measured on PI-HAILO-13T (MVS 5.0.2, free-run 100 fps): `SwitchView` takes about 28 ms and the gap between the two views is about 27 ms in device time.

## 4. 时间戳与帧计数 / Timestamps and Frame Counter

`timestamp_us` 取设备时间戳（`nDevTimeStampHigh/Low`），按 `DeviceTimestampIncrement` 换算为微秒；该节点不可读时按 1 tick 等于 1 µs。设备时间戳在停止、开始取流后继续累加。

`timestamp_us` is the device timestamp (`nDevTimeStampHigh/Low`) converted to microseconds with `DeviceTimestampIncrement`; when that node cannot be read, one tick is taken as 1 µs. The device timestamp keeps counting across stream stops and starts.

`frame_counter` 取 SDK 的 `nFrameNum`，每次开始取流从 0 起，连续递增。CameraFrameSync 用它把帧对应到触发边沿；没有空图像槽时这一帧被取出后丢弃，计数随之跳号。

`frame_counter` is the SDK's `nFrameNum`, which restarts at 0 on every stream start and increases by one per frame. CameraFrameSync uses it to match frames to trigger edges; when no image slot is free the frame is grabbed and dropped, so the counter skips.

## 5. 配置示例 / Configuration Example

```yaml
modules:
  - module: QDU-Robomaster/HikCamera
    id: camera
    args:
      - calibration: AutoAimRunConfig::MainCameraCalibration
      - narrow: {u: 0.5, v: 0.5}
      - name: "gimbal"
      - settings:
          exposure_us: 2000.0F
          gain_db: 16.0F
          adc_bits: 8
          external_trigger: true
          free_run_fps: 0.0F
```

相机名 `gimbal` 决定图像 Topic `gimbal_image`。打开或配置相机失败即致命退出，并打印失败的节点与 SDK 错误码。

The camera name `gimbal` gives the image Topic `gimbal_image`. Failing to open or configure the camera is fatal; the log names the failing node and the SDK error code.

## 6. SDK 与运行环境 / SDK and Runtime

仓库自带 Hikrobot MVS 5.0.2（CameraControl 4.8.1.2）的头文件和 amd64、arm64 运行库，只含 USB 相机取流需要的文件，`hikSDK/SHA256SUMS` 列出各文件的哈希。系统装有官方 MVS（`/opt/MVS`）时 CMake 优先使用它：x86_64 取 `lib/64`，aarch64 取 `lib/aarch64`。

The repository bundles the headers and the amd64 and arm64 runtime of Hikrobot MVS 5.0.2 (CameraControl 4.8.1.2), limited to the files a USB camera stream needs; `hikSDK/SHA256SUMS` lists their hashes. When the official MVS is installed under `/opt/MVS`, CMake uses it instead: `lib/64` on x86_64 and `lib/aarch64` on aarch64.

程序只直接链接 `libMvCameraControl.so`，其余库由 SDK 运行时加载，所以运行时须让 SDK 找到库目录：

The program links only `libMvCameraControl.so`; the SDK loads the other libraries at run time, so it has to be told where they are:

```bash
sdk=<HikCamera>/hikSDK
lib=$sdk/lib/arm64
export MVCAM_SDK_PATH=$sdk MVCAM_COMMON_RUNENV=$lib MVCAM_SOFTWARE_LIBENV=$lib
export ALLUSERSPROFILE=$HOME/.cache/hikrobot-mvs
export LD_LIBRARY_PATH=$lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

`ALLUSERSPROFILE` 指向一个可写目录，SDK 在其中缓存相机描述文件。非 root 用户访问相机需要 vendor `2bdf` 的 udev 规则。`libMVRender.so` 依赖系统的 `libGL`、`libX11`；无显示器时 SDK 打印 `XOpenDisplay Fail`，不影响取流。

`ALLUSERSPROFILE` points to a writable directory where the SDK caches the camera description files. A non-root user needs a udev rule for vendor `2bdf`. `libMVRender.so` needs the system `libGL` and `libX11`; without a display the SDK prints `XOpenDisplay Fail`, which does not affect streaming.

SDK 文件的使用与再分发遵循 Hikrobot 的 MVS SDK 条款，见 `NOTICE` 与 `hikSDK/license/`。

Use and redistribution of the SDK files follow Hikrobot's MVS SDK terms; see `NOTICE` and `hikSDK/license/`.

## 7. 测试 / Tests

`tests/hik_camera_test.cpp` 检查几何到相机节点的换算、设备 tick 到微秒的换算和 ADC 枚举值，并链接一次完整的驱动以确认 SDK 符号齐全。实机行为（取流、切档、帧计数）需在接相机的机器上验证。

`tests/hik_camera_test.cpp` checks the geometry-to-node conversion, the tick-to-microsecond conversion and the ADC enum values, and links the whole driver once to prove the SDK symbols resolve. Live behaviour (streaming, view switches, frame counter) is verified on a machine with the camera.

## 8. 依赖 / Dependencies

CameraBase、LibXR、Hikrobot MVS SDK。

CameraBase, LibXR, Hikrobot MVS SDK.
