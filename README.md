# HikCamera

`HikCamera` 是 Hikrobot USB 相机采集模块。它继承 `CameraBase<Info>`，
从相机 SDK 取 BGR8 图像，并通过 `CameraBase` 的共享图像槽位发布给后续视觉模块。

模块实现集中在 `HikCamera.hpp`。仓库内的 `hikSDK/include` 与 `hikSDK/lib`
是构建所需的 Hikrobot SDK 头文件和动态库。

## 构建边界

模块日志会输出 `uint64_t` 传感器时间戳和 Hikrobot 时间戳。BSP 或 CI 的顶层
CMake 必须在 `add_subdirectory(libxr)` 前打开
`LIBXR_PRINT_INTEGER_ENABLE_64BIT`，模块 CMake 不修改 libxr print profile。

## 时间戳

发布到 `ImageFrame::timestamp_us` 的时间戳来自相机硬件，不使用主机到达时间。

- 原始硬件时间戳来自 SDK 帧信息里的 `nDevTimeStampHigh / nDevTimeStampLow`，代码中合成为 `dev_ts`。
- 当前 Hik USB 相机的 `dev_ts` 已经是单调递增的微秒设备时间戳，直接写入 `ImageFrame::timestamp_us`。
- 启动时读取 `DeviceTimestampIncrement` 只作为诊断日志输出，不参与换算；把它当作 `ns/tick`
  会让当前 USB 设备的时间戳乘法溢出，并把帧周期放大数个数量级。
- 如果某一帧没有 `dev_ts`，该帧会被丢弃；不会用 `nHostTimeStamp` 或 `Timebase` 冒充传感器时间。
- `nHostTimeStamp` 只在首帧日志中作为 SDK/主机侧对照信息输出，不参与同步。
- 图像写入 `CameraBase` 槽位后立即提交；图像队列和同步关系由 `CameraFrameSync` 管。

## 运行参数

默认参数面向外部触发采集：

- `external_trigger = true`
- `TriggerSource = Line0`
- `TriggerActivation = RisingEdge`
- `encoding = BGR8`

关闭外部触发时，模块会配置 `AcquisitionFrameRate`，由相机自由运行。

## 与 IMU 同步

本模块只发布图像，不发布伪 IMU，也不响应 `sensor_sync_cmd`。
实机链路中的 `camera_gyro / camera_accl / camera_quat` 由板端或 IMU 侧发布，
图像与 IMU 的同步由后级同步模块完成。

## 验证边界

没有 Hikrobot 相机和对应触发硬件时，只能做编译验证。实机验证需要确认：

- 相机能稳定输出非零 `dev_ts`。
- `dev_ts` 帧间差应与实际触发周期一致，例如 50fps 时约 20000us。
- 外部触发信号与板端 IMU 采样策略匹配。
