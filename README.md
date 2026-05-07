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
- 当前 Hik USB 相机上 `DeviceTimestampIncrement` 表现为设备时间戳频率，例如
  `100000000` 表示 `100MHz` tick。
- 代码按 `dev_ts * 1000000 / DeviceTimestampIncrement` 换算到微秒，并用商/余数
  分段计算避免 64 位乘法溢出。
- 不能把 `DeviceTimestampIncrement` 当作 `ns/tick`；也不能直接把 100MHz `dev_ts`
  当作微秒，否则帧周期会放大 100 倍。
- 如果某一帧没有 `dev_ts`，该帧会被丢弃；不会用 `nHostTimeStamp` 或 `Timebase` 冒充传感器时间。
- `nHostTimeStamp` 只在首帧日志中作为 SDK/主机侧对照信息输出，不参与同步。
- 图像写入 `CameraBase` 槽位后立即提交；图像队列和同步关系由 `CameraFrameSync` 管。
- 需要内录时，通过 `runtime.recording` 打开 `CameraBase` 生产者侧原始图像记录。

## 运行参数

默认参数面向外部触发采集：

- `external_trigger = true`
- `TriggerSource = Line0`
- `TriggerActivation = RisingEdge`
- `ExposureAuto = Off`
- `GainAuto = Off`
- `BalanceWhiteAuto = Continuous`
- `exposure_time = 2000us`
- `gain = 16`
- `encoding = BGR8`

`gain` 会被限制到 `16`，运行时调用 `SetGain()` 传入更大值时会打印警告并按
`16` 下发给 SDK。

关闭外部触发时，模块会配置 `AcquisitionFrameRate`，由相机自由运行。

`rotate_180 = true` 用于修正倒装相机方向。模块会在开始取流前优先设置相机
`ReverseX` 和 `ReverseY` 两个 boolean 节点；两者都成功时，旋转由相机侧完成。
如果相机型号或当前配置不支持这两个节点，模块会恢复原节点值并退回到原有的
host buffer 180 度旋转路径，保持输出方向语义不变。启动日志里的
`rotate_mode` 会显示 `device_reverse_xy`、`host_buffer` 或 `none`。使用相机端
旋转时，模块停止相机前会把 `ReverseX / ReverseY` 恢复到启动前读到的值。

## 与 IMU 同步

本模块只发布图像，不发布伪 IMU，也不响应 `sensor_sync_cmd`。
实机链路中的 `camera_gyro / camera_accl / camera_quat` 由板端或 IMU 侧发布，
图像与 IMU 的同步由后级同步模块完成。

## 验证边界

没有 Hikrobot 相机和对应触发硬件时，只能做编译验证。实机验证需要确认：

- 相机能稳定输出非零 `dev_ts`。
- `ImageFrame::timestamp_us` 帧间差应与实际触发周期一致，例如 50fps 时约 20000us。
- 外部触发信号与板端 IMU 采样策略匹配。
