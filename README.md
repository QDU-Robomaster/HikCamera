# HikCamera

Hikrobot USB 相机采集模块，负责通过 `CameraBase` 图像槽位发布 BGR8 图像。
仓库内保留 `hikSDK/include` 和 `hikSDK/lib`，不额外生成空 `.cpp` 占位文件。

关键边界：

- `HikCamera<Info>` 继承 `CameraBase<Info>`。
- 相机只通过 `CameraBase` image lease 发布 BGR8 图像。
- 模板实现全部在 `HikCamera.hpp`，不保留空的 `.cpp` 占位文件。
- 默认 `external_trigger = true`，触发源为 `Line0` 上升沿。
- `image->timestamp_us` 使用 Hik SDK 帧信息里的 `nDevTimeStampHigh/Low`，这是相机传感器侧时间戳。
- 启动时读取 `DeviceTimestampIncrement`，按 `ns/tick` 将设备 tick 转换成微秒。
- 如果 `DeviceTimestampIncrement` 节点不可用，会按 USB3 Vision 常见的 1 ns/tick 降级并打印告警。
- 如果设备没有给出硬件时间戳，本模块会报错并丢弃该帧，不用主机到达时间冒充传感器时间。
- 首帧只打印简要元信息：相机时间戳、SDK host timestamp、帧计数、触发计数和丢包数。
- 不发布伪 IMU，也不软件响应 `sensor_sync_cmd`。
- 实机链路的 `camera_gyro/camera_accl/camera_quat` 和硬件触发响应由板端/IMU 侧提供。

没有 Hik 相机和对应硬件触发平台时，只做编译验证。
