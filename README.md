# HikCamera cleanup notes

这个目录是现有 `HikCamera` 模块的源码替换版本，不包含 `hikSDK/` vendor payload。
集成到原模块仓库时保留原有 `hikSDK/include` 和 `hikSDK/lib`。

关键边界：

- `HikCamera<Info>` 继承 `CameraBase<Info>`。
- 相机只通过 `CameraBase` image lease 发布 BGR8 图像。
- 默认 `external_trigger = true`，触发源为 `Line0` 上升沿。
- 不发布伪 IMU，也不软件响应 `sensor_sync_cmd`。
- 实机链路的 `camera_gyro/camera_accl/camera_quat` 和硬件触发响应由板端/IMU 侧提供。

没有 Hik 相机和对应硬件触发平台时，只做编译验证。
