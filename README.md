# HikCamera

HikCamera 从 Hikrobot USB 相机读取 BGR8 图像，并写入 `CameraBase<Info>`
提供的图像槽。

本模块使用仓库内的 `hikSDK/include` 和 `hikSDK/lib` 构建。如果系统存在
`/opt/MVS`，CMake 会优先使用系统安装的 Hikrobot SDK。

## 相机信息

模板参数 `Info` 必须描述相机实际输出图像：

- `encoding` 必须是 `CameraTypes::Encoding::BGR8`
- `step` 必须等于 `width * 3`
- `width` 和 `height` 必须能被相机 SDK 接受
- 使用下采样时，`Info` 写下采样后的图像尺寸和内参

模块启动时会按 `Info.width` 和 `Info.height` 配置相机输出尺寸。目标尺寸小于
相机可用最大尺寸时，模块会设置居中的 `OffsetX` 和 `OffsetY`。

## 时间戳

`ImageFrame::timestamp_us` 来自 Hikrobot 帧信息中的设备时间戳。

代码读取：

```text
nDevTimeStampHigh
nDevTimeStampLow
DeviceTimestampIncrement
```

然后把设备 tick 换算成微秒。

如果某一帧没有设备时间戳，该帧会被丢弃。`nHostTimeStamp` 只用于首帧日志，
不会写入 `ImageFrame::timestamp_us`。

## 运行参数

`RuntimeParam` 字段：

- `camera_name`：相机实例名
- `image_topic_name`：图像名称
- `imu_topic_name`：同步 IMU 名称
- `gain`：相机增益，最大值限制为 `16`
- `exposure_time`：曝光时间，单位微秒
- `external_trigger`：是否使用外触发
- `acquisition_frame_rate`：自由运行帧率
- `grab_timeout_ms`：SDK 等待一帧图像的超时时间
- `image_node_num`：SDK 取流缓存节点数
- `decimation_horizontal`：横向下采样倍率，`1` 表示不下采样
- `decimation_vertical`：纵向下采样倍率，`1` 表示不下采样
- `rotate_180`：是否使用相机 `ReverseX` 和 `ReverseY` 旋转图像

`external_trigger = true` 时使用：

```text
TriggerMode = On
TriggerSource = Line0
TriggerActivation = RisingEdge
```

`external_trigger = false` 时关闭触发，并配置 `AcquisitionFrameRate`。

## 图像旋转

`rotate_180 = true` 时，模块会设置相机的 `ReverseX` 和 `ReverseY`。

如果相机不支持这两个节点，启动失败。模块不会在主机侧旋转图像。

停止相机时，模块会恢复启动前的 `ReverseX` 和 `ReverseY`。

## 图像写入

采集线程流程：

1. 等待图像槽可用
2. 调用 `MV_CC_GetImageForBGR`
3. 检查图像宽高和字节数
4. 写入 `ImageFrame::timestamp_us`
5. 调用 `CommitImage()`

如果图像槽还没有注册，采集线程会等待并定期打印日志。

## 命令

继承自 `CameraBase` 的 RamFS 命令可用于调整曝光和增益：

```text
set_exposure <微秒>
set_gain <值>
```
