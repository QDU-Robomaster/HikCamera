# HikCamera

HikCamera 从 Hikrobot USB 相机读取 BGR8 图像，并写入 `CameraBase<FrameLayout>`
提供的图像槽。

本模块使用仓库内的 `hikSDK/include` 和 `hikSDK/lib` 构建。如果系统存在
`/opt/MVS`，CMake 会优先使用系统安装的 Hikrobot SDK。

## 相机信息

模板参数 `FrameLayout` 描述相机实际输出图像的固定存储布局：

- `encoding` 必须是 `CameraTypes::Encoding::BGR8`
- `step` 必须等于 `width * 3`
- `width` 和 `height` 必须能被相机 SDK 接受
- 使用下采样时，`FrameLayout` 写下采样后的图像尺寸；`CameraCalibration`
  始终保留原生传感器尺寸下的内参和畸变

模块启动时会按 `FrameLayout.width`、`FrameLayout.height` 和 WIDE 下采样倍率
配置相机。默认产品配置使用 `720x540 / 2x2` 覆盖完整 `1440x1080` 传感器；
标定配置可显式使用 `1440x1080 / 1x1`。NARROW 档固定使用 `1x1` 居中 ROI，
启动时按相机 `OffsetX/OffsetY` 的实际步进对齐并以 SDK readback 生成档位描述。

切档失败时模块会先在当前 SDK handle 上恢复上一档；若恢复失败，则关闭并重开
设备后再次恢复上一档。只有恢复后的取流线程成功启动，才会把相机保留为可用状态。
`SwitchProfile()`、`set_exposure` 和 `set_gain` 共用同一个设备控制锁，SDK handle 的
停止、销毁和重建不会与参数写入并发；采集线程不获取该锁，切档通过停止并 join
采集线程建立几何切换边界。

第一次修改设备几何、下采样或旋转前会保存一份生命周期级原始快照。中途 reopen
不会覆盖或释放该快照；每次恢复都要通过 SDK readback。只有最终 handle 已确认销毁且
原始状态恢复验证成功，快照才会提交释放。关闭或销毁失败会记录 SDK 错误；旧 handle
未确认销毁时不会再次打开设备。

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
- `rotate_180`：是否使用相机 `ReverseX` 和 `ReverseY` 旋转图像
- `wide_decimation_x`：WIDE 档横向下采样倍率，默认 `2`
- `wide_decimation_y`：WIDE 档纵向下采样倍率，默认 `2`
- `wide_trigger_period_us`：WIDE 档外触发周期，默认 `10000 us`
- `narrow_trigger_period_us`：NARROW 档外触发周期，默认 `5000 us`

下采样倍率和触发周期必须大于零。标定配置应使用 `1x1` 和较低的真实外触发
频率；只降低预览帧率不能减少相机与同步链路负载。

运行参数仍兼容旧生成配置中位于 `rotate_180` 之前的
`decimation_horizontal/decimation_vertical` 两项；新配置统一使用上述 WIDE 字段。

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

这两个命令与档位切换使用同一设备控制锁，不会在 handle reopen 期间调用 SDK。
