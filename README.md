# HikCamera 模块

## 模块描述

`HikCamera` 模块是基于 **Hikrobot MVS SDK** 的工业相机采集与发布组件。模块在启动时完成设备枚举（USB）、打开与参数下发，随后进入抓取线程进行取流：将 Bayer8 原始图像去马赛克为 `RGB8`，并通过 LibXR 的话题系统发布图像帧与相机信息。若采集中断，守护线程将自动执行“停止→复位→重开→重抓取”的恢复流程，确保采集链路的稳定运行。

## 依赖

- **硬件**
  - 兼容 Hikrobot MVS 的 **USB 工业相机**
  - 开发/运行主机（安装 MVS SDK）
  - USB 3.x 数据线与稳定供电
- **软件模块**
  - `app_framework`：LibXR 核心
  - `CameraBase`：相机驱动基类
  - `OpenCV`：Bayer→RGB 去马赛克与调试显示
  - `Hikrobot MVS SDK`：`MV_CC_*` 设备/取流/控制 API

## 构造参数

`HikCamera` 的构造函数：

```cpp
HikCamera(LibXR::HardwareContainer& hw,
          LibXR::ApplicationManager& app,
          CameraBase::CameraInfo info,
          RuntimeParam runtime);
```

参数说明：

- `hw`：硬件容器句柄；`HikCamera` 继承自 `CameraBase` 并在构造中完成资源注册。
- `app`：应用管理器；用于将模块注册进应用生命周期（`app.Register(*this)`）。
- `info`：相机基础信息；构造时强制设为 `RGB8` 编码，尺寸与步长在抓取线程中按帧更新。
- `runtime`：运行期参数：
  - `exposure_time`（μs）：曝光时间，会按设备支持范围裁剪后下发。
  - `gain`：增益，会按设备支持范围裁剪后下发。
  - `autocap`（bool）：
    - `true`：关闭触发，连续自由采集。
    - `false`：开启触发；`AcquisitionMode=Continuous`，`TriggerSource=Line0`，上升沿触发。

## 主要功能

- **设备枚举与打开**：枚举 `MV_USB_DEVICE`，创建句柄并打开设备；配置图像节点缓冲数量。
- **采集/触发配置**：根据 `autocap` 切换触发模式；下发自动白平衡（连续）、曝光与增益手动、目标帧率（249 fps，实际受限于链路/曝光）。
- **参数动态下发**：`SetExposure`/`SetGain`/`SetRuntimeParam` 会调用 `UpdateParameters()`，在设备最小/最大范围内裁剪后下发。
- **抓取线程（HikCapture）**：以 100 ms 超时从相机取缓冲；更新时间戳、尺寸、步长；依据像素格式映射表执行 Bayer→RGB；发布帧与信息话题。
- **守护线程（HikGuard）**：监听停止事件，在抓取线程退出后重建采集链路，并以较低优先级重启抓取线程。
- **停止与复位**：停止取流并下发 `DeviceReset`，随后关闭与销毁句柄。

## 设计原则与工作方式

1. **启动**：构造完成后依次 `CaptureStart()` → `StartGrabbing()`，成功即创建并启动抓取与守护线程。
2. **数据通路**：
   - 抓取线程调用 `MV_CC_GetImageBuffer` 获取帧（超时 100 ms）。
   - 写入时间戳；更新 `width/height/step`；将 `Bayer8` 转换为 `RGB8`。
   - 发布图像帧与相机信息到内部话题。
3. **故障恢复**：抓取线程失败退出后，守护线程执行停止与重启流程。
4. **关闭**：析构时同步结束线程、停止取流、复位并释放句柄。

## 如何使用

```bash
# 添加 HikCamera 模块实例
xrobot_add_mod.exe HikCamera --instance-id hik_camera
# 生成主应用程序入口
xrobot_gen_main.exe
```

然后在 `User/xrobot.yaml` 中配置 `HikCamera` 的构造参数与运行参数（曝光/增益/触发模式等）。也可在运行时通过控制接口动态调整。

## 主要方法

### 核心控制方法（内部线程循环相关）

- `static void CapThreadFun(HikCamera* self)`：抓取线程入口；循环取帧、时间戳/尺寸更新、Bayer→RGB 去马赛克，发布帧与信息；失败退出并通知守护线程。
- `static void GuardThreadFun(HikCamera* self)`：守护线程入口；等待抓取线程的停止事件，随后执行停止→重新打开→重新取流→重启抓取线程的恢复流程。

### 设备与参数方法

- `bool CaptureStart()`：枚举并打开设备，设置采集/触发/自动白平衡、曝光、增益、帧率等，并读取图像信息。
- `bool StartGrabbing()`：开始取流。
- `void CaptureStop()`：停止取流并 `DeviceReset`，关闭与销毁句柄。
- `void UpdateParameters()`：读取设备允许范围，裁剪并下发 `ExposureTime` 与 `Gain`。
- `void SetExposure(double exposure)` / `void SetGain(double gain)` / `void SetRuntimeParam(const RuntimeParam& p)`：对运行参数进行更新并调用 `UpdateParameters()` 下发。
