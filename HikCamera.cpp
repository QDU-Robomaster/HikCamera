#include "HikCamera.hpp"

#include <algorithm>  // std::clamp
#include <cstring>

HikCamera::HikCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                     CameraBase::CameraInfo info, RuntimeParam runtime)
    : CameraBase(hw, "hik_camera"), info_(info), runtime_(runtime)
{
  XR_LOG_INFO("Starting HikCamera!");

  // 编码保持为 RGB8（与清单一致），其余字段由流填充
  info_.encoding = CameraBase::Encoding::RGB8;

  MV_CC_DEVICE_INFO_LIST device_list{};
  // enum device
  auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  XR_LOG_INFO("Found camera count = %d", device_list.nDeviceNum);

  while (device_list.nDeviceNum == 0)
  {
    XR_LOG_ERROR("No camera found!");
    XR_LOG_INFO("Enum state: [%x]", ret);
    LibXR::Thread::Sleep(1000);
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }

  XR_LOG_PASS("Hik Camera found!");

  // 取第一台（可按需暴露选择逻辑）
  if (MV_OK != MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]))
  {
    XR_LOG_ERROR("MV_CC_CreateHandle failed");
    return;
  }
  if (MV_OK != MV_CC_OpenDevice(camera_handle_))
  {
    XR_LOG_ERROR("MV_CC_OpenDevice failed");
    return;
  }

  // 获取图像基础信息
  if (MV_OK != MV_CC_GetImageInfo(camera_handle_, &img_info_))
  {
    XR_LOG_ERROR("MV_CC_GetImageInfo failed");
    return;
  }

  frame_buf_ = std::make_unique<std::array<uint8_t, BUF_BYTES>>();

  // 初始化像素转换参数：源信息在抓帧时动态填写
  std::memset(&convert_param_, 0, sizeof(convert_param_));
  convert_param_.nWidth = img_info_.nWidthValue;
  convert_param_.nHeight = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
  convert_param_.pDstBuffer = frame_buf_->data();
  convert_param_.nDstBufferSize = BUF_BYTES;

  UpdateParameters();  // <<< 应用初始曝光/增益

  if (MV_OK != MV_CC_StartGrabbing(camera_handle_))
  {
    XR_LOG_ERROR("MV_CC_StartGrabbing failed");
    return;
  }

  running_.store(true);
  capture_thread_.Create(this, ThreadFun, "HikCameraThread",
                         static_cast<size_t>(1024 * 128),
                         LibXR::Thread::Priority::REALTIME);

  app.Register(*this);
}

HikCamera::~HikCamera()
{
  running_.store(false);
  // 视你的 LibXR::Thread 实现而定：此处假设线程在析构/停止时会 join，
  // 如果需要可补充 capture_thread_.Join();

  if (camera_handle_)
  {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
  }
  XR_LOG_INFO("HikCamera destroyed!");
}

void HikCamera::SetRuntimeParam(const RuntimeParam& p)
{
  runtime_ = p;
  UpdateParameters();
}

// --- CameraBase 接口实现 ---
// 注意：外部命令以单位传入（定义见你的接口）。这里延续“微秒/ExposureTime”的单位约定。
void HikCamera::SetExposure(double exposure)
{
  runtime_.exposure_time = static_cast<float>(exposure);
  UpdateParameters();
}

void HikCamera::SetGain(double gain)
{
  runtime_.gain = static_cast<float>(gain);
  UpdateParameters();
}

void HikCamera::UpdateParameters()
{
  if (!camera_handle_)
  {
    return;
  }

  // Exposure（微秒）
  {
    MVCC_FLOATVALUE f_value{};
    if (MV_OK == MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value))
    {
      runtime_.exposure_time =
          std::clamp(runtime_.exposure_time, f_value.fMin, f_value.fMax);
      MV_CC_SetFloatValue(camera_handle_, "ExposureTime", runtime_.exposure_time);
      XR_LOG_INFO("Exposure time: %f us", runtime_.exposure_time);
    }
    else
    {
      XR_LOG_WARN("Get ExposureTime range failed");
    }
  }

  // Gain
  {
    MVCC_FLOATVALUE f_value{};
    if (MV_OK == MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value))
    {
      runtime_.gain = std::clamp(runtime_.gain, f_value.fMin, f_value.fMax);
      MV_CC_SetFloatValue(camera_handle_, "Gain", runtime_.gain);
      XR_LOG_INFO("Gain: %f", runtime_.gain);
    }
    else
    {
      XR_LOG_WARN("Get Gain range failed");
    }
  }
}

void HikCamera::ThreadFun(HikCamera* self)
{
  MV_FRAME_OUT out_frame{};
  XR_LOG_INFO("Publishing image!");

  while (self->running_.load())
  {
    auto ret = MV_CC_GetImageBuffer(self->camera_handle_, &out_frame, 1000);

    if (MV_OK == ret)
    {
      const int W = out_frame.stFrameInfo.nWidth;
      const int H = out_frame.stFrameInfo.nHeight;

      self->info_.width = static_cast<uint32_t>(W);
      self->info_.height = static_cast<uint32_t>(H);
      self->info_.step = static_cast<uint32_t>(W * CH);
      self->info_.timestamp = LibXR::Timebase::GetMicroseconds();

      // Convert to RGB8
      self->convert_param_.pSrcData = out_frame.pBufAddr;
      self->convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
      self->convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
      MV_CC_ConvertPixelType(self->camera_handle_, &self->convert_param_);
      MV_CC_FreeImageBuffer(self->camera_handle_, &out_frame);

      cv::Mat img(H, W, CV_8UC3, self->frame_buf_->data(), static_cast<size_t>(W) * CH);

      self->frame_topic_.Publish(img);
      self->info_topic_.Publish(self->info_);

      self->fail_count_ = 0;
    }
    else
    {
      XR_LOG_WARN("Get buffer failed! nRet: [%x]", ret);
      MV_CC_StopGrabbing(self->camera_handle_);
      MV_CC_StartGrabbing(self->camera_handle_);
      if (++self->fail_count_ > 5)
      {
        XR_LOG_ERROR("Camera failed!");
      }
    }
  }
}
