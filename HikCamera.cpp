#include "HikCamera.hpp"

HikCamera::HikCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                     Config cfg)
    : CameraBase(hw, "hik_camera"), cfg_(cfg), hik_state_(Hik::STOPPED)
{
  this->CaptureInit();
  this->ProtectRunning();

  this->image_topic_ = LibXR::Topic("image_topic", sizeof(HikCamera::ImageData));
  this->hik_sdk_thread_ = std::thread(
      [&]() -> void
      {
        while (true)
        {
          if (this->hik_state_ == Hik::STOPPED)
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }

          ImageData data;

          bool ok = this->Read(data);

          if (data.image.empty() || !ok)
          {
            continue;
          }

          this->image_topic_.Publish(data);
        }
      });
  app.Register(*this);
}

HikCamera::~HikCamera()
{
  if (this->guard_.protectthread.joinable())
  {
    this->guard_.protectthread.join();
  }
  this->CaptureStop();
  XR_LOG_INFO("HikRobot destructed.");
}

bool HikCamera::Read(ImageData& imgdata)
{
  while (this->hik_state_ == Hik::STOPPED)
  {
    return false;
  }

  MV_FRAME_OUT raw;
  unsigned int ret = 0;
  unsigned int n_msec = 100;

  auto start = LibXR::Timebase::GetMicroseconds();
  ret = MV_CC_GetImageBuffer(handle_, &raw, n_msec);

  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_GetImageBuffer failed: {:#x}", ret);
    this->hik_state_ = Hik::STOPPED;
    this->guard_.HikIsquit.notify_all();
    return false;
  }

  auto timestamp = LibXR::Timebase::GetMicroseconds();
  if ((LibXR::Timebase::GetMicroseconds() - start).ToMicrosecond() < 2000)
  {
    return false;
  }

  cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U,
              raw.pBufAddr);

  const auto& frame_info = raw.stFrameInfo;
  auto pixel_type = frame_info.enPixelType;
  cv::Mat dst_image;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> TYPE_MAP = {
      {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
      {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
      {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
      {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
  cv::cvtColor(img, dst_image, TYPE_MAP.at(pixel_type));
  img = dst_image;

  ret = MV_CC_FreeImageBuffer(handle_, &raw);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_FreeImageBuffer failed: {:#x}", ret);
    this->hik_state_ = Hik::STOPPED;
    this->guard_.HikIsquit.notify_all();
  }

  imgdata = {img, timestamp};
  return true;
}

void HikCamera::CaptureInit()
{
  unsigned int ret = 0;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0)
  {
    XR_LOG_ERROR("Not found camera!");
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  unsigned int n_image_node_num = 3;
  ret = MV_CC_SetImageNodeNum(handle_, n_image_node_num);
  if (MV_OK != ret)
  {
    // 设置失败
    XR_LOG_ERROR("MV_CC_SetImageNodeNum failed: {:#x}", ret);
    return;
  }

  if (!this->cfg_.autocap)
  {
    ret = MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous");
    if (MV_OK != ret)
    {
      XR_LOG_ERROR("Set Acquisition Mode to Continuous fail! nRet [0x%x]", ret);
      return;
    }

    //    将触发模式设置为开启 (On)
    //    参数 "TriggerMode" 的值: 0 表示 Off, 1 表示 On
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
    if (MV_OK != ret)
    {
      XR_LOG_ERROR("Set Trigger Mode to On fail! nRet [0x%x]", ret);
      return;
    }

    //    设置触发源为外部硬件触发 (Line0)
    //    可用的值通常有 "Line0", "Line1", "Line2", "Software",
    //    "FrequencyConverter" 等 请根据您的物理接线选择正确的一项
    ret = MV_CC_SetEnumValueByString(handle_, "TriggerSource", "Line0");
    if (MV_OK != ret)
    {
      XR_LOG_ERROR("Set Trigger Source to Line0 fail! nRet [0x%x]", ret);
      return;
    }

    //    (可选) 设置触发激活方式
    //    例如设置为上升沿触发 "RisingEdge"
    //    其他可选值如 "FallingEdge", "LevelHigh", "LevelLow"
    ret = MV_CC_SetEnumValueByString(handle_, "TriggerActivation", "RisingEdge");
    if (MV_OK != ret)
    {
      XR_LOG_ERROR("Set Trigger Activation to RisingEdge fail! nRet [0x%x]", ret);
      return;
    }
  }
  else
  {
    // 将触发模式设置为开启 (On)
    // 参数 "TriggerMode" 的值: 0 表示 Off, 1 表示 On
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    if (MV_OK != ret)
    {
      XR_LOG_ERROR("Set Trigger Mode to Off fail! nRet [0x%x]", ret);
      return;
    }
  }

  SetEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  SetEnumValue("GainAuto", MV_GAIN_MODE_OFF);
  SetFloatValue("ExposureTime", this->cfg_.exposure_ms);
  SetFloatValue("Gain", this->cfg_.gain);

  ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", 249.0);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_SetFloatValue(set framerate) failed: {:#x}", ret);
    return;
  }
  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  this->hik_state_ = Hik::RUNNING;
}

void HikCamera::ProtectRunning()
{
  this->guard_.protectthread =
      std::thread{[this]() -> void
                  {
                    std::unique_lock<std::mutex> lock(this->guard_.mux);
                    while (true)
                    {
                      this->guard_.HikIsquit.wait(
                          lock, [this] { return (this->hik_state_ == Hik::STOPPED); });
                      this->CaptureStop();
                      this->CaptureInit();
                      std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                  }};
}

void HikCamera::CaptureStop()
{
  this->hik_state_ = Hik::STOPPED;
  if (hik_sdk_thread_.joinable())
  {
    hik_sdk_thread_.join();
  }

  unsigned int ret = 0;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  // ret = MV_CC_SetCommandValue(handle_, "DeviceReset");
  // if (ret != MV_OK) {
  //     tools::logger()->error("Hard Reset failed:
  //     MV_CC_SetCommandValue('DeviceReset') failed with {:#x}", ret);
  //     // 即使失败，也尝试关闭设备
  //     MV_CC_CloseDevice(handle_);
  //     MV_CC_DestroyHandle(handle_);
  //     return ;
  // }
  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }
}

void HikCamera::SetFloatValue(const std::string& name, double value)
{
  unsigned int ret = 0;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), static_cast<float>(value));

  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name.c_str(), value,
                 ret);
    return;
  }
}

void HikCamera::SetEnumValue(const std::string& name, unsigned int value)
{
  unsigned int ret = 0;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK)
  {
    XR_LOG_ERROR("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name.c_str(), value,
                 ret);
    return;
  }
}
