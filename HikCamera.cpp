#include "HikCamera.hpp"

#include <opencv2/opencv.hpp>
#include <unordered_map>

HikCamera::HikCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                     CameraBase::CameraInfo info, RuntimeParam runtime)
    : CameraBase(hw, "hik_camera"), info_(info), runtime_(runtime)
{
  XR_LOG_INFO("Starting HikCamera (template format, logic preserved)");

  info_.encoding = CameraBase::Encoding::RGB8;

  if (!CaptureStart())
  {
    XR_LOG_ERROR("OpenAndConfigure failed");
    return;
  }
  if (!StartGrabbing())
  {
    XR_LOG_ERROR("StartGrabbing failed");
    CaptureStop();
    return;
  }

  camera_state_.store(true);
  stopped_flag_.store(false);

  capture_thread_.Create(this, HikCamera::CapThreadFun, "HikCapture",
                         static_cast<size_t>(128 * 1024),
                         LibXR::Thread::Priority::REALTIME);
  cap_thread_created_ = true;

  guard_thread_.Create(this, HikCamera::GuardThreadFun, "HikGuard",
                       static_cast<size_t>(64 * 1024), LibXR::Thread::Priority::REALTIME);
  guard_thread_created_ = true;

  app.Register(*this);
}

HikCamera::~HikCamera()
{
  camera_state_.store(false);

  if (cap_thread_created_)
  {
    pthread_join(static_cast<LibXR::libxr_thread_handle>(capture_thread_), nullptr);
    cap_thread_created_ = false;
  }

  stop_event_.Post();
  if (guard_thread_created_)
  {
    pthread_join(static_cast<LibXR::libxr_thread_handle>(guard_thread_), nullptr);
    guard_thread_created_ = false;
  }
  CaptureStop();
  XR_LOG_INFO("HikCamera destroyed");
}

// ===== CameraBase controls =====
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

void HikCamera::SetRuntimeParam(const RuntimeParam& p)
{
  runtime_ = p;
  UpdateParameters();
}

bool HikCamera::CaptureStart()
{
  MV_CC_DEVICE_INFO_LIST device_list{};
  auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  XR_LOG_INFO("Enum ret=%x, count=%d", ret, device_list.nDeviceNum);

  if (ret != MV_OK || device_list.nDeviceNum == 0)
  {
    XR_LOG_ERROR("No camera found or enum failed");
    return false;  // original semantics: do not loop, fail fast
  }

  if (MV_OK != MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]))
  {
    XR_LOG_ERROR("MV_CC_CreateHandle failed");
    return false;
  }
  if (MV_OK != MV_CC_OpenDevice(camera_handle_))
  {
    XR_LOG_ERROR("MV_CC_OpenDevice failed");
    return false;
  }

  unsigned int n_image_node_num = 1;
  if (MV_OK != MV_CC_SetImageNodeNum(camera_handle_, n_image_node_num))
  {
    XR_LOG_WARN("SetImageNodeNum failed");
  }

  if (!runtime_.autocap)
  {
    MV_CC_SetEnumValueByString(camera_handle_, "AcquisitionMode", "Continuous");
    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 1);  // On
    MV_CC_SetEnumValueByString(camera_handle_, "TriggerSource", "Line0");
    MV_CC_SetEnumValueByString(camera_handle_, "TriggerActivation", "RisingEdge");
  }
  else
  {
    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);  // Off
  }

  MV_CC_SetEnumValue(camera_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(camera_handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", runtime_.exposure_time);
  MV_CC_SetFloatValue(camera_handle_, "Gain", runtime_.gain);

  MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", 249.0);

  MV_CC_GetImageInfo(camera_handle_, &img_info_);

  XR_LOG_PASS("HikCamera configured");
  XR_LOG_PASS("Camera Gain: %f, ExposureTime: %f us, AutoCap: %d", runtime_.gain,
              runtime_.exposure_time, runtime_.autocap);
  return true;
}

bool HikCamera::StartGrabbing()
{
  if (MV_OK != MV_CC_StartGrabbing(camera_handle_))
  {
    XR_LOG_ERROR("MV_CC_StartGrabbing failed");
    return false;
  }
  return true;
}

void HikCamera::CaptureStop()
{
  if (!camera_handle_)
  {
    return;
  }

  MV_CC_StopGrabbing(camera_handle_);
  auto ret = MV_CC_SetCommandValue(camera_handle_, "DeviceReset");
  if (ret != MV_OK)
  {
    XR_LOG_WARN("DeviceReset failed: %x", ret);
  }
  MV_CC_CloseDevice(camera_handle_);
  MV_CC_DestroyHandle(camera_handle_);
  camera_handle_ = nullptr;
}

void HikCamera::UpdateParameters()
{
  if (!camera_handle_)
  {
    return;
  }

  MVCC_FLOATVALUE f_exp{};
  if (MV_OK == MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_exp))
  {
    if (runtime_.exposure_time < f_exp.fMin)
    {
      runtime_.exposure_time = f_exp.fMin;
    }
    if (runtime_.exposure_time > f_exp.fMax)
    {
      runtime_.exposure_time = f_exp.fMax;
    }
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", runtime_.exposure_time);
    XR_LOG_INFO("Exposure time: %f us", runtime_.exposure_time);
  }
  else
  {
    XR_LOG_WARN("Get ExposureTime range failed");
  }

  MVCC_FLOATVALUE f_gain{};
  if (MV_OK == MV_CC_GetFloatValue(camera_handle_, "Gain", &f_gain))
  {
    if (runtime_.gain < f_gain.fMin)
    {
      runtime_.gain = f_gain.fMin;
    }
    if (runtime_.gain > f_gain.fMax)
    {
      runtime_.gain = f_gain.fMax;
    }
    MV_CC_SetFloatValue(camera_handle_, "Gain", runtime_.gain);
    XR_LOG_INFO("Gain: %f", runtime_.gain);
  }
  else
  {
    XR_LOG_WARN("Get Gain range failed");
  }
}

// ===== threads =====
void HikCamera::CapThreadFun(HikCamera* self)
{
  XR_LOG_INFO("Hik capture thread started");
  MV_FRAME_OUT raw{};

  while (self->camera_state_.load())
  {
    unsigned int n_msec = 100;  // original wait
    auto ret = MV_CC_GetImageBuffer(self->camera_handle_, &raw, n_msec);
    if (ret != MV_OK)
    {
      XR_LOG_WARN("MV_CC_GetImageBuffer failed: %x", ret);
      break;  // break -> guard will restart
    }

    // timestamp
    self->info_.timestamp = LibXR::Timebase::GetMicroseconds();

    // size & step
    const int W = raw.stFrameInfo.nWidth;
    const int H = raw.stFrameInfo.nHeight;
    self->info_.width = static_cast<uint32_t>(W);
    self->info_.height = static_cast<uint32_t>(H);
    self->info_.step = static_cast<uint32_t>(W * CH);

    // construct src cv::Mat (Bayer)
    cv::Mat src(H, W, CV_8U, raw.pBufAddr);

    static const std::unordered_map<MvGvspPixelType, int> KMAP = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB},
    };

    auto it = KMAP.find(raw.stFrameInfo.enPixelType);
    if (it == KMAP.end())
    {
      XR_LOG_WARN("Unsupported pixel type: %d", raw.stFrameInfo.enPixelType);
      MV_CC_FreeImageBuffer(self->camera_handle_, &raw);
      continue;
    }

    cv::Mat rgb;
    cv::cvtColor(src, rgb, static_cast<int>(it->second));

    MV_CC_FreeImageBuffer(self->camera_handle_, &raw);
    cv::imshow("raw_img", rgb);

    self->frame_topic_.Publish(rgb);
    self->info_topic_.Publish(self->info_);
  }

  // on exit -> mark stopped and signal guard
  {
    LibXR::Mutex::LockGuard lg(self->state_mtx_);
    self->stopped_flag_.store(true);
  }
  self->stop_event_.Post();
  XR_LOG_INFO("Hik capture thread stopped");
}

void HikCamera::GuardThreadFun(HikCamera* self)
{
  XR_LOG_INFO("Hik guard thread started");
  for (;;)
  {
    // wait for stop event (posted by capture thread)
    self->stop_event_.Wait();

    if (!self->camera_state_.load())
    {
      break;  // shutting down
    }

    // stop -> start -> run
    self->CaptureStop();

    if (!self->CaptureStart())
    {
      XR_LOG_ERROR("Re-open failed; retry in 1s");
      LibXR::Thread::Sleep(1000);
      continue;
    }
    if (!self->StartGrabbing())
    {
      XR_LOG_ERROR("Re-start grabbing failed; retry in 1s");
      LibXR::Thread::Sleep(1000);
      continue;
    }

    // re-launch capture loop if previous one has exited
    if (!self->camera_state_.load())
    {
      break;
    }
    self->capture_thread_.Create(self, CapThreadFun, "HikCapture",
                                 static_cast<size_t>(128 * 1024),
                                 LibXR::Thread::Priority::MEDIUM);
  }
  XR_LOG_INFO("Hik guard thread stopped");
}
