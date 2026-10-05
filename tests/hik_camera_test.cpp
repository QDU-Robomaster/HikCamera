#include <cstdio>
#include <cstdlib>

#include "HikCamera.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

void TestNodeGeometry()
{
  using HikCameraDetail::ToNodeGeometry;
  // WIDE：原生 (80, 24) 在跳采后的节点偏移为 (40, 12)，与 rawcap 一致。
  const auto wide = ToNodeGeometry(CameraBase::WIDE_GEOMETRY);
  Expect(wide.decimation == 2 && wide.width == 640 && wide.height == 512, "wide size");
  Expect(wide.offset_x == 40 && wide.offset_y == 12, "wide offsets");
  const auto narrow = ToNodeGeometry({400, 284, 1});
  Expect(narrow.decimation == 1 && narrow.offset_x == 400 && narrow.offset_y == 284,
         "narrow offsets");
}

void TestTicksToUs()
{
  using HikCameraDetail::TicksToUs;
  Expect(TicksToUs(123456789, 1000000) == 123456789, "microsecond ticks");
  Expect(TicksToUs(1000000000, 1000000000) == 1000000, "nanosecond ticks");
  Expect(TicksToUs(2500000000ULL, 1000000000) == 2500000, "nanosecond ticks, 2.5 s");
  // 大 tick 数不溢出 / Large tick counts do not overflow.
  Expect(TicksToUs(1ULL << 62, 1000000000) == (1ULL << 62) / 1000, "large ticks");
}

// 只编译链接、不调用：确认驱动对 SDK 的调用都能链接 / Compiled and linked, never called:
// proves every SDK call the driver makes links.
[[maybe_unused]] void LinkCheck()
{
  constexpr CameraTypes::CameraCalibration CALIBRATION{
      1440, 1080, 2328.69, 2328.67, 733.36, 540.62, {0.0, 0.0, 0.0, 0.0, 0.0}};
  HikCamera camera(CALIBRATION, {0.5, 0.5}, "link", {2000.0F, 0.0F, 8, true, 0.0F});
}

void TestAdcBitDepth()
{
  Expect(HikCameraDetail::AdcBitDepthEnum(8) == 0, "adc 8");
  Expect(HikCameraDetail::AdcBitDepthEnum(12) == 3, "adc 12");
}
}  // namespace

int main()
{
  TestNodeGeometry();
  TestTicksToUs();
  TestAdcBitDepth();
  std::puts("hik_camera_test passed");
  return 0;
}
