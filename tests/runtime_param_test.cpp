#include "HikCamera.hpp"

constexpr CameraTypes::FrameLayout kLayout{1440U, 1080U, 4320U,
                                           CameraTypes::Encoding::BGR8};
using Camera = HikCamera<kLayout>;
constexpr Camera::RuntimeParam kDefault{};
constexpr Camera::RuntimeParam kCurrent{"camera", "image", "imu", 16.0F, 2000.0F,
                                        true,     249.0F,  100U,  3U,    false};
constexpr Camera::RuntimeParam kLegacy{"camera", "image", "imu", 16.0F, 2000.0F, true,
                                       249.0F,   100U,    3U,    1U,    1U,      false};
constexpr Camera::RuntimeParam kNative{"camera", "image", "imu",  16.0F, 2000.0F,
                                       true,     249.0F,  100U,   3U,    false,
                                       1U,       1U,      50000U, 50000U};
static_assert(kDefault.wide_decimation_x == 2U && kDefault.wide_decimation_y == 2U);
static_assert(kDefault.wide_trigger_period_us == Camera::wide_trigger_period_us);
static_assert(kCurrent.narrow_trigger_period_us == Camera::narrow_trigger_period_us);
static_assert(kLegacy.wide_decimation_x == 1U && !kLegacy.rotate_180);
static_assert(kNative.wide_trigger_period_us == 50000U);
static_assert(kNative.narrow_trigger_period_us == 50000U);
static_assert(kNative.wide_decimation_x == 1U && kNative.wide_decimation_y == 1U);
int main() { return 0; }
