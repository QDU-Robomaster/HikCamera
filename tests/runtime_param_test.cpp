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
static_assert(!kDefault.adc_bit_depth && !kDefault.gamma_enabled);
static_assert(kDefault.gamma == 1.0F);
static_assert(!kCurrent.adc_bit_depth && !kLegacy.adc_bit_depth &&
              !kNative.adc_bit_depth);
constexpr Camera::RuntimeParam kSettings{
    "camera", "image", "imu",  16.0F,  2000.0F,
    false,    100.0F,  100U,   3U,     false,
    1U,       1U,      50000U, 50000U, Camera::AdcBitDepth::BIT_8,
    true,     0.8F};
constexpr Camera::RuntimeParam kSimpleSettings{"camera",
                                               "image",
                                               "imu",
                                               16.0F,
                                               2000.0F,
                                               true,
                                               249.0F,
                                               100U,
                                               3U,
                                               false,
                                               Camera::AdcBitDepth::BIT_12,
                                               true,
                                               1.2F};
constexpr Camera::RuntimeParam kLegacySettings{"camera",
                                               "image",
                                               "imu",
                                               16.0F,
                                               2000.0F,
                                               true,
                                               249.0F,
                                               100U,
                                               3U,
                                               1U,
                                               1U,
                                               false,
                                               Camera::AdcBitDepth::BIT_10,
                                               false,
                                               1.0F};
static_assert(kSettings.adc_bit_depth == Camera::AdcBitDepth::BIT_8);
static_assert(kSettings.gamma_enabled && kSettings.gamma == 0.8F);
static_assert(kSettings.acquisition_frame_rate == 100.0F);
static_assert(kSimpleSettings.adc_bit_depth == Camera::AdcBitDepth::BIT_12);
static_assert(kLegacySettings.adc_bit_depth == Camera::AdcBitDepth::BIT_10);
static_assert(kLegacySettings.wide_decimation_x == 1U && !kLegacySettings.rotate_180);
int main() { return 0; }
