#pragma once

#include <cstdint>
#include <type_traits>
#include <utility>

namespace HikCameraDetail
{

struct IntegerRange
{
  int64_t minimum = 0;
  int64_t maximum = 0;
  int64_t increment = 1;
};

[[nodiscard]] constexpr int64_t AlignDownToRange(int64_t value,
                                                 const IntegerRange& range) noexcept
{
  if (range.maximum < range.minimum)
  {
    return range.minimum;
  }
  if (value < range.minimum)
  {
    value = range.minimum;
  }
  if (value > range.maximum)
  {
    value = range.maximum;
  }

  const int64_t increment = range.increment > 1 ? range.increment : 1;
  return range.minimum + ((value - range.minimum) / increment) * increment;
}

[[nodiscard]] constexpr int64_t CenteredAlignedOffset(int64_t full_extent,
                                                      int64_t target_extent,
                                                      const IntegerRange& range) noexcept
{
  return AlignDownToRange((full_extent - target_extent) / 2, range);
}

enum class ProfileSwitchOutcome : uint8_t
{
  APPLIED = 0,
  ROLLED_BACK,
  REOPENED,
  UNRECOVERED,
};

/**
 * @brief Tracks whether one successful SDK stream start still needs a stop.
 */
class SdkStreamState
{
 public:
  [[nodiscard]] constexpr bool Active() const noexcept { return active_; }

  constexpr void MarkStarted() noexcept { active_ = true; }

  template <typename StopFn>
  [[nodiscard]] constexpr bool Stop(StopFn&& stop) noexcept(
      noexcept(std::forward<StopFn>(stop)()))
  {
    if (!active_)
    {
      return true;
    }
    if (!std::forward<StopFn>(stop)())
    {
      return false;
    }
    active_ = false;
    return true;
  }

  constexpr void MarkReleased() noexcept { active_ = false; }

 private:
  bool active_{false};
};

/**
 * @brief Keeps the first device-state snapshot until a verified final restore.
 *
 * Reopen attempts may restore the snapshot temporarily, but must not replace or
 * release it. A failed write or readback therefore remains retryable on a later
 * handle.
 */
template <typename State>
class OriginalStateSnapshot
{
  static_assert(std::is_trivially_copyable_v<State>);

 public:
  [[nodiscard]] constexpr bool CaptureOnce(const State& state) noexcept
  {
    if (saved_)
    {
      return false;
    }
    state_ = state;
    saved_ = true;
    return true;
  }

  [[nodiscard]] constexpr const State* Get() const noexcept
  {
    return saved_ ? &state_ : nullptr;
  }

  [[nodiscard]] constexpr bool CompleteRestore(bool writes_succeeded,
                                               bool readback_matches,
                                               bool release_on_success) noexcept
  {
    if (!saved_)
    {
      return true;
    }
    if (!writes_succeeded || !readback_matches)
    {
      return false;
    }
    if (release_on_success)
    {
      saved_ = false;
    }
    return true;
  }

 private:
  State state_{};
  bool saved_{false};
};

template <typename Apply, typename Rollback, typename Reopen>
[[nodiscard]] ProfileSwitchOutcome RunProfileSwitchWithRecovery(Apply&& apply,
                                                                Rollback&& rollback,
                                                                Reopen&& reopen)
{
  if (std::forward<Apply>(apply)())
  {
    return ProfileSwitchOutcome::APPLIED;
  }
  if (std::forward<Rollback>(rollback)())
  {
    return ProfileSwitchOutcome::ROLLED_BACK;
  }
  if (std::forward<Reopen>(reopen)())
  {
    return ProfileSwitchOutcome::REOPENED;
  }
  return ProfileSwitchOutcome::UNRECOVERED;
}

}  // namespace HikCameraDetail
