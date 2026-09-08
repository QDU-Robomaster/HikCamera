#pragma once

#include <cstdint>
#include <utility>

namespace HikCameraDetail
{

inline constexpr uint32_t profile_switch_retry_limit = 3U;

/** Tracks SDK acquisition independently of the image-processing thread. */
class SdkStreamState
{
 public:
  [[nodiscard]] bool Active() const noexcept { return active_; }

  void MarkStarted() noexcept { active_ = true; }

  template <typename Stop>
  [[nodiscard]] bool StopIfActive(Stop&& stop)
  {
    if (!active_)
    {
      return true;
    }
    if (!std::forward<Stop>(stop)())
    {
      return false;
    }
    active_ = false;
    return true;
  }

 private:
  bool active_{false};
};

/** Make at most four complete attempts at the same requested profile. */
template <typename Attempt>
[[nodiscard]] bool RunProfileSwitchWithRetry(Attempt&& attempt)
{
  for (uint32_t index = 0U; index <= profile_switch_retry_limit; ++index)
  {
    if (attempt(index + 1U))
    {
      return true;
    }
  }
  return false;
}

}  // namespace HikCameraDetail
