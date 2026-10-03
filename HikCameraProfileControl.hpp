#pragma once

#include <cstdint>
#include <utility>

namespace HikCameraDetail
{

/// 档位切换的重试次数，加首次尝试共四次
/// Retries of a profile switch; together with the first attempt there are four attempts
inline constexpr uint32_t profile_switch_retry_limit = 3U;

/**
 * @brief 记录 SDK 取流状态，与图像处理线程相互独立。
 *        Track the SDK stream state independently of the image-processing thread.
 */
class SdkStreamState
{
 public:
  /**
   * @brief SDK 取流是否处于已启动状态。
   *        Whether the SDK stream is started.
   *
   * @return 已启动为 true。
   *         True when started.
   */
  [[nodiscard]] bool Active() const noexcept { return active_; }

  /**
   * @brief 标记 SDK 取流已启动。
   *        Mark the SDK stream as started.
   */
  void MarkStarted() noexcept { active_ = true; }

  /**
   * @brief 取流处于已启动状态时执行停流回调，回调返回 true 后标记为已停止。
   *        Run the stop callback when the stream is started, and mark it as stopped
   *        once the callback returns true.
   *
   * @tparam Stop 停流回调类型，返回 bool。
   *              Stop callback type returning bool.
   * @param stop 停流回调。
   *             Stop callback.
   * @return 取流本就未启动或停流成功为 true，停流回调失败为 false。
   *         True when the stream was not started or stopping succeeded, false when the
   *         stop callback failed.
   */
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

/**
 * @brief 对同一个目标档位最多完整尝试四次。
 *        Make at most four complete attempts at the same requested profile.
 *
 * @tparam Attempt 尝试回调类型，参数为从 1 开始的尝试序号，返回 bool。
 *                 Attempt callback type taking the attempt number from 1 and returning
 *                 bool.
 * @param attempt 尝试回调。
 *                Attempt callback.
 * @return 任一次尝试成功为 true，四次都失败为 false。
 *         True when any attempt succeeds, false when all four fail.
 */
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
