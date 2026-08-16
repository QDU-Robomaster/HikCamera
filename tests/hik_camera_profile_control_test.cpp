#include <cstdlib>
#include <iostream>
#include <vector>

#include "HikCameraProfileControl.hpp"

namespace
{

using HikCameraDetail::IntegerRange;
using HikCameraDetail::OriginalStateSnapshot;
using HikCameraDetail::ProfileSwitchOutcome;
using HikCameraDetail::SdkStreamState;

void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void TestAlignedCenteredOffset()
{
  constexpr IntegerRange aligned_range{.minimum = 0, .maximum = 720, .increment = 16};
  static_assert(HikCameraDetail::CenteredAlignedOffset(1440, 720, aligned_range) == 352);
  static_assert(HikCameraDetail::CenteredAlignedOffset(1440, 1440, aligned_range) == 0);

  constexpr IntegerRange shifted_range{.minimum = 4, .maximum = 100, .increment = 8};
  Expect(HikCameraDetail::AlignDownToRange(50, shifted_range) == 44,
         "alignment must use the SDK range origin");
  Expect(HikCameraDetail::AlignDownToRange(-1, shifted_range) == 4,
         "alignment must clamp below the SDK range");
  Expect(HikCameraDetail::AlignDownToRange(200, shifted_range) == 100,
         "alignment must clamp above the SDK range");
}

ProfileSwitchOutcome RunRecoveryCase(bool apply_result, bool rollback_result,
                                     bool reopen_result, std::vector<int>& calls)
{
  return HikCameraDetail::RunProfileSwitchWithRecovery(
      [&]()
      {
        calls.push_back(1);
        return apply_result;
      },
      [&]()
      {
        calls.push_back(2);
        return rollback_result;
      },
      [&]()
      {
        calls.push_back(3);
        return reopen_result;
      });
}

void TestRecoveryOrder()
{
  std::vector<int> calls;
  Expect(RunRecoveryCase(true, false, false, calls) == ProfileSwitchOutcome::APPLIED &&
             calls == std::vector<int>{1},
         "successful apply must not run recovery");

  calls.clear();
  Expect(
      RunRecoveryCase(false, true, false, calls) == ProfileSwitchOutcome::ROLLED_BACK &&
          calls == std::vector<int>({1, 2}),
      "failed apply must first restore the previous profile");

  calls.clear();
  Expect(RunRecoveryCase(false, false, true, calls) == ProfileSwitchOutcome::REOPENED &&
             calls == std::vector<int>({1, 2, 3}),
         "failed rollback must reopen the device");

  calls.clear();
  Expect(
      RunRecoveryCase(false, false, false, calls) == ProfileSwitchOutcome::UNRECOVERED &&
          calls == std::vector<int>({1, 2, 3}),
      "all recovery stages must be attempted exactly once");
}

void TestOriginalStateSnapshot()
{
  struct DeviceState
  {
    int value;
  };

  OriginalStateSnapshot<DeviceState> snapshot;
  Expect(snapshot.Get() == nullptr, "new snapshot must be empty");
  Expect(snapshot.CaptureOnce({.value = 17}), "first state must be captured");
  Expect(!snapshot.CaptureOnce({.value = 99}),
         "reopen must not replace the original state");
  Expect(snapshot.Get() != nullptr && snapshot.Get()->value == 17,
         "original state must remain stable across reopen");

  Expect(!snapshot.CompleteRestore(false, true, true),
         "failed writes must retain the original snapshot");
  Expect(!snapshot.CompleteRestore(true, false, true),
         "readback mismatch must retain the original snapshot");
  Expect(snapshot.Get() != nullptr && snapshot.Get()->value == 17,
         "failed restore must remain retryable");

  Expect(snapshot.CompleteRestore(true, true, false),
         "verified reopen restore must succeed");
  Expect(snapshot.Get() != nullptr && snapshot.Get()->value == 17,
         "reopen restore must retain the lifetime snapshot");
  Expect(snapshot.CompleteRestore(true, true, true),
         "verified final restore must commit");
  Expect(snapshot.Get() == nullptr,
         "only a verified final restore may release the snapshot");
}

void TestSdkStreamState()
{
  SdkStreamState state;
  int stop_calls = 0;
  Expect(!state.Active(), "new SDK stream state must be inactive");
  Expect(state.Stop(
             [&]()
             {
               ++stop_calls;
               return true;
             }) &&
             stop_calls == 0,
         "stopping an inactive SDK stream must not call the SDK");

  state.MarkStarted();
  Expect(state.Active(), "a successful SDK start must be tracked");
  Expect(!state.Stop(
             [&]()
             {
               ++stop_calls;
               return false;
             }) &&
             state.Active() && stop_calls == 1,
         "a failed SDK stop must remain retryable");
  Expect(state.Stop(
             [&]()
             {
               ++stop_calls;
               return true;
             }) &&
             !state.Active() && stop_calls == 2,
         "a successful retry must clear the SDK stream state");
  Expect(state.Stop(
             [&]()
             {
               ++stop_calls;
               return true;
             }) &&
             stop_calls == 2,
         "outer cleanup must not issue a second SDK stop");

  state.MarkStarted();
  state.MarkReleased();
  Expect(!state.Active(), "closing or destroying the handle must clear stream state");
}

}  // namespace

int main()
{
  TestAlignedCenteredOffset();
  TestRecoveryOrder();
  TestOriginalStateSnapshot();
  TestSdkStreamState();
  std::cout << "HikCamera profile-control tests passed\n";
  return 0;
}
