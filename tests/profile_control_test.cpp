#include <cstdlib>
#include <iostream>

#include "HikCameraProfileControl.hpp"

void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

int main()
{
  using HikCameraDetail::RunProfileSwitchWithRetry;
  for (uint32_t success_at = 1U; success_at <= 4U; ++success_at)
  {
    uint32_t attempts = 0U;
    const bool result = RunProfileSwitchWithRetry(
        [&](uint32_t attempt)
        {
          Expect(attempt == ++attempts, "attempt numbering must be consecutive");
          return attempt == success_at;
        });
    Expect(result && attempts == success_at, "success must end retries immediately");
  }
  uint32_t attempts = 0U;
  Expect(!RunProfileSwitchWithRetry(
             [&](uint32_t)
             {
               ++attempts;
               return false;
             }),
         "persistent failure must report failure");
  Expect(attempts == 4U, "initial attempt plus exactly three retries");

  HikCameraDetail::SdkStreamState stream;
  uint32_t stop_calls = 0U;
  Expect(stream.StopIfActive(
             [&]
             {
               ++stop_calls;
               return true;
             }),
         "inactive stream is stopped");
  Expect(stop_calls == 0U, "do not call SDK stop without a successful start");
  stream.MarkStarted();
  Expect(!stream.StopIfActive(
             [&]
             {
               ++stop_calls;
               return false;
             }),
         "stop error must propagate");
  Expect(stream.Active(), "failed stop must block geometry writes on retry");
  Expect(stream.StopIfActive(
             [&]
             {
               ++stop_calls;
               return true;
             }),
         "successful retry stops stream");
  Expect(!stream.Active() && stop_calls == 2U,
         "successful stop clears only SDK activity");
  std::cout << "profile retry and SDK state checks passed\n";
}
