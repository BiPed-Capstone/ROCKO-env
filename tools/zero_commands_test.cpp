// Host-side self-test for rocko_env::write_zero_to_all (ZeroCommands.hpp), the zero-on-deactivate of
// AddFeedforwardController (row 4.14 of docs/next-steps.md). No ROS; stand-in command interfaces
// whose set_value can be told to fail.
//
// Build and run (from the repo root):
//   g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/controllers/add_feedforward/include \
//       tools/zero_commands_test.cpp -o /tmp/zero_commands_test && /tmp/zero_commands_test
// Exit status 0 and "all ZeroCommands tests passed" mean success. This proves the counting only; that
// on_deactivate returns ERROR on a non-zero count, and what controller_manager and the supervisor then
// do, is read from the code, not run (the failure cannot be provoked on the mock).

#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <vector>

#include "rocko_env/ZeroCommands.hpp"

namespace
{
struct FakeCommandInterface
{
  double value = 99.0;  // a non-zero last command
  bool fails = false;
  bool set_value(double v)
  {
    if (fails) {
      return false;  // like a LoanedCommandInterface that could not store the value
    }
    value = v;
    return true;
  }
};

void test_all_succeed()
{
  std::vector<FakeCommandInterface> c(2);
  assert(rocko_env::write_zero_to_all(c) == 0);
  assert(c[0].value == 0.0 && c[1].value == 0.0);
}

void test_one_fails_others_still_zeroed()
{
  std::vector<FakeCommandInterface> c(3);
  c[0].fails = true;  // the FIRST fails: the later ones must still be tried
  assert(rocko_env::write_zero_to_all(c) == 1);
  assert(c[0].value == 99.0 && c[1].value == 0.0 && c[2].value == 0.0);
}

void test_all_fail()
{
  std::vector<FakeCommandInterface> c(2);
  c[0].fails = c[1].fails = true;
  assert(rocko_env::write_zero_to_all(c) == 2);
}

void test_none()
{
  std::vector<FakeCommandInterface> c;
  assert(rocko_env::write_zero_to_all(c) == 0);
}
}  // namespace

int main()
{
  test_all_succeed();
  test_one_fails_others_still_zeroed();
  test_all_fail();
  test_none();
  std::puts("all ZeroCommands tests passed");
  return 0;
}
