#ifndef ROCKO_ENV_ZERO_COMMANDS
#define ROCKO_ENV_ZERO_COMMANDS

// Writes 0.0 to every command interface and says how many writes failed (row 4.14 of
// docs/next-steps.md). Used by AddFeedforwardController::on_deactivate(), where the zero is what
// stops the wheel (finding H-7: the motor plugin keeps writing whatever is left in its command slot).
//
// A template so it can be tested on a laptop with stand-in interfaces
// (tools/zero_commands_test.cpp): anything with `bool set_value(double)` works, which is what
// ros2_control's LoanedCommandInterface provides. No allocation, no lock waited on. The boring
// version is this loop written inline in on_deactivate; it is a template only so the test needs no
// ros2_control.

#include <cstddef>

namespace rocko_env
{

// Tries every interface, even after one fails, so one stuck slot does not leave the others running.
// Returns the number of interfaces whose set_value(0.0) returned false; 0 means all were zeroed.
template<typename CommandInterfaces>
std::size_t write_zero_to_all(CommandInterfaces & interfaces)
{
  std::size_t failed = 0;
  for (auto & command_interface : interfaces) {
    if (!command_interface.set_value(0.0)) {
      ++failed;
    }
  }
  return failed;
}

}  // namespace rocko_env

#endif  // ROCKO_ENV_ZERO_COMMANDS
