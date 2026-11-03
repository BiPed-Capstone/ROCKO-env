// Copyright (c) 2023, PAL Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rocko_env/AddFeedforwardController.hpp"
#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

namespace rocko_env
{

controller_interface::CallbackReturn AddFeedforwardController::on_init()
{
  RCLCPP_INFO(this->get_node()->get_logger(), "load successful");
  try
  {
    param_listener_ = std::make_shared<add_feedforward_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // set up subscriber for feedforward data
  subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    params_.feedforward_topic_name, 10, std::bind(&AddFeedforwardController::topic_callback, this, _1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AddFeedforwardController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration AddFeedforwardController::state_interface_configuration()
  const
{
  // The only state this controller reads is pitch, for the tipover cutoff (finding H-4). With the
  // cutoff disabled it claims nothing. Called after on_configure, which sets pitch_interface_name_.
  // A state interface can be claimed by several controllers at once (pitch_pid_controller reads
  // the same one), so this takes nothing away from the cascade.
  if (!tipover_enabled_)
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = {pitch_interface_name_};
  return state_interfaces_config;
}

controller_interface::CallbackReturn AddFeedforwardController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interface_names_ = params_.interfaces;

  joints_cmd_sub_ = this->get_node()->create_subscription<DataType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const DataType::SharedPtr msg)
    {
      // check if message is correct size, if not ignore
      if (msg->data.size() == command_interface_names_.size())
      {
        rt_buffer_ptr_.writeFromNonRT(msg);
      }
      else
      {
        RCLCPP_ERROR(
          this->get_node()->get_logger(), "Invalid command received of %zu size, expected %zu size",
          msg->data.size(), command_interface_names_.size());
      }
    });

  // pre-reserve command interfaces
  command_interfaces_.reserve(command_interface_names_.size());

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

  // The names should be in the same order as for command interfaces for easier matching
  reference_interface_names_ = command_interface_names_;
  // for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize(
    reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());
  // Per-interface state for the stale-reference hold in update_and_write_commands. Sized here
  // (not in the 100 Hz loop) so the loop never allocates.
  last_command_.assign(command_interface_names_.size(), 0.0);
  stale_reference_cycles_.assign(command_interface_names_.size(), 0);

  return configure_tipover_cutoff();
}

controller_interface::CallbackReturn AddFeedforwardController::configure_tipover_cutoff()
{
  // --- Tipover cutoff (finding H-4) ---
  // Why: past a certain lean the wheels can no longer get back under the body, so the robot cannot
  // recover. Driving them then only spins them against the floor, or shoves the robot that is
  // already lying there; zero output is the least harmful thing the motors can do (TipoverLatch.hpp
  // says more). This ADDS a safeguard and weakens nothing: with the robot inside the limit every
  // command is exactly what it was before.
  // The limit is a hardware decision, so it is a parameter (tipover_pitch_deg, proposed 35 deg from
  // the H-4 write-up in docs/claude-analysis/balance-loop-findings.md, not from this robot's
  // physics; a tethered measurement replaces it). It is compared with the pitch the IMU plugin
  // reports, which has ICM20948.py's +7.6 deg pitch_offset already added. Whether reported 0.0 is
  // truly upright is unmeasured, so in true lean the cutoff may sit nearer 35 - 7.6 on one side and
  // 35 + 7.6 on the other.
  tipover_pitch_deg_ = params_.tipover_pitch_deg;
  rearm_max_pitch_deg_ = params_.rearm_max_pitch_deg;
  tipover_enabled_ = TipoverLatch::is_enabled(tipover_pitch_deg_);
  if (!tipover_enabled_)
  {
    // Loud on purpose: running a balancing robot without the cutoff should never go unnoticed.
    // (Only here, once: a repeating reminder would have to come from the 100 Hz loop.)
    RCLCPP_WARN(
      get_node()->get_logger(),
      "TIPOVER CUTOFF DISABLED (tipover_pitch_deg = %f, which is <= 0). This controller "
      "will keep driving the wheel however far the robot leans.",
      tipover_pitch_deg_);
    rearm_sub_.reset();  // in case an earlier configure (with the cutoff on) created it
    return controller_interface::CallbackReturn::SUCCESS;
  }

  if (!TipoverLatch::is_valid_rearm_max(rearm_max_pitch_deg_, tipover_pitch_deg_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "rearm_max_pitch_deg = %f is unusable: it must be > 0 and < tipover_pitch_deg (%f), so a "
      "re-arm needs the robot clearly back up. Refusing to configure.",
      rearm_max_pitch_deg_, tipover_pitch_deg_);
    return controller_interface::CallbackReturn::ERROR;
  }

  // Default pitch interface: "<joint>/pitch", where <joint> comes from the first command
  // interface ("left_wheel_joint/velocity" -> "left_wheel_joint/pitch"). The IMU plugin, the mock
  // and SimPlant all export pitch under each wheel joint, so rocko_controllers.yaml needs no edit.
  // The boring alternative, making pitch_interface a required key, was rejected because it would
  // need a line added to rocko_controllers.yaml, a human-gated file (CLAUDE.md section 3).
  pitch_interface_name_ = params_.pitch_interface;
  if (pitch_interface_name_.empty())
  {
    const std::string & first_command = command_interface_names_.front();  // not_empty<> checked
    const size_t slash = first_command.rfind('/');
    if (slash == std::string::npos || slash == 0)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Cannot derive the pitch interface from '%s' (expected '<joint>/<interface>'); set "
        "pitch_interface explicitly.",
        first_command.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    pitch_interface_name_ = first_command.substr(0, slash) + "/pitch";
  }

  // Re-arming is a deliberate operator action, never automatic. The callback runs on the
  // executor thread and only sets an atomic flag; the 100 Hz loop decides (see update).
  // kRearmTopic is absolute, so the left and right instances subscribe to the same topic and one
  // publish re-arms both wheels. The logged command uses "-w 2": `ros2 topic pub --once` waits for
  // one subscriber by default, and could publish before it has found the second wheel's.
  rearm_sub_ = get_node()->create_subscription<std_msgs::msg::Empty>(
    kRearmTopic, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Empty::SharedPtr /*msg*/) { tipover_latch_.request_rearm(); });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Tipover cutoff ON: |%s| > %.1f deg zeroes this wheel's command and latches. Re-arm both "
    "wheels, with |pitch| <= %.1f deg, with: ros2 topic pub --once -w 2 %s std_msgs/msg/Empty {}",
    pitch_interface_name_.c_str(), tipover_pitch_deg_, rearm_max_pitch_deg_, kRearmTopic);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AddFeedforwardController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_names_, std::string(""), ordered_interfaces) ||
    command_interface_names_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Tipover cutoff (finding H-4): refuse to activate without the pitch input rather than run
  // silently without the cutoff. (controller_manager normally refuses first if the interface does
  // not exist; this makes the rule hold here too.)
  if (tipover_enabled_)
  {
    if (state_interfaces_.size() != 1 || state_interfaces_[0].get_name() != pitch_interface_name_)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Tipover cutoff needs state interface '%s', but it is not available (%zu state interfaces "
        "claimed). Refusing to activate. To run without the cutoff, set tipover_pitch_deg <= 0.",
        pitch_interface_name_.c_str(), state_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    // Activation is itself a deliberate operator action, so it starts armed. If the robot is still
    // past the limit, the first update trips again before any non-zero command is written.
    tipover_latch_.reset();
    refusal_log_age_cycles_ = kRefusalLogMinCycles;  // the first refusal after activation is logged
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  // Start with "no command yet" (NaN); update_and_write_commands turns NaN into a 0.0 stop, so the
  // first cycles after activation command zero velocity rather than whatever was left over.
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  consecutive_write_failures_ = 0;
  // Nothing has been commanded yet: a NaN gap right after activation holds 0.0, not old data.
  // (Same sizes as set in on_configure, so assign() does not allocate.)
  last_command_.assign(last_command_.size(), 0.0);
  stale_reference_cycles_.assign(stale_reference_cycles_.size(), 0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AddFeedforwardController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

  // Command zero velocity on the way out (finding H-7). The motor plugin keeps writing whatever is
  // in its command slot, so if this controller just stops, the wheel keeps its last command.
  // Never waits on a lock: set_value() tries to store a number and gives up (returns false) if it cannot. A failure is counted and logged (throttled)
  // but does not stop the other interfaces from being zeroed.
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    last_command_[i] = 0.0;
    if (!command_interfaces_[i].set_value(0.0))
    {
      ++failed_command_writes_;
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), kLogThrottleMs,
        "set_value(0.0) failed on deactivate; %lu failed writes since start", failed_command_writes_);
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool AddFeedforwardController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::return_type AddFeedforwardController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Why this function never skips a write, and never ignores a failed one:
  //   - Finding H-2: a skipped write leaves the PREVIOUS speed running on the motor. Before this
  //     change a NaN reference meant "skip", so a dead upstream controller left the last command
  //     driving the wheel. This controller now always sets a value. The motor plugin's write()
  //     now always writes the speed pin too (a command near zero writes 0 %). Both are checked by
  //     reading the code and building it only, not on the robot.
  //   - Finding M-5: set_value() can return false. That must not look like success.
  //   - Finding H-7 (mitigation): see the NaN reset below.
  //   - Finding H-4: past the tipover limit the command is 0.0 until an operator re-arms.
  // Findings are described in docs/claude-analysis/balance-loop-findings.md.
  const double feedforward = _feedforward.load(std::memory_order_relaxed);
  bool all_writes_ok = true;

  // Tipover cutoff (finding H-4): read pitch once, let the latch decide, log only on a change.
  // The trip and re-arm logs are not throttled but cannot repeat every cycle: a trip is logged once
  // (the latch then stays disarmed, so kTripped cannot come back until a re-arm), and a re-arm is
  // logged once per operator request. "Refused" has its own per-instance throttle
  // (refusal_log_age_cycles_), in case someone publishes /tipover_rearm in a loop. (rclcpp's
  // throttle would be wrong for all three: its state is shared by every instance of this class, so
  // the right wheel's line would be hidden behind the left one's.)
  // NOTE: tipover_tripped_this_cycle() below holds a copy of this block (review finding: extract
  // it). Replacing this block with a call to it was refused by the safety-word guard hook, which
  // counts words and cannot tell a move from a removal; a human finishes the move.
  bool tipover_stop = false;
  if (tipover_enabled_)
  {
    if (refusal_log_age_cycles_ < kRefusalLogMinCycles)
    {
      ++refusal_log_age_cycles_;
    }
    // get_optional() gives nothing if the value could not be read this cycle; treat that as NaN
    // ("no reading"), which neither trips the cutoff nor allows a re-arm.
    const auto pitch_reading = state_interfaces_[0].get_optional();
    const double pitch_deg =
      pitch_reading.has_value() ? *pitch_reading : std::numeric_limits<double>::quiet_NaN();
    switch (tipover_latch_.update(pitch_deg, tipover_pitch_deg_, rearm_max_pitch_deg_))
    {
      case TipoverLatch::Event::kTripped:
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "TIPOVER CUTOFF TRIPPED: |pitch| = %.1f deg > %.1f deg. Wheel command zeroed and latched "
          "(trip %lu). Stand the robot up (|pitch| <= %.1f deg), then re-arm both wheels: "
          "ros2 topic pub --once -w 2 %s std_msgs/msg/Empty {}",
          std::fabs(pitch_deg), tipover_pitch_deg_, tipover_latch_.trips(), rearm_max_pitch_deg_,
          kRearmTopic);
        break;
      case TipoverLatch::Event::kRearmed:
        RCLCPP_WARN(
          get_node()->get_logger(), "Tipover cutoff RE-ARMED by operator at pitch %.1f deg.",
          pitch_deg);
        break;
      case TipoverLatch::Event::kRearmRefused:
        if (refusal_log_age_cycles_ >= kRefusalLogMinCycles)
        {
          refusal_log_age_cycles_ = 0;
          RCLCPP_WARN(
            get_node()->get_logger(),
            "Tipover re-arm refused: pitch %.1f deg is not within +/-%.1f deg "
            "(rearm_max_pitch_deg), or unknown. Still disarmed.",
            pitch_deg, rearm_max_pitch_deg_);
        }
        break;
      case TipoverLatch::Event::kNone:
        break;
    }
    tipover_stop = !tipover_latch_.armed();
  }

  // There is ONE command interface per controller instance (rocko_controllers.yaml: each of
  // left/right_add_feedforward_controller lists a single <wheel>_joint/velocity). The loop is
  // there because `interfaces` is a parameter, not because a second wheel is protected by it.
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    const double reference = reference_interfaces_[i];  // wheel velocity command from upstream

    // In chained mode the upstream pitch_pid_controller writes a fresh value every cycle. If it stops,
    // nobody overwrites this slot and the old value would be re-sent forever (finding H-7). So
    // consume the value: after reading it, put NaN back. Fresh data replaces the NaN next cycle;
    // no fresh data leaves NaN, which becomes a stop below. In non-chained mode the value is
    // refreshed from the topic buffer, and NaN means "no command received yet".
    if (is_in_chained_mode())
    {
      reference_interfaces_[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // A NaN reference (never started, or upstream stopped) is a STOP: write 0.0. Same in chained
    // and non-chained mode, because 0.0 is the safe command in both. One exception: a short gap
    // of up to kMaxStaleReferenceCycles cycles repeats the last finite command, so a single late
    // upstream cycle does not make the wheel twitch to zero and back.
    double command = 0.0;  // wheel velocity command [same unit as the reference]
    if (std::isnan(reference))
    {
      if (stale_reference_cycles_[i] < kMaxStaleReferenceCycles)
      {
        ++stale_reference_cycles_[i];
        command = last_command_[i];
      }
      // else: the gap is too long, command stays 0.0 (stop).
    }
    else
    {
      stale_reference_cycles_[i] = 0;  // fresh data (even a bad value) ends the gap
      if (std::isfinite(reference + feedforward))
      {
        command = reference + feedforward;
      }
      else
      {
        // +/-inf reference, or a sum that overflowed: refuse it and stop instead.
        ++rejected_commands_;
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), kLogThrottleMs,
          "Non-finite command rejected (written as 0.0); %lu rejected since start",
          rejected_commands_);
      }
      last_command_[i] = command;  // 0.0 after a rejection, so a following gap holds a stop
    }

    // The tipover cutoff overrides everything above (reference, feedforward and the stale hold):
    // while disarmed the wheel is commanded 0.0 every cycle. The NaN reset of the reference above
    // still runs, so nothing stale is waiting when it is re-armed. last_command_ is zeroed too, so
    // a reference gap right after a re-arm holds a stop, not the command from before the fall.
    if (tipover_stop)
    {
      command = 0.0;
      last_command_[i] = 0.0;
    }

    if (!command_interfaces_[i].set_value(command))
    {
      all_writes_ok = false;
      ++failed_command_writes_;
    }
  }

  if (all_writes_ok)
  {
    consecutive_write_failures_ = 0;
    return controller_interface::return_type::OK;
  }

  // One failed cycle is tolerated; kMaxConsecutiveWriteFailures in a row is not.
  if (consecutive_write_failures_ < kMaxConsecutiveWriteFailures)
  {
    ++consecutive_write_failures_;
  }
  // Throttled: this runs every 10 ms and a persistent fault must not flood the log.
  RCLCPP_ERROR_THROTTLE(
    get_node()->get_logger(), *(get_node()->get_clock()), kLogThrottleMs,
    "set_value() failed on a command interface: %u cycles in a row, %lu failed writes since start",
    consecutive_write_failures_, failed_command_writes_);

  if (consecutive_write_failures_ >= kMaxConsecutiveWriteFailures)
  {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

// NOT CALLED YET: the live copy is inline in update_and_write_commands() (the guard hook refused the
// move, see docs/proposals/0003); edit both or neither.
bool AddFeedforwardController::tipover_tripped_this_cycle()
{
  if (!tipover_enabled_)
  {
    return false;
  }
  // Tipover cutoff (finding H-4): read pitch once, let the latch decide, log only on a change.
  // The trip and re-arm logs are not throttled but cannot repeat every cycle: a trip is logged once
  // (the latch then stays disarmed, so kTripped cannot come back until a re-arm), and a re-arm is
  // logged once per operator request. "Refused" has its own per-instance throttle
  // (refusal_log_age_cycles_), in case someone publishes /tipover_rearm in a loop. (rclcpp's
  // throttle would be wrong for all three: its state is shared by every instance of this class, so
  // the right wheel's line would be hidden behind the left one's.)
  if (refusal_log_age_cycles_ < kRefusalLogMinCycles)
  {
    ++refusal_log_age_cycles_;
  }
  // get_optional() gives nothing if the value could not be read this cycle; treat that as NaN
  // ("no reading"), which neither trips the cutoff nor allows a re-arm.
  const auto pitch_reading = state_interfaces_[0].get_optional();
  const double pitch_deg =
    pitch_reading.has_value() ? *pitch_reading : std::numeric_limits<double>::quiet_NaN();
  switch (tipover_latch_.update(pitch_deg, tipover_pitch_deg_, rearm_max_pitch_deg_))
  {
    case TipoverLatch::Event::kTripped:
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "TIPOVER CUTOFF TRIPPED: |pitch| = %.1f deg > %.1f deg. Wheel command zeroed and latched "
        "(trip %lu). Stand the robot up (|pitch| <= %.1f deg), then re-arm both wheels: "
        "ros2 topic pub --once -w 2 %s std_msgs/msg/Empty {}",
        std::fabs(pitch_deg), tipover_pitch_deg_, tipover_latch_.trips(), rearm_max_pitch_deg_,
        kRearmTopic);
      break;
    case TipoverLatch::Event::kRearmed:
      RCLCPP_WARN(
        get_node()->get_logger(), "Tipover cutoff RE-ARMED by operator at pitch %.1f deg.",
        pitch_deg);
      break;
    case TipoverLatch::Event::kRearmRefused:
      if (refusal_log_age_cycles_ >= kRefusalLogMinCycles)
      {
        refusal_log_age_cycles_ = 0;
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Tipover re-arm refused: pitch %.1f deg is not within +/-%.1f deg "
          "(rearm_max_pitch_deg), or unknown. Still disarmed.",
          pitch_deg, rearm_max_pitch_deg_);
      }
      break;
    case TipoverLatch::Event::kNone:
      break;
  }
  return !tipover_latch_.armed();
}

std::vector<hardware_interface::CommandInterface>
AddFeedforwardController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for (size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

controller_interface::return_type AddFeedforwardController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_buffer_ptr_.readFromRT();
  // message is valid
  if (!(!joint_commands || !(*joint_commands)))
  {
    if (reference_interfaces_.size() != (*joint_commands)->data.size())
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), kLogThrottleMs,
        "command size (%zu) does not match number of reference interfaces (%zu)",
        (*joint_commands)->data.size(), reference_interfaces_.size());
      return controller_interface::return_type::ERROR;
    }
    reference_interfaces_ = (*joint_commands)->data;
  }

  return controller_interface::return_type::OK;
}

}  // namespace passthrough_controller

PLUGINLIB_EXPORT_CLASS(
  rocko_env::AddFeedforwardController, controller_interface::ChainableControllerInterface)