#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rocko_env/Motor12VoltQuadEncoder.hpp"
#include <wiringPi.h>
#include <softPwm.h>

namespace rocko_env
{
  std::string joint_name_;

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    // Check that we have pin names for speed and direction
    if (!(info.hardware_parameters.count(PIN_NUMBER_SPEED_KEY) > 0))
    {
      RCLCPP_FATAL(
          get_logger(), "Hardware '%s' does not have a '%s' parameter.", info.name.c_str(),
          PIN_NUMBER_SPEED_KEY.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else if (!(info.hardware_parameters.count(PIN_NUMBER_DIRECTION_KEY) > 0))
    {
      RCLCPP_FATAL(
          get_logger(), "Hardware '%s' does not have a '%s' parameter.", info.name.c_str(),
          PIN_NUMBER_DIRECTION_KEY.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // The encoder parameters have no defaults on purpose: a wheel that quietly read the other wheel's
    // pins, or used a guessed radius, would report a wrong speed without any error (finding U-2 was
    // exactly that). Each must be present, a number with nothing after it, and believable.
    {
      // Reads parameter `key` into `out`; false if it is missing or is not entirely a number.
      const auto read_number = [&info](const std::string & key, double & out) {
        const auto it = info.hardware_parameters.find(key);
        if (it == info.hardware_parameters.end()) {
          return false;
        }
        try {
          std::size_t used = 0;
          out = std::stod(it->second, &used);
          return used == it->second.size() && std::isfinite(out);
        } catch (const std::exception &) {
          return false;
        }
      };
      double pin_a = -1.0, pin_b = -1.0, wheel_radius_m = 0.0, counts_per_wheel_rev = 0.0;
      // Pins are WiringPi numbers (not BCM or physical pin numbers): 0..31 covers the 40-pin header,
      // see `gpio readall` on the Pi.
      const auto is_pin = [](double pin) {return pin >= 0.0 && pin <= 31.0 && pin == std::floor(pin);};
      const char * bad_key = nullptr;
      if (!read_number(ENCODER_PIN_A_KEY, pin_a) || !is_pin(pin_a)) {
        bad_key = ENCODER_PIN_A_KEY.c_str();
      } else if (!read_number(ENCODER_PIN_B_KEY, pin_b) || !is_pin(pin_b) || pin_b == pin_a) {
        bad_key = ENCODER_PIN_B_KEY.c_str();
      } else if (!read_number(WHEEL_RADIUS_KEY, wheel_radius_m) || wheel_radius_m <= 0.0) {
        bad_key = WHEEL_RADIUS_KEY.c_str();
      } else if (!read_number(COUNTS_PER_REV_KEY, counts_per_wheel_rev) || counts_per_wheel_rev <= 0.0) {
        bad_key = COUNTS_PER_REV_KEY.c_str();
      }
      if (bad_key != nullptr)
      {
        RCLCPP_FATAL(
            get_logger(),
            "Hardware '%s': parameter '%s' is missing or wrong (pins: two different whole WiringPi numbers 0..31; "
            "radius and counts per turn: positive numbers).", info.name.c_str(), bad_key);
        return hardware_interface::CallbackReturn::ERROR;
      }
      _encoderPinA = static_cast<int>(pin_a);
      _encoderPinB = static_cast<int>(pin_b);
      _countsPerMetre = counts_per_metre(counts_per_wheel_rev, wheel_radius_m);
    }

    // Check that we have a motor
    if (info.joints.size() != 1)
    {
      RCLCPP_FATAL(
          get_logger(), "Hardware '%s' has %zu joints found. 1 expected.", info.name.c_str(),
          info.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check that each motor has one command and two states
    for (hardware_interface::ComponentInfo joint : info.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      else if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has a '%s' command interface. Expected a '%s' command interface.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interfaces found. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      else if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has a '%s' state interface as the first interface. Expected a '%s' state interface.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
      else if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has a '%s' state interface as the second interface. Expected a '%s' state interface.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    _speedPin = stoi(info.hardware_parameters.at(PIN_NUMBER_SPEED_KEY));
    _dirPin = stoi(info.hardware_parameters.at(PIN_NUMBER_DIRECTION_KEY));
    std::istringstream(info.hardware_parameters.at(INVERT_KEY)) >> std::boolalpha >> _invert;

    _wheel.setup(info.joints[0].name, 0);

    // Save joint name for topic selection
    joint_name_ = info.joints[0].name;

    // Set up pins to have WiringPi numberings
    wiringPiSetup();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // The encoder count moves on WiringPi's interrupt threads (one per pin, which WiringPi runs at
  // real-time priority 55 when the process is root) while read() loads it on the control thread.
  // It must never need a lock to read.
  static_assert(std::atomic<std::int64_t>::is_always_lock_free,
                "read() must load the encoder count without a lock");

  void Motor12VoltQuadEncoder::on_encoder_edge(struct WPIWfiStatus /*status*/, void * userdata)
  {
    // Runs on WiringPi's interrupt thread, NOT the control thread. Reads both pins NOW, like the
    // Python Encoder library did, and steps the decoder from the last state. The mutex is shared only
    // with the other pin's interrupt thread; read() never takes it, so it cannot delay the loop.
    auto * self = static_cast<Motor12VoltQuadEncoder *>(userdata);
    std::lock_guard<std::mutex> lock(self->_encoderMutex);
    const int state = quadrature_state(
      digitalRead(self->_encoderPinA) == HIGH, digitalRead(self->_encoderPinB) == HIGH);
    const int step = quadrature_step(self->_encoderState, state);
    self->_encoderState = state;
    self->_encoderCount.fetch_add(step, std::memory_order_relaxed);
  }

  void Motor12VoltQuadEncoder::stop_encoder_interrupts()
  {
    if (_encoderInterruptsOn)
    {
      // wiringPiISRStop cancels the pin's interrupt thread and joins it (pthread_cancel then
      // pthread_join, wiringPi.c in WiringPi 3.20), so after this no callback can be running or start.
      // The thread can only be cancelled while it waits for an edge, never inside our callback. The
      // count is kept.
      wiringPiISRStop(_encoderPinA);
      wiringPiISRStop(_encoderPinB);
      _encoderInterruptsOn = false;
    }
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_configure(
      const rclcpp_lifecycle::State & /* previous_state */)
  {
    // Set up the speed pin to be PWM and the dir pin to be output
    pinMode(_speedPin, SOFT_PWM_OUTPUT);
    pinMode(_dirPin, OUTPUT);

    // Encoder: inputs, the starting state, then one interrupt per pin on both edges. Edges are counted
    // from here on, active or not, so the position stays continuous across activate/deactivate.
    // No pull-up or pull-down is set (the encoder drives its lines; the Python code set none either).
    // The interrupts stay on from the first configure until cleanup or shutdown, so skip this if they
    // are already running.
    if (!_encoderInterruptsOn)
    {
      pinMode(_encoderPinA, INPUT);
      pinMode(_encoderPinB, INPUT);
      {
        std::lock_guard<std::mutex> lock(_encoderMutex);
        _encoderState = quadrature_state(
          digitalRead(_encoderPinA) == HIGH, digitalRead(_encoderPinB) == HIGH);
      }
      if (wiringPiISR2(_encoderPinA, INT_EDGE_BOTH, &Motor12VoltQuadEncoder::on_encoder_edge, 0, this) != 0)
      {
        RCLCPP_FATAL(get_logger(), "Could not start the interrupt on encoder pin %d (WiringPi number).", _encoderPinA);
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (wiringPiISR2(_encoderPinB, INT_EDGE_BOTH, &Motor12VoltQuadEncoder::on_encoder_edge, 0, this) != 0)
      {
        RCLCPP_FATAL(get_logger(), "Could not start the interrupt on encoder pin %d (WiringPi number).", _encoderPinB);
        wiringPiISRStop(_encoderPinA);
        return hardware_interface::CallbackReturn::ERROR;
      }
      _encoderInterruptsOn = true;
    }
    // The velocity starts from no history: the first read() reports 0.0, because there is no earlier
    // position to compare with.
    _velocityFilter.reset();
    _haveLastReadTime = false;

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_cleanup(
      const rclcpp_lifecycle::State & /* previous_state */)
  {
    stop_encoder_interrupts();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_shutdown(
      const rclcpp_lifecycle::State & /* previous_state */)
  {
    // Safeguard: ros2_control can shut a component down straight from active
    // without calling on_deactivate, and software PWM would keep driving the wheel at its last duty.
    stop_motor();
    stop_encoder_interrupts();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  void Motor12VoltQuadEncoder::stop_motor()
  {
    if (_pwmRunning)
    {
      softPwmWrite(_speedPin, 0);  // 0 % duty first, then end the PWM thread
      softPwmStop(_speedPin);
      _pwmRunning = false;
    }
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_error(
      const rclcpp_lifecycle::State & /* previous_state */)
  {
    // Safeguard: a component that fails while active must not leave the wheel at its last duty.
    stop_motor();
    stop_encoder_interrupts();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  Motor12VoltQuadEncoder::~Motor12VoltQuadEncoder()
  {
    // Safeguard: destroyed while still active (no shutdown transition) must not leave PWM running.
    stop_motor();
    // The interrupt callbacks were handed `this`; they must be gone before the object is.
    stop_encoder_interrupts();
  }

  std::vector<hardware_interface::StateInterface> Motor12VoltQuadEncoder::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        _wheel.name, hardware_interface::HW_IF_POSITION, &_wheel.pos));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> Motor12VoltQuadEncoder::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Set up PWM
    // A PWM thread that failed to start would leave a wheel that is never driven while the plugin
    // says active: refuse to activate instead (finding M-5: never ignore a return value).
    if (softPwmCreate(_speedPin, 0, 100) != 0)
    {
      RCLCPP_FATAL(get_logger(), "Could not start software PWM on speed pin %d (WiringPi number).", _speedPin);
      return hardware_interface::CallbackReturn::ERROR;
    }
    _pwmRunning = true;

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor12VoltQuadEncoder::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    stop_motor();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Motor12VoltQuadEncoder::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // No service call and no waiting: the interrupt threads keep the count, so this only loads it
    // (lock-free) and does arithmetic, and read() always returns within microseconds.
    // Units: position in metres of wheel travel, velocity in m/s, forward positive (the controllers
    // and their gains expect exactly this).
    const std::int64_t count = _encoderCount.load(std::memory_order_relaxed);
    // The time step is measured HERE, when the count is taken, not the controller's nominal `period`:
    // another component read earlier in the same cycle (the IMU's I2C burst, up to ~3.6 ms on the Pi)
    // moves the moment this count is taken without changing `period`.
    // steady_clock::now() is a cheap, non-blocking clock read on Linux.
    const auto now = std::chrono::steady_clock::now();
    const double dt_s = _haveLastReadTime ?
      std::chrono::duration<double>(now - _lastReadTime).count() : 0.0;
    _lastReadTime = now;
    _haveLastReadTime = true;
    _wheel.pos = static_cast<double>(count) / _countsPerMetre;
    _wheel.vel = _velocityFilter.update(_wheel.pos, dt_s);

    // Nothing here can fail the way the service call could. What is NOT detected: a dead or unplugged
    // encoder just stops counting, so the wheel looks stationary. Nothing detects that yet
    // (docs/next-steps.md row 1.10).
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Motor12VoltQuadEncoder::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Finding H-2: softPwmWrite() must be called on EVERY cycle. Software PWM keeps running at the
    // last duty it was given, so skipping the write for a small command leaves the previous
    // (possibly large) duty driving the wheel. A command near zero therefore writes 0 %.
    // No per-cycle logging here (finding M-1): this runs at 100 Hz on the critical path.
    const double radPerSec = _wheel.cmd;  // [rad/s] wheel speed we want; sign gives direction

    // NaN / inf: stop, and leave the direction pin alone (a NaN has no sign to follow).
    // A command inside the deadband is also a stop, with the direction pin unchanged.
    if (!std::isfinite(radPerSec) || std::abs(radPerSec) < kDeadbandRadPerSec)
    {
      softPwmWrite(_speedPin, 0);
      return hardware_interface::return_type::OK;
    }

    // Set direction
    if (radPerSec >= 0)
    {
      digitalWrite(_dirPin, !_invert);
    }
    else
    {
      digitalWrite(_dirPin, _invert);
    }

    // rad/s -> percent of full duty, plus the explicit stiction term. Clamped to [0, 100] (M-3)
    // so a command above MAX_RAD_PER_SEC cannot ask softPwm for more than full duty.
    const double dutyPercent = std::clamp(
      std::abs(radPerSec) / MAX_RAD_PER_SEC * 100.0 + kStictionDutyPercent, 0.0, 100.0);
    softPwmWrite(_speedPin, static_cast<int>(dutyPercent));  // softPwm takes whole percent

    return hardware_interface::return_type::OK;
  }

} // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rocko_env::Motor12VoltQuadEncoder, hardware_interface::ActuatorInterface)
