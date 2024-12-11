#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rocko_env/DriveWheel.hpp"
#include <wiringPi.h>
#include <softPwm.h>

namespace rocko_env
{
hardware_interface::CallbackReturn DriveWheel::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Check that we have pin names for speed and direction
  if (!(info.hardware_parameters.count(PIN_NUMBER_SPEED_KEY) > 0)) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s' does not have a '%s' parameter.", info.name.c_str(),
      PIN_NUMBER_SPEED_KEY.c_str()
    );
    return hardware_interface::CallbackReturn::ERROR;
  } else if (!(info.hardware_parameters.count(PIN_NUMBER_DIRECTION_KEY) > 0)) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s' does not have a '%s' parameter.", info.name.c_str(),
      PIN_NUMBER_DIRECTION_KEY.c_str()
    );
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check that we have a motor
  if (info.joints.size() != 1) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s' has %zu joints found. 1 expected.", info.name.c_str(),
      info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check that each motor has one command and two states
  for (hardware_interface::ComponentInfo joint : info.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    } else if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has a '%s' command interface. Expected a '%s' command interface.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interfaces found. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    } else if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has a '%s' state interface as the first interface. Expected a '%s' state interface.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    } else if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has a '%s' state interface as the second interface. Expected a '%s' state interface.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  _speedPin = stoi(info.hardware_parameters.at(PIN_NUMBER_SPEED_KEY));
  _dirPin = stoi(info.hardware_parameters.at(PIN_NUMBER_DIRECTION_KEY));
  _aPin = stoi(info.hardware_parameters.at(PIN_NUMBER_A_KEY));
  _bPin = stoi(info.hardware_parameters.at(PIN_NUMBER_B_KEY));

  _wheel.setup(info.joints[0].name, 0);

  // Set up pins to have WiringPi numberings
  wiringPiSetup();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveWheel::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  // Set up the speed pin to be PWM and the dir pin to be output
  pinMode(_speedPin, SOFT_PWM_OUTPUT);
  pinMode(_dirPin, OUTPUT);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DriveWheel::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _wheel.name, hardware_interface::HW_IF_POSITION, &_wheel.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DriveWheel::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DriveWheel::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set up PWM
  softPwmCreate(_speedPin, 0, SPEED_TO_PWM_COEFF);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveWheel::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop PWM here
  softPwmStop(_speedPin);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DriveWheel::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read the encoders to find speed and pos, and update the wheel object, which gets read when states are exported
  _wheel.vel = 0;
  _wheel.pos = _wheel.getPosition();

  RCLCPP_INFO(get_logger(), "Read %7.1f from %s encoder", _wheel.pos, _wheel.name.c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DriveWheel::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // handle any conversions here (assuming we have percent speed right now)
  int pwmVal = std::abs(_wheel.cmd * SPEED_TO_PWM_COEFF); // _wheel.cmd holds the speed we want to go

  // Set direction
  if (_wheel.cmd >= 0) {
    digitalWrite(_dirPin, HIGH);
  } else {
    digitalWrite(_dirPin, LOW);
  }

  // Set speed
  softPwmWrite(_speedPin, pwmVal);

  return hardware_interface::return_type::OK;
}

void DriveWheel::calcCurrentEncVal() {
    uint8_t p1val = digitalRead(_aPin);
    uint8_t p2val = digitalRead(_bPin);
    uint8_t s = _encState & 3;
    if (p1val) s |= 4;
    if (p2val) s |= 8;
    _encState = (s >> 2);
    
    switch (s) {
        case 1: case 7: case 8: case 14:
            _wheel.enc++;
            return;
        case 2: case 4: case 11: case 13:
            _wheel.enc--;
            return;
        case 3: case 12:
            _wheel.enc += 2;
            return;
        case 6: case 9:
            _wheel.enc -= 2;
            return;
    }
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::DriveWheel, hardware_interface::ActuatorInterface)
