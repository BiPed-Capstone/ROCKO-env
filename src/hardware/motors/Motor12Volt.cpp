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

#include "rocko_env/Motor12Volt.hpp"
#include "rocko_env/GPIOInterface.hpp"

namespace rocko_env
{
hardware_interface::CallbackReturn Motor12Volt::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Check that we have pin names for speed and direction
  if (!(info.hardware_parameters.count(PIN_NAME_SPEED_KEY) > 0)) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s' does not have a '%s' parameter.", info.name.c_str(),
      PIN_NAME_SPEED_KEY.c_str()
    );
    return hardware_interface::CallbackReturn::ERROR;
  } else if (!(info.hardware_parameters.count(PIN_NAME_DIRECTION_KEY) > 0)) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s' does not have a '%s' parameter.", info.name.c_str(),
      PIN_NAME_DIRECTION_KEY.c_str()
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

  _pinNameSpeed = info.hardware_parameters.at(PIN_NAME_SPEED_KEY);
  _pinNameDirection = info.hardware_parameters.at(PIN_NAME_DIRECTION_KEY);

  _wheel.setup(info.joints[0].name, 0);

  if (!GPIOInterface::getInstance().initSuccessful()) {
    RCLCPP_FATAL(get_logger(), "Python GPIO library failed to initialize");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Motor12Volt::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  bool result = GPIOInterface::getInstance().setupPin(_pinNameSpeed, true) && GPIOInterface::getInstance().setupPin(_pinNameDirection, true);
  if (!result) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Motor12Volt::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  bool result = GPIOInterface::getInstance().cleanup();
  if (!result) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Motor12Volt::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    _wheel.name, hardware_interface::HW_IF_POSITION, &_wheel.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Motor12Volt::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    _wheel.name, hardware_interface::HW_IF_VELOCITY, &_wheel.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn Motor12Volt::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bool result = GPIOInterface::getInstance().startPWM(_pinNameSpeed, 0, 2000, false) && GPIOInterface::getInstance().startPWM(_pinNameDirection, 0, 2000, false); // TODO: frequency and isFallingEdge?
  if (!result) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Motor12Volt::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Motor12Volt::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read info from hardware for all states and store them inside export_state_interfaces

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Motor12Volt::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // command the hardware to do things based on inputs in export_command_interfaces

  return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::Motor12Volt, hardware_interface::ActuatorInterface)
