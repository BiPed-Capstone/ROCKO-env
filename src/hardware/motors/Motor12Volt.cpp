

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

#include "include/rocko_env/Motor12Volt.hpp"

namespace rocko_env
{
hardware_interface::CallbackReturn Motor12Volt::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Check that we have a motor
  if (info.joints.size() != 1) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s' has %zu joints found. 1 expected.", info.name.c_str(),
      info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check that each motor has one command and one state
  for (hardware_interface::ComponentInfo joint : info.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interfaces found. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  _wheel.setup(info.joints[0].name, 0);

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
  // Set up pin directions

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