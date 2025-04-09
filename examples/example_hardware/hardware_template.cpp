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

// TODO: include your .hpp file

// TODO: Replace <CLASS_NAME> with your class name

namespace rocko_env
{

// This file does not contain all the methods described in https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
// because some of the methods are not needed in the current implementation
hardware_interface::CallbackReturn <CLASS_NAME>::on_init(
  const hardware_interface::HardwareInfo & info)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> <CLASS_NAME>::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  return state_interfaces;
}

hardware_interface::return_type <CLASS_NAME>::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

// TODO: Uncomment if implementing a motor

// std::vector<hardware_interface::CommandInterface> <CLASS_NAME>::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//   return command_interfaces;
// }

// hardware_interface::CallbackReturn <CLASS_NAME>::on_activate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn <CLASS_NAME>::on_deactivate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::return_type <CLASS_NAME>::write(
//   const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
// {
//   return hardware_interface::return_type::OK;
// }

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::<CLASS_NAME>, hardware_interface::SensorInterface) // TODO: Change this to ActuatorInterface if implementing a motor
